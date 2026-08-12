# 把 jetson_deploy 裡的東西推上 Jetson,並重啟整套服務。
#
# 用法(Jetson 開機、同網域之後):
#   powershell -ExecutionPolicy Bypass -File .\jetson_deploy\deploy.ps1
#   powershell -ExecutionPolicy Bypass -File .\jetson_deploy\deploy.ps1 -OnlyPush
#
# -OnlyPush 只傳檔不重啟,適合已經在跑、想手動控制的時候。

param(
    [string]$JetsonIp = "192.168.40.98",
    [string]$User     = "andykuo",
    [string]$Pw       = "2919",
    [switch]$OnlyPush
)

$ErrorActionPreference = "Stop"
$here    = Split-Path -Parent $MyInvocation.MyCommand.Path
$pscp    = "C:\Program Files\PuTTY\pscp.exe"
$plink   = "C:\Program Files\PuTTY\plink.exe"
$hostkey = "SHA256:ph8AvnetrS39dH8fnsuW8FPp9tsIg3wGecSXe/egfI4"
$target  = "$User@$JetsonIp"

function Push-File($local, $remote) {
    Write-Host "  -> $remote" -ForegroundColor DarkGray
    & $pscp -batch -hostkey $hostkey -pw $Pw $local "${target}:$remote" | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "pscp failed: $local" }
}

function Run-Remote($script) {
    $b = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($script))
    & $plink -ssh -batch -hostkey $hostkey -pw $Pw $target "echo $b | base64 -d | bash"
}

Write-Host "=== 傳檔 ===" -ForegroundColor Cyan
Run-Remote "mkdir -p ~/calib_view ~/slam2d ~/lidar_web" | Out-Null

Push-File "$here\lidar_web\server.py"          "/home/$User/lidar_web/server.py"
Push-File "$here\lidar_web\index.html"         "/home/$User/lidar_web/index.html"
Push-File "$here\scripts\stop_nvblox.sh"       "/home/$User/stop_nvblox.sh"
Push-File "$here\scripts\purge_nvblox.sh"      "/home/$User/purge_nvblox.sh"
Push-File "$here\scripts\startall.sh"          "/home/$User/startall.sh"
Push-File "$here\scripts\calib_base_link.py"   "/home/$User/calib_base_link.py"
Push-File "$here\scripts\calib_extrinsic.py"   "/home/$User/calib_extrinsic.py"
Push-File "$here\scripts\base_link_tf.sh"      "/home/$User/slam2d/base_link_tf.sh"
Push-File "$here\scripts\start_slam2d.sh"      "/home/$User/slam2d/start_slam2d.sh"
Push-File "$here\calib_view\overlay_server.py" "/home/$User/calib_view/overlay_server.py"
Push-File "$here\calib_view\start_overlay.sh"  "/home/$User/calib_view/start_overlay.sh"

# Windows 的 CRLF 會讓 bash 在 shebang 和每一行結尾出錯,而錯誤訊息完全看不出
# 原因(通常是 "$'\r': command not found")。傳完一律洗掉。
Write-Host "=== 正規化換行 + 語法檢查 ===" -ForegroundColor Cyan
Run-Remote @'
cd ~
SH="stop_nvblox.sh purge_nvblox.sh startall.sh slam2d/base_link_tf.sh
    slam2d/start_slam2d.sh calib_view/start_overlay.sh"
PY="lidar_web/server.py calib_base_link.py calib_extrinsic.py
    calib_view/overlay_server.py"
for f in $SH $PY lidar_web/index.html; do sed -i 's/\r$//' "$f"; done
for f in $SH; do bash -n "$f" && echo "  OK  $f"; done
for f in $PY; do python3 -m py_compile "$f" && echo "  OK  $f"; done
chmod +x $SH
'@

if ($OnlyPush) {
    Write-Host "`n只傳檔,未執行任何東西。" -ForegroundColor Yellow
    Write-Host "接下來照 README_部署.md 的『執行步驟』一步一步跑。"
    exit 0
}

Write-Host "`n=== 拆掉 nvblox(容器,不含映像檔)===" -ForegroundColor Cyan
Run-Remote "bash ~/stop_nvblox.sh"

Write-Host "`n=== 重啟整套(約 2 分鐘)===" -ForegroundColor Cyan
Run-Remote "bash ~/startall.sh"

Write-Host "`n完成。瀏覽器開:" -ForegroundColor Green
Write-Host "  http://${JetsonIp}:8080   三維彩色點雲"
Write-Host "  http://${JetsonIp}:8090   二維地圖 + 導航"
Write-Host "  http://${JetsonIp}:8092   相機影像"
Write-Host "`n完整清除映像檔與 workspace,以及 base_link 校準,"
Write-Host "請照 README_部署.md 的『執行步驟』手動跑 —— 那些不該自動化。"
