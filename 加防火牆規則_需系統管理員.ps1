# =====================================================================
#  讓 WSL 的 RViz2 收得到 Jetson 的 ROS 2 資料
#
#  用法:對這個檔案按右鍵 →「以 PowerShell 執行」
#        若跳出 UAC 請按「是」。只需要執行一次。
#
#  為什麼需要:
#    WSL 開了 mirrored 網路模式後,和 Windows 共用同一個 IP,
#    因此入站封包要通過 Windows 主機防火牆。
#    Windows 預設拒絕所有「未經請求的」入站 UDP。
#
#    實測結果(tcpdump 兩端對照):
#      Jetson 端  → 有送出封包到 192.168.40.222:7410
#      WSL 端     → 只看得到 Out,零個 In
#    也就是說 Jetson 的 DDS 回應在 Windows 這一層就被丟掉了,
#    ROS 2 的參與者探索因此永遠無法完成。
#
#  這支腳本開放 UDP 7400-7700(ROS 2 / DDS 的探索與資料埠),
#  只對私人網路生效。
# =====================================================================

$ErrorActionPreference = 'Stop'

# --- 自動提權 ---
$isAdmin = ([Security.Principal.WindowsPrincipal] `
    [Security.Principal.WindowsIdentity]::GetCurrent()
    ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

if (-not $isAdmin) {
    Write-Host "需要系統管理員權限,正在重新啟動..." -ForegroundColor Yellow
    Start-Process powershell.exe -Verb RunAs `
        -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

Write-Host ""
Write-Host "=== 設定 ROS 2 / DDS 防火牆規則 ===" -ForegroundColor Cyan
Write-Host ""

# --- 1. 主機防火牆:允許 DDS 入站 ---
$name = "ROS2 DDS inbound (WSL)"
Get-NetFirewallRule -DisplayName $name -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName $name `
    -Direction Inbound -Protocol UDP -LocalPort 7400-7700 `
    -Action Allow -Profile Private, Domain | Out-Null
Write-Host "[1/2] 主機防火牆規則已建立:UDP 7400-7700 入站允許" -ForegroundColor Green

# --- 2. Hyper-V 防火牆:同樣放行(WSL 專用那一層) ---
$vm = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'   # WSL 的固定 VM Creator ID
try {
    Get-NetFirewallHyperVRule -Name "ROS2-DDS-WSL-In" -ErrorAction SilentlyContinue |
        Remove-NetFirewallHyperVRule
    New-NetFirewallHyperVRule -Name "ROS2-DDS-WSL-In" `
        -DisplayName "ROS 2 DDS inbound (WSL)" `
        -Direction Inbound -VMCreatorId $vm `
        -Protocol UDP -LocalPorts 7400-7700 -Action Allow | Out-Null
    Write-Host "[2/2] Hyper-V 防火牆規則已建立" -ForegroundColor Green
} catch {
    Write-Host "[2/2] Hyper-V 規則略過(.wslconfig 已設 firewall=false,不影響)" -ForegroundColor DarkGray
}

Write-Host ""
Write-Host "完成。接下來:" -ForegroundColor Cyan
Write-Host "  1. 執行  wsl --shutdown"
Write-Host "  2. 開啟控制台,確認「ROS 2 連線」卡片出現 /cloud_registered"
Write-Host "  3. 按「開啟 RViz2」"
Write-Host ""
Read-Host "按 Enter 關閉"
