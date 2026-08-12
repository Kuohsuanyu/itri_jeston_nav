# 移除 nvblox,改用光達點雲直接上色

暫存區。Jetson 不在手邊時先在這裡改完,接上之後一鍵推上去。

---

## 為什麼換掉 nvblox

| | nvblox | 現在這個做法 |
|---|---|---|
| 幾何來源 | D435 深度,融合距離上限 4 m | 光達,40 m / 360° |
| 顏色來源 | D435 彩色 | D435 彩色(同一個) |
| 對導航的貢獻 | **零**(ESDF 從未接進 Nav2 costmap) | 零(導航一樣吃 :8090 的二維圖) |
| 額外 RAM | 約 660 MB + GPU | 約 0,併進本來就在跑的 server.py |
| 相依 | Isaac ROS 3.2 容器全套 | 無 |

nvblox 唯一獨佔的能力是「表面上色」,而那件事用**把光達點投影進彩色影像**就能做到,
而且幾何品質更好 —— 因為幾何是光達給的,不是 4 公尺的深度相機。

沒有丟掉的東西:`/mnt/ssd/ws` 和 Isaac 映像檔保留。重建要花掉大半天,
之後若要做動態避障還用得到。

---

## 改了什麼

### `lidar_web/server.py`
- 每點從 4 個 float32 變 5 個:`x, y, z, intensity, rgb`
- rgb 打包成單一 float(`r*65536 + g*256 + b`)。float32 尾數 24 bit,
  裝得下 0~16777215,拆回來精確。**只多 4 bytes/點**,不用擴成三個 float。
- 新增訂閱:`color/image_raw`、`color/camera_info`、`aligned_depth_to_color/image_raw`
- 新增 TF listener,查 `世界座標 -> camera_color_optical_frame`
- 沒有顏色的點存 `-1`,瀏覽器端會退回高度上色並壓暗

### `lidar_web/index.html`
- 新增「相機彩色」模式(預設)
- 新增「已上色」統計列
- shader 拆 rgb 打包值;`-1` 的點壓到 30% 亮度

### `scripts/startall.sh`
- 開頭先清掉殘留的 Isaac 容器(它會佔住 D435 的 USB)
- 相機提前到第 4 步、web viewer 移到第 6 步 —— server.py 要先拿到內參
- 結尾多一段「上色前置條件檢查」,直接告訴你三件事有沒有到位

### `scripts/stop_nvblox.sh`
- 移除 `nvblox` / `nvblox_mesh_web` 容器,釋放相機與記憶體

---

## 上色的三個關鍵設計

**1. 影像要留一串,不能只留最新一張。**
相機用「現在時間」蓋時戳,但 FAST-LIO 的 TF 落後 0.15~0.3 秒,
所以查最新影像時戳的 TF 幾乎必定失敗。
留 12 張(0.8 秒),由新到舊挑第一個查得到 TF 的來用。

> 這正是 nvblox 建不出圖的原因:它的 `maximum_input_queue_length` 預設 3,
> 只有 0.2 秒緩衝,深度幀還沒等到 TF 就被丟光。當時我用
> `rclpy.time.Time()`(最新可用)去驗 TF,那個查詢**永遠成功**,
> 所以一直誤判「TF 沒問題」。

**2. 要用對齊深度做遮蔽測試。**
沒有這道檢查,牆壁的顏色會被刷到牆後面的物體上 —— 投影只算「這個點落在哪個像素」,
不管中間有沒有東西擋著。`align_depth.enable:=true` 讓深度和彩色共用像素格,
所以直接查同一個 (u,v) 比一比距離就行。
深度讀到 0(黑色、反光、太近)時放行,否則大片深色表面會永遠是灰的。

**3. 距離要設上限(5 m)。**
外參的角度誤差會被距離線性放大。校準殘差 5 cm、角度誤差約 1°,
在 5 m 處錯位約 9 cm —— 還可接受;10 m 就是 17 cm,顏色會明顯貼錯物體。
寧可留白,等推近了再補上。

---

## 部署(Jetson 接上之後)

```powershell
cd c:\Users\ag133\Desktop\JetsonConsole
powershell -ExecutionPolicy Bypass -File .\jetson_deploy\deploy.ps1
```

會依序做:傳檔 → 洗掉 CRLF → 語法檢查 → 拆 nvblox → 重啟整套 → 印出檢查結果。

只想傳檔不重啟:加 `-OnlyPush`。

---

## 驗收

開 `http://192.168.40.98:8080`,左上「已上色」那一列。

| 現象 | 意思 |
|---|---|
| 百分比隨著轉動慢慢爬升 | 正常。相機掃過的地方才有顏色 |
| 一直 0% | TF 斷了。看 `/tmp/cam_extrinsic.log` 和 `/tmp/webviewer.log` |
| 有顏色但明顯貼錯位置 | 外參跑掉了,重跑 `python3 ~/calib_extrinsic.py` |
| 顏色穿透牆壁 | `align_depth` 沒開,遮蔽測試失效 |

正常情況下顏色只會覆蓋相機視野掃過的區域,其餘維持壓暗的高度上色。
這是刻意的 —— 一眼就看得出哪裡已經記錄過顏色。

---

## 還沒處理:光達傾斜 30° 對二維地圖的影響

`~/slam2d/start_slam2d.sh` 裡的 `pointcloud_to_laserscan` 用
`target_frame: base_link`、`min_height:=-0.35 max_height:=0.60`。

但 `base_link` 就在光達上,而光達**機構上斜 30°**(校準量到 31~32°,CAD 標 30°)。
所以那個「水平切片」其實是斜的 —— 掃出來的 2D 地圖和 Nav2 的 costmap
都帶著這個傾斜誤差。

修法是在 `base_link` 底下掛一個補正 −30° pitch 的靜態子座標系
(例如 `base_footprint_level`),讓 `pointcloud_to_laserscan` 以它為 target。

**還沒動手**,因為這會改到已經在跑的導航展示的座標鏈。要改再說。
