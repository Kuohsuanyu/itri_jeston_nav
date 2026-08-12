# wheeled-robot-lidar-nav

輪式機器人的光達建圖與導航系統。Jetson Orin Nano 跑感測與運算,Windows 筆電當控制台。

原本叫 `Desktop/JetsonConsole`,2026-08-08 搬到這裡並改名。

---

## 硬體

| | |
|---|---|
| 底盤 | ITRI DD-M(差動輪,輪距 0.6221 m,輪半徑 0.2032 m / 16 吋) |
| 運算 | Jetson Orin Nano Super 8GB,`andykuo@192.168.40.98` |
| 光達 | Livox Mid-360,機構上斜約 30°,IP 192.168.0.50 |
| 相機 | RealSense D435F,只當彩色來源 |
| 控制台 | Windows 筆電,`console.py` + `ui.html` |

---

## 資料夾

```
console.py / ui.html          Windows 控制台(SSH 遠端啟停 + 健康監控)
chassis/
  chassis-ros2-driver/        ITRI 原廠 repo(URDF + STL + 驅動)
    chassis_description/urdf/
      chassis_DD-M.xacro          原廠底盤,不要改
      qbot_sensors.xacro          ★ 感測器掛載參數
      qbot_preview.urdf           自動產生,給檢視器用
  place_tool/                 這台車專用的感測器擺放工具  :8095
  gen_preview_urdf.py         xacro -> 純 URDF
  watch_urdf.py               存檔自動重產預覽
  chassis_console.py          底盤測試介面(未部署)
tools/
  urdf_sensor_studio/         ★ 通用工具:任意 URDF 都能擺感測器並匯出
jetson_deploy/                要推到 Jetson 的檔案 + 執行步驟
wsl/                          WSL 端 RViz 與 zenoh 設定
問題排除紀錄.md                踩過的坑
```

---

## 座標系

這是整個系統最容易出錯的地方,先看這裡。

```
map ──(slam_toolbox)── odom ──(FAST-LIO)── base_footprint
                                                │  URDF,固定 z=0.2032
                                            base_link          ← 四輪中心、輪軸高度
                                                ├── wiring_box     理線盒 200×200×160
                                                ├── body           Mid-360 的 IMU 座標系
                                                └── camera_link    D435F
```

**`base_link` 不在地上,在輪軸上(離地 0.2032 m)。** URDF 裡填的每個 `Z` 都要加 0.2032 才是離地高度。

三個高度基準:

| 基準 | 離地 |
|---|---|
| 地面 / `base_footprint` | 0 |
| `base_link` 輪軸 | 0.2032 |
| 上蓋平面 | 0.4542 |
| 理線盒頂 | 0.6142 |

---

## 工具

### 通用:URDF Sensor Studio

任意 URDF 都能用。在 cmd 執行:

```
tools\urdf_sensor_studio\run.cmd
```

會問 URDF 路徑(可以直接把檔案拖進視窗),然後開瀏覽器。可以新增方塊感測器、匯入 STL、調位置與角度,最後輸入輸出資料夾匯出。

匯出的資料夾是**自足的** —— 新 URDF + 所有 mesh 複製一份 + 原始檔備份(檔名帶 `.original`),整包可以搬走或寄給別人。

只吃純 `.urdf`。xacro 請先展開。

### 這台車專用:place_tool

```
python chassis\place_tool\server.py        →  http://127.0.0.1:8095/
```

直接讀寫 `qbot_sensors.xacro`,支援用基準面量距離自動解座標、復原、自動備份到 `urdf/backup/`。

### 預覽自動更新

```
python chassis\watch_urdf.py
```

存檔 `qbot_sensors.xacro` 就自動重產 `qbot_preview.urdf`。

---

## Jetson 部署

看 [jetson_deploy/執行步驟.md](jetson_deploy/執行步驟.md)。順序不能反:

```
清 nvblox ──> 起服務 ──> base_link ──> 相機外參 ──> 疊圖驗證 ──> 彩色點雲
```

`base_link` 是所有東西的基準,它沒定好,外參是錯的、二維地圖有尺度誤差、Nav2 的 costmap 也跟著歪。

### 網頁介面(Jetson 上)

| 埠 | 內容 |
|---|---|
| 8080 | 三維彩色點雲 |
| 8090 | 二維地圖 + 導航 |
| 8092 | 相機影像 |
| 8094 | 外參疊圖驗證 |

---

## 箱子離線校準(2026-08-10)

導航箱還沒上車時,可以先把**箱子內部**那段量完 —— 那是剛性的,鎖上車不會變:

```
base_link ──(上車才知道:x/y 捲尺 + 盒底 0.2510)── 盒底安裝面 ──(★ 已量完)── body / camera_link
```

```
python3 ~/calib_box.py --yaw        # 箱子底面朝下,正放在開闊的地板上,30 秒不要碰
```

基準是**光達擬合出來的安裝面**,不是重力 —— 要的是 body 相對盒底的姿態,
所以地板歪不歪都不影響結果。IMU 只當交叉檢查。

實測結果(三個獨立來源互相驗證通過):

| | |
|---|---|
| 光達傾角 | 29.69°(標稱 30) |
| 光達離盒底 | 0.2005 m → 支架 4.05 cm |
| IMU 重力 vs 地面法線 | 差 0.94° |
| 相機 pitch 換算後 | −1.59°(平視相機應為 0) |
| 光達 vs 深度相機測距 | 中位差 +5.1 mm,87.7% 落在 5 cm 內 |

已寫進 `qbot_sensors.xacro`。還沒定的只剩 `LIDAR_X` / `LIDAR_Y`(上車捲尺量)
和 `LIDAR_YAW`(把箱子某一面貼平直牆再跑一次 `--yaw`)。

---

## 導航校準已套用(2026-08-10)

箱子上車後,`~/slam2d/base_link_tf.sh` 已填入實測值並生效。**最關鍵的修正是 `odom`**:

```
              修正前                    修正後
camera_init 離重力    31.28 度      odom 離水平 0.42 度
二維投影尺度誤差       14.5%              0.00%
```

FAST-LIO 的世界座標系 `camera_init` 是「開機那一刻光達的姿態」,光達斜 30° 它就斜 30°。
slam_toolbox 把軌跡投影到那個斜平面做二維定位,**地上走 1 m 只記 0.855 m** ——
繞一圈回不到原點,迴路閉合必然失敗。這跟相機一點關係都沒有,是純光達導航的前提。

`base_link` 的定義**跟原廠 URDF 一致:輪軸上,離地 0.2032 m**(不是舊版
`calib_base_link.py` 假設的地面上,兩者差 0.2032)。`start_slam2d.sh` 的
`/scan` 高度帶跟著改成 `-0.1032 ~ 1.2968`(= 離地 0.10 ~ 1.50)。

`BL_*` 不是現場用 IMU 算的,是由 `qbot_sensors.xacro` 的 `LIDAR_*` 反推 ——
那組角度綁在**車體**上,車停斜坡也不會漂。驗證:把現場重力用那組角度轉進
base_link,離垂直 1.48°(= 現場地板坡度,不是誤差)。

---

## 已知待辦

- **`/scan` 有抖動**:車子靜止時同一條射線的距離標準差中位 0.12 m,每幀只有
  64.7% 的角度格有回波。原因是 Mid-360 的**非重複掃描** —— 每幀取樣方向都不同,
  0.5° 的格子每幀被不同的物理點填。不是校準問題。要改善就加大
  `angle_increment` 或累積數幀再轉 `/scan`
- `LIDAR_YAW` 還沒量 —— 純光達 SLAM 不受影響(yaw 只決定「哪邊叫前面」),
  但底盤驅動接上要送 `cmd_vel` 時就會有影響
- `LIDAR_X` / `LIDAR_Y` 是暫定值,捲尺量。改的時候 `CAM_X` / `CAM_Y`
  要照 xacro 註解裡的式子一起改(兩個工具看不懂 xacro 運算式)
- URDF 還沒被 `robot_state_publisher` 載進 ROS。要載的時候必須同時關掉
  FAST-LIO 自己的 TF 並改用 `fastlio_odom_tf.py`,否則 `body` 會有兩個父節點
- `startall.sh` 不會殺掉舊的 FAST-LIO 和 livox 驅動,重跑會變成兩份同時發布
  (2026-08-10 踩到:FAST-LIO 發散後重跑 startall 沒換掉它,兩個 livox 驅動
  同時往 `/livox/lidar` 發)。重啟前先確認舊的都清乾淨
- `fastlio_odom_tf.py` 寫好但未部署 —— 改用 URDF 之後,FAST-LIO 自己的 TF 必須關掉,否則 `body` 會有兩個父節點
- Nav2 的 `footprint` 已按實際車體改成 1.04 × 0.78 m(原本 `robot_radius: 0.20` 差了三倍)
- 底盤驅動尚未接上,`publish_tf` 要設 `false`(FAST-LIO 當里程計來源)
