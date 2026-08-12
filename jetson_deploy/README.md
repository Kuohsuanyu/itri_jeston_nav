# jetson_deploy —— 跑在 Jetson 上的東西

**這個 repo 就是 Jetson 上的正本。** 2026-08-12 起,Jetson 上的
`~/slam2d`、`~/nav2`、`~/chassis_test` 都是指向這裡的符號連結,
不再有「Windows 一份、Jetson 一份」的問題。

編輯方式:VS Code Remote-SSH 連 `andykuo@192.168.40.98`,開 `~/qbot`。
存檔即生效 —— 但**行程是啟動時讀設定的,要重啟才會用到新的**(見下)。

## 目錄

| 目錄 | Jetson 上的路徑 | 內容 |
|---|---|---|
| `scripts/` | `~/slam2d` | TF、里程計融合、SLAM、校準 |
| `nav2/` | `~/nav2` | Nav2 參數與行為樹 |
| `chassis_test/` | `~/chassis_test` | 底盤遙控網頁(:8091) |
| `web/` | — | 點雲(:8080)和相機的網頁檢視 |
| `archive/` | — | 已退役,只為查歷史 |

## 啟動順序

```bash
bash ~/slam2d/startall.sh             # 光達 + FAST-LIO + 相機 + 網頁
bash ~/slam2d/start_slam2d.sh         # TF + EKF + /scan + slam_toolbox
bash ~/slam2d/start_zenoh_jetson.sh   # 給 WSL 的 RViz 用
bash ~/nav2/start_nav2_control.sh     # Nav2(要先有 /map)
```

底盤(樹莓派 192.168.40.160)要自己先起 bringup。`robot_tf.sh` 會遠端把
它的 `publish_tf` 關掉 —— 那個參數**重開機不保存**,所以每次都重設一次。

## 改了什麼就要重啟什麼

存檔不等於生效。2026-08-11 實測:`slam_params.yaml` 15:20 改好,而跑著的
slam_toolbox 是 14:47 啟動的,用著舊的 `base_frame` 跑了一個多小時。

| 改了 | 重跑 |
|---|---|
| `robot_tf.sh` `odom_cov_relay.py` `ekf_multi.yaml` `slam_params.yaml` `start_slam2d.sh` `tf_static_repeat.py` | `bash ~/slam2d/start_slam2d.sh` |
| `nav2/*.yaml` `bt_*.xml` | `bash ~/nav2/start_nav2_control.sh` |
| `start_zenoh_jetson.sh` `fastdds_peers.xml` | `bash ~/slam2d/start_zenoh_jetson.sh` |
| `startall.sh`、`mid360.yaml` | `bash ~/slam2d/startall.sh` |

**驗證的時候查跑著的行程,不要查檔案:**

```bash
ros2 param get /slam_toolbox base_frame
ros2 param get /chassis_driver publish_tf
ps -eo lstart,cmd | grep '[a]sync_slam_toolbox'
```

## 座標樹

```
map ─(slam_toolbox)─ multi_odom ─(EKF)─ base_footprint ─(底盤rsp)─ base_link ─┬─ 四個輪子
                                                                               └─ box_link ─┬─ body
                                                                                            └─ camera_link
```

一棵樹,沒有雙父節點。`base_footprint` 的父節點只能有一個,所以底盤必須讓出
`odom -> base_footprint`(`localization_mode:=external_takeover`,或執行期把
`publish_tf` 設成 false),由 EKF 接手。

**tf2 對雙父節點不報錯**,它會在兩個答案之間隨機翻轉 —— 症狀看起來像里程計在飄。

## 校準值(全部實測,不是機構圖)

在 [`scripts/robot_tf.sh`](scripts/robot_tf.sh) 最上面:

| 值 | 數字 | 來源 |
|---|---|---|
| `BOX_Z` | 0.3705 | `calib_height.py` 地面擬合。STL 推論是 0.2510,**差 11.95 cm** —— 盒子和上蓋之間墊了東西 |
| `BODY_Z` | 0.2005 | `calib_box.py`,箱子單獨放地上量 |
| `BODY_PITCH` | 0.5181 (29.69°) | IMU 重力 / 地面法線 / 相機 pitch 三者交叉驗證 |
| `CAM_*` | — | ICP,殘差 12.9 → 4.6 cm |
| `BOX_X` `BOX_Y` `BODY_YAW` | **0(還沒量)** | X/Y 只平移地圖原點影響小;YAW 要貼牆量 |

改了要重跑 `bash ~/slam2d/start_slam2d.sh`,Windows 那邊再跑「更新模型.bat」讓 RViz 的模型跟上。

## 三套「車有多大」,沒有自動同步

| 用途 | 在哪 |
|---|---|
| Nav2 碰撞 | `nav2/nav2_control.yaml` 的 `footprint` |
| `/scan` 自我濾除 | `scripts/start_slam2d.sh` 的 `RANGE_MIN` |
| 視覺模型 | `chassis_description` 的 STL |

Nav2 **完全不讀 URDF**。要看兩者一不一致,在 RViz 加 `Polygon` 顯示
`/local_costmap/published_footprint`,疊在車體模型上對照。
