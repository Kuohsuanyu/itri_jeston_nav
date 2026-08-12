# 用 cmd 連 Jetson 並檢查感測器

這份裡的每一條指令都在 2026-08-10 的 Jetson 上實際跑過,輸出是真的。

---

## 一、連線

Windows 10/11 內建 OpenSSH,不用裝 PuTTY。開 **cmd**(或 Windows Terminal):

```cmd
ssh andykuo@192.168.40.98
```

第一次會問:

```
The authenticity of host '192.168.40.98' can't be established.
ED25519 key fingerprint is SHA256:ph8AvnetrS39dH8fnsuW8FPp9tsIg3wGecSXe/egfI4.
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

指紋對得上就打 `yes`。之後存進 `%USERPROFILE%\.ssh\known_hosts`,不會再問。

密碼:`2919`

離開:`exit` 或 <kbd>Ctrl</kbd>+<kbd>D</kbd>

### 免密碼(建議)

打密碼打十次就會煩,而且腳本沒辦法自動化。用金鑰:

```cmd
ssh-keygen -t ed25519 -C "windows-laptop"
```

一路 Enter(passphrase 留空)。然後把公鑰送上去:

```cmd
type %USERPROFILE%\.ssh\id_ed25519.pub | ssh andykuo@192.168.40.98 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```

之後 `ssh andykuo@192.168.40.98` 直接進去。

### 不進去,只跑一條指令

```cmd
ssh andykuo@192.168.40.98 "free -m"
```

適合寫進 .bat 做健康檢查。

---

## 二、進去之後先 source

**每開一個新的 SSH 連線都要重來一次** —— 這是最常見的「為什麼 ros2 找不到指令」。

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export ROS_DOMAIN_ID=0
```

打三行很煩,加進 `~/.bashrc` 一勞永逸:

```bash
cat >> ~/.bashrc <<'EOF'
alias rosenv='source /opt/ros/humble/setup.bash; source ~/ws_livox/install/setup.bash; export ROS_DOMAIN_ID=0; echo "ROS 2 Humble  DOMAIN_ID=0"'
EOF
```

之後每次連進去打 `rosenv` 就好。

> 不直接把 source 寫進 .bashrc 而是做成 alias,是因為 source ROS 會拖慢每一個
> 非互動 shell(scp、rsync 都會受影響),而且出錯時很難查。

---

## 三、看有什麼

```bash
ros2 topic list          # 所有 topic
ros2 node list           # 所有節點
ros2 node info /laser_mapping    # 某個節點訂了什麼、發了什麼
```

本機目前應該看到:

```
/livox/lidar             光達原始點雲
/livox/imu               光達內建 IMU
/cloud_registered        FAST-LIO 配準後(世界座標)
/cloud_registered_body   同上,但感測器座標
/Odometry                FAST-LIO 的位姿
/scan                    壓成二維的雷射
/map                     slam_toolbox 的佔據地圖
/camera/camera/...       RealSense
```

---

## 四、檢查資料有沒有在流

### 頻率 —— 最常用

```bash
ros2 topic hz /livox/imu
```

```
average rate: 199.964
	min: 0.001s max: 0.014s std dev: 0.00128s window: 201
```

<kbd>Ctrl</kbd>+<kbd>C</kbd> 停止。

| topic | 應該是 |
|---|---|
| `/livox/imu` | 200 Hz |
| `/livox/lidar` | 10 Hz |
| `/cloud_registered` | 10 Hz |
| `/scan` | 10 Hz |
| `/camera/camera/color/image_raw` | 15 Hz |

**沒有輸出 = 沒有資料。** 不是指令壞掉,是真的沒東西在發。

### 頻寬

```bash
ros2 topic bw /cloud_registered
```

```
1.96 MB/s from 69 messages
	Message size mean: 0.20 MB min: 0.19 MB max: 0.20 MB
```

WiFi 傳不動的時候先看這個。

### 內容 —— 只看想看的欄位

整包 echo 會刷屏(點雲有幾萬個點),用 `--field`:

```bash
ros2 topic echo /Odometry --once --field pose.pose.position
```

```
x: -0.02602015749868361
y: 0.01701218796657859
z: -0.024053663781636106
```

`--once` 收一筆就結束。

### 誰在發、用什麼 QoS

```bash
ros2 topic info /livox/lidar --verbose
```

```
Type: livox_ros_driver2/msg/CustomMsg
Publisher count: 2
  Node name: livox_lidar_publisher
  QoS profile:
    Reliability: RELIABLE
    Durability: VOLATILE
```

**`Publisher count` 大於 1 要留意** —— 通常代表有東西在重複發布,可能是 bridge 迴圈。

---

## 五、TF

```bash
ros2 run tf2_ros tf2_echo map base_link
```

看不到就一段一段查,哪一段斷掉一目了然:

```bash
ros2 run tf2_ros tf2_echo odom camera_init
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_ros tf2_echo body base_link
ros2 run tf2_ros tf2_echo map odom
```

`map → odom` 是 slam_toolbox 的修正量。平常接近 0,迴路閉合時會跳。

---

## 六、不用 ROS 的檢查

```bash
ping -c3 192.168.0.50            # 光達通不通
ip -4 -o addr show enP8p1s0      # 網卡有沒有拿到 192.168.0.100
lsusb | grep -i intel            # RealSense 有沒有被認到
free -m                          # 記憶體(Jetson 只有 7.6 GB,吃緊過)
tegrastats                       # GPU / CPU / 溫度,Ctrl+C 停
df -h /mnt/ssd                   # SSD 空間
uptime                           # 開機多久、負載
```

看行程:

```bash
pgrep -af fastlio                # 找特定行程
htop                             # 互動式(q 離開)
```

---

## 七、看 log

啟動腳本把每個服務的輸出丟到 `/tmp`:

```bash
tail -20 /tmp/fastlio.log
tail -20 /tmp/slam2d.log
tail -20 /tmp/livox.log
tail -f  /tmp/webviewer.log      # -f = 持續跟著看,Ctrl+C 停
```

只看錯誤:

```bash
grep -iE "error|warn|fail" /tmp/slam2d.log | tail -20
```

---

## 八、踩過的坑

**`ros2 topic hz` 不吃 `--qos-profile`**(這一版)

```bash
ros2 topic hz /cloud_registered --qos-profile sensor_data
# ros2: error: unrecognized arguments: --qos-profile sensor_data
```

`ros2 topic echo` 才有 QoS 參數。`hz` 就直接跑,大多數 topic 都能對上。

**BEST_EFFORT 的發布端配 RELIABLE 的訂閱端會收不到。** `echo` 遇到會自動降級並印出提示:

```
Some, but not all, publishers are offering QoSReliabilityPolicy.RELIABLE.
Falling back to BEST_EFFORT as it will connect to all publishers
```

**`/tmp` 開機會清空。** 常用腳本放家目錄,不要放 `/tmp`。

**指令裡有中文,用 base64 傳。** 直接 `ssh host "echo 中文"` 在編碼不一致時會壞掉:

```cmd
ssh andykuo@192.168.40.98 "echo <base64字串> | base64 -d | bash"
```

---

## 九、一分鐘健康檢查

連進去之後貼這一段:

```bash
rosenv
for t in /livox/imu /livox/lidar /cloud_registered /scan; do
  echo -n "$t  "
  timeout 4 ros2 topic hz $t 2>/dev/null | head -1 || echo "沒有資料"
done
free -m | head -2
```
