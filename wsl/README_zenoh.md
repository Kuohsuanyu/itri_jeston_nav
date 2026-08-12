# zenoh bridge —— 取代跨機 DDS

## 為什麼換掉直連 DDS

原本讓 WSL 的 Fast DDS 直接跟 Jetson 對話,實測會**先通後斷**:
剛重啟 Jetson 時 `/cloud_registered` 收得到 10Hz,幾分鐘後衰減到 0。

用 tcpdump 兩端對照確認,封包其實**有**進到 WSL(15 秒 6648 個),
但 Fast DDS 就是不把資料交付給訂閱者。已排除的原因:

- 防火牆 —— 封包有到,不是被擋
- `/dev/shm` 耗盡 —— 只用了 2%

換成 zenoh 之後這條路徑整個消失:WSL 改用 `ROS_DOMAIN_ID=1`,
跟 Jetson 的 domain 0 刻意分開,兩邊各自跑本機 DDS,
中間只有**一條 WSL 主動撥出的 TCP 連線**。
不需要任何 Windows 入站防火牆規則,也沒有 UDP 分片問題。

```
Jetson (ROS_DOMAIN_ID=0)                WSL (ROS_DOMAIN_ID=1)
  livox driver ─┐                         ┌─ RViz2
  FAST-LIO ─────┼─ zenoh-bridge ══TCP══> zenoh-bridge ─┘
  web viewer ───┘   -l tcp/0.0.0.0:7447   -e tcp/192.168.40.98:7447
```

## 安裝

兩端版本**必須一致**(這裡是 1.9.0)。Jetson 連得到 GitHub 但連不到
`download.eclipse.org`,所以走 GitHub release 二進位檔,不用 apt。

```bash
V=1.9.0
B=https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/$V

# WSL(x86_64)
curl -sSL -o z.zip $B/zenoh-plugin-ros2dds-$V-x86_64-unknown-linux-gnu-standalone.zip
unzip -o z.zip && sudo install -m755 zenoh-bridge-ros2dds /usr/local/bin/

# Jetson(aarch64)—— 在有外網的機器下載後用 pscp 送過去
curl -sSL -o z.zip $B/zenoh-plugin-ros2dds-$V-aarch64-unknown-linux-gnu-standalone.zip
```

## Jetson 的 kernel socket buffer(關鍵,不做的話大點雲收不到)

`/cloud_registered` 每則 **0.42 MB**,遠超過單一 UDP datagram 上限。
Jetson 出廠的 `net.core.rmem_max` 只有 **212992(208KB)**,
分片還沒收齊就被丟掉。

症狀非常好認:`/Odometry`、`/tf`、`/cloud_registered_body` 全部正常,
**只有 `/cloud_registered` 一則都收不到**。

已寫入 `/etc/sysctl.d/60-ros2-lidar.conf`(可存活重開機):

```
net.core.rmem_max = 8388608
net.core.rmem_default = 8388608
net.core.wmem_max = 8388608
net.core.wmem_default = 1048576
net.ipv4.ipfrag_high_thresh = 134217728
net.ipv4.ipfrag_time = 3
```

改完**必須重啟 zenoh bridge**,行程只在啟動時讀一次 socket buffer 大小。

## 啟動順序

Jetson 端 bridge 要在 **FAST-LIO 之後**啟動,否則探索不到
`/cloud_registered` 的 publisher。控制台的「一鍵啟動」已經照這個順序做。

WSL 端由 `start_zenoh.sh` 負責(idempotent),
`start_rviz.sh` 會先呼叫它,控制台啟動時也會呼叫一次。

## 一個要知道的行為

新訂閱者建立後,**前 20 秒左右收不到資料**,路由建立需要時間。
所以 RViz 開起來不會馬上出現點雲,等一下就好 —— 這是正常的,
不要以為又壞了。

## 排錯

```bash
# 連線建立了嗎
ss -tn | grep 7447          # 應該看到 ESTAB ... 192.168.40.98:7447

# 路由建立了嗎
grep "Route Publisher"  /tmp/zenoh.log      # Jetson 端
grep "Route Subscriber" /tmp/zenoh_wsl.log  # WSL 端

# 資料真的有流嗎(給 40 秒,別只給 10 秒)
ros2 topic hz /cloud_registered
```
