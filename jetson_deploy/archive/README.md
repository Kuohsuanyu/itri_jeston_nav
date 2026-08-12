# archive —— 已退役的腳本

這裡的東西**都不再使用**,留著只為了查「當初是怎麼做的」。
散在 Jetson 家目錄好幾個月,2026-08-12 整併 repo 時收進來。

## 為什麼退役

| 群組 | 檔案 | 原因 |
|---|---|---|
| **nvblox** | `nvblox_*.sh`、`start_nvblox*.sh`、`start_mesh_web.sh` | 2026-08-04 決定不用 nvblox —— 有光達,深度相機幫不上多少忙,改成直接把光達點雲上色 |
| **Isaac / Docker** | `isaac_*.sh`、`enter_isaac.sh`、`container_*.sh`、`install_rs_container.sh`、`docker_snapshotter.sh`、`nitros_*.sh`、`NV_SET_TARGET_*.sh` | 都是為了跑 nvblox 才需要容器,一起退役 |
| **一次性診斷** | `check[234].sh`、`diag_*.sh`、`subcheck.sh`、`qoscheck.sh`、`shm_test.sh`、`pulldiag.sh`、`final_check.sh`、`blobtest.sh` | 當時查特定問題臨時寫的,問題解決就沒用了 |
| **硬體測試** | `depth_*.sh`、`camalive.sh`、`usb_reset*.sh`、`speedtest.sh` | 相機和 USB 穩定之後不需要 |
| **舊版 TF** | `base_link_tf.sh`、`calc_tf.sh`、`fix_tf_tree.sh` | 被 `scripts/robot_tf.sh` 取代。舊的把感測器發成 `base_link` 的**父**節點,造成雙父節點 —— 兩次相同查詢隔五分鐘得到 x=+0.831 和 x=+0.202 |

## 現役的在哪

```
scripts/     跑在 Jetson 上的主要腳本 → ~/slam2d
nav2/        Nav2 設定 → ~/nav2
chassis_test/  底盤遙控網頁 → ~/chassis_test
web/         點雲和相機的網頁檢視
```

## 要刪嗎

可以,git 歷史裡留著。留在工作目錄的唯一理由是 `grep` 得到 —— 例如想知道「當初 nvblox 是怎麼裝的」直接搜比翻 git log 快。
