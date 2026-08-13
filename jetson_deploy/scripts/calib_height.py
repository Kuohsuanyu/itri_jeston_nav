#!/usr/bin/env python3
"""量「光達實際比模型高多少」—— 用地面當基準。

★★ 不要在狹小空間裡跑這支,結果會是垃圾 ★★

2026-08-12 在車庫裡跑,回報「內點 20%、地面法線離垂直 9.05 度、
建議 BOX_Z 改成 -0.3223」—— 全錯。原因是四周的牆離車不到 2 公尺,
往下的射線在 1.22 公尺就打到牆,根本到不了地面:

    往下(仰角 < -5 度)的點 41479 個
        打到的位置中位數:高度 +0.403 m、水平距離 1.22 m
        真正落在地面高度(|z| < 0.10)的只有 1.7%

沒有足夠的地面點,RANSAC 就去挑牆 —— 牆也是平面,而且點更多。

要用的話:**車開到空曠處**(四周至少 5 公尺沒有東西),再跑。
判斷結果可不可信看兩個數:內點比例要 > 35%,法線離垂直要 < 2 度。

另外輸出訊息裡的 0.2510 是寫死的舊值,不會讀現在的 BOX_Z ——
它算出來的「BOX_Z 要改成多少」是以那個舊值為基準的,不能直接套用。

現在 BOX_Z / BOX_X 是捲尺直接量的(見 robot_tf.sh 的註解):
量單一段、沒有誤差轉嫁,比擬合可靠。

── 以下是原本的原理說明 ──────────────────────────────────────────

原理:base_footprint 依定義**就在地面上**(z=0)。把光達的點雲透過現有的
TF 鏈換算到 base_footprint,再擬合地面平面,那個平面的 z 應該是 0。
不是 0 的話,差值就是整條鏈的高度誤差:

    base_footprint --(0.2032)-- base_link --(BOX_Z)-- box_link --(BODY_Z)-- body

三段裡:
    0.2032   原廠 URDF 的輪半徑,信得過
    BODY_Z   calib_box.py 把箱子單獨放地上量的,直接量到,信得過
    BOX_Z    量自 base_link.STL 的上蓋內凹平面,**假設盒子直接坐在那個面上**
             —— 中間要是墊了東西(理線槽、緩衝墊、額外支架),這裡就會少算

所以誤差歸給 BOX_Z。

擬合用 RANSAC:地面點佔多數但不是全部(牆、家具、車體自己都在點雲裡),
最小平方法會被那些點拉歪,RANSAC 不會。

用法:
    python3 calib_height.py            量 12 秒
    python3 calib_height.py 30         量 30 秒(點多一點更穩)
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

# PointField.datatype -> numpy 格式
_PF = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_xyz(msg):
    """照 PointField 的 offset 解出 xyz。

    不要用 sensor_msgs_py.point_cloud2.read_points_numpy —— 這個雲除了 xyz
    還有 intensity / offset_time 之類的欄位,那支在 Humble 上會把位元組
    對錯位置,解出來的座標是幾千公尺等級的垃圾(2026-08-11 實測:
    水平距離中位數 3867 m)。而且它不報錯,只是給你錯的數字。

    自己照 offset 建 dtype 就完全不會有這問題,順便快很多。
    """
    dt = np.dtype({
        "names": [f.name for f in msg.fields],
        "formats": [_PF[f.datatype] for f in msg.fields],
        "offsets": [f.offset for f in msg.fields],
        "itemsize": msg.point_step,
    })
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    P = np.stack([a["x"], a["y"], a["z"]], axis=-1).astype(np.float64)
    return P[np.isfinite(P).all(axis=1)]

CLOUD = "/cloud_registered_body"
GROUND = "base_footprint"
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0

# 只用車子附近的點做地面擬合。遠處的地面在光達的斜視角下入射角很淺,
# 距離誤差會放大成高度誤差;而且遠處容易掃到斜坡或門檻。
R_MIN, R_MAX = 1.0, 6.0
# 粗篩:地面應該落在 base_footprint 的 z=0 附近。放寬到 ±0.5 m 是為了
# 「就算現在偏了 30 公分也還抓得到」—— 太窄的話量不到就是量不到。
Z_BAND = 0.5


def q2R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ransac_plane(P, iters=400, tol=0.02, rng=None):
    """回傳 (法線, 平面上一點, 內點遮罩)。tol 是點到平面的距離門檻(公尺)。"""
    rng = rng or np.random.default_rng(0)
    best = (None, None, np.zeros(len(P), bool))
    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        a, b, c = P[idx]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        if abs(n[2]) < 0.85:        # 只要接近水平的平面,牆壁直接跳過
            continue
        d = np.abs((P - a) @ n)
        m = d < tol
        if m.sum() > best[2].sum():
            best = (n, a, m)
    return best


def main():
    rclpy.init()
    n = Node("calib_height")
    buf = Buffer()
    TransformListener(buf, n)
    pts = []

    def cb(msg):
        try:
            T = buf.lookup_transform(GROUND, msg.header.frame_id,
                                     rclpy.time.Time())
        except Exception:
            return
        q = T.transform.rotation
        v = T.transform.translation
        R = q2R(q.x, q.y, q.z, q.w)
        t = np.array([v.x, v.y, v.z])
        raw = cloud_xyz(msg)
        if len(raw) == 0:
            return
        P = raw @ R.T + t
        r = np.hypot(P[:, 0], P[:, 1])
        keep = (r > R_MIN) & (r < R_MAX) & (np.abs(P[:, 2]) < Z_BAND)
        if keep.any():
            pts.append(P[keep])

    n.create_subscription(
        PointCloud2, CLOUD, cb,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST))

    import time
    t0 = time.time()
    while time.time() - t0 < SECONDS:
        rclpy.spin_once(n, timeout_sec=0.1)

    if not pts:
        print("✗ 一個點都沒收到。檢查:")
        print("   ros2 topic hz %s" % CLOUD)
        print("   ros2 run tf2_ros tf2_echo %s body" % GROUND)
        return

    P = np.vstack(pts)
    print("候選點 %d 個(距離 %.1f~%.1f m,|z| < %.2f m)" % (len(P), R_MIN, R_MAX, Z_BAND))

    nrm, pt, mask = ransac_plane(P)
    if nrm is None or mask.sum() < 200:
        print("✗ 擬合不出地面(內點只有 %d)。車是不是停在很空曠或很雜亂的地方?"
              % mask.sum())
        return

    G = P[mask]
    if nrm[2] < 0:
        nrm = -nrm
    # 平面在原點正上/下方的高度 = 平面上任一點沿法線投影回 z 軸
    z0 = float(pt @ nrm / nrm[2])
    tilt = math.degrees(math.acos(min(1.0, abs(nrm[2]))))

    print("內點 %d / %d   (%.0f%%)" % (mask.sum(), len(P), 100 * mask.mean()))
    print("地面法線離垂直 %.2f 度" % tilt)
    print("地面在 %s 座標裡的高度 z = %+.4f  (應該是 0)" % (GROUND, z0))
    print()
    err = -z0        # z0 是負的代表地面比預期低 => 光達裝得比模型高
    print("═" * 56)
    print("  高度誤差 %+.4f m  (%.1f cm)" % (err, err * 100))
    if abs(err) < 0.015:
        print("  1.5 cm 以內,不用改。")
    else:
        print("  BOX_Z 要從 0.2510 改成 %.4f" % (0.2510 + err))
        print("  (改 robot_tf.sh 的 BOX_Z,然後重跑 start_slam2d.sh)")
    print("═" * 56)
    print()
    print("交叉檢查 —— 這幾個值改完之後應該是:")
    print("  base_footprint -> body  z = %.4f" % (0.2032 + 0.2510 + err + 0.2005))
    print("  /scan 高度帶 0.10 ~ 1.50 就會真的是離地 0.10 ~ 1.50")
    if tilt > 3:
        print()
        print("⚠ 地面法線歪了 %.1f 度 —— 車可能沒停在平地,或 BODY_ROLL/PITCH 有誤。" % tilt)
        print("  這種情況下高度值不可信,換個平坦的地方重量。")


if __name__ == "__main__":
    main()
