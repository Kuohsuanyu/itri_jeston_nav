#!/usr/bin/env python3
"""量出 base_link —— 把傾斜的光達座標系校正成水平的車體座標系。

問題:
  FAST-LIO 的 body frame 就是 Mid-360 的 IMU 座標系,而光達在機構上斜了約 30 度。
  目前 start_slam2d.sh 用兩個 identity static TF 把座標鏈接起來:

      odom --(identity)-- camera_init --(FAST-LIO)-- body --(identity)-- base_link

  兩個 identity 各造成一個錯誤:

  1. base_link 跟著光達斜 -> pointcloud_to_laserscan 的「水平切片」是斜的
  2. odom 也跟著斜 -> slam_toolbox 把 odom->base_link 投影到 odom 的 x-y 平面
     做二維定位,平面斜 30 度的話,地上走 1 m 投影下來只剩 0.87 m。
     **13% 的尺度誤差**,地圖繞一圈回來會對不上。

這支程式用三個獨立來源把傾斜量出來,不靠 CAD 猜:

  A. IMU 靜止時的加速度 = 重力方向 -> body 的 roll/pitch     (約 0.2 度)
  B. 點雲的地面平面擬合 -> 地面法線 + 光達離地高度            (約 0.5 度)
  C. 目前 TF 裡相機的 pitch(相機是平視的)-> 交叉檢查        (約 1 度)

A 和 B 若差超過 2 度就不要用結果 —— 通常代表車沒停在水平地面上,
或量測期間有人在動。

★ 量測條件:
  - 車停在**平的地面**上,不要在斜坡、不要壓到門檻
  - 量測期間整台不要動(30 秒)
  - 周圍地面要看得到,不要四周都被箱子擋住
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, TransformListener

IMU_TOPIC = "/livox/imu"
CLOUD_TOPIC = "/cloud_registered_body"   # body frame,不是世界座標
COLLECT_SEC = 30.0

GROUND_MIN_DROP = 0.20   # 離感測器至少這麼低才可能是地面
GROUND_MAX_RADIUS = 4.0  # 只用近處的地面,遠處點稀疏又容易掃到別的樓層
GROUND_BIN = 0.02        # 高度直方圖的格寬,找地面峰值用
AGREE_WARN_DEG = 2.0     # IMU 和地面法線差超過這麼多就警告


def R_to_rpy(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        return (math.atan2(R[2, 1], R[2, 2]),
                math.atan2(-R[2, 0], sy),
                math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def level_basis(up, fwd_hint=np.array([1.0, 0.0, 0.0])):
    """給一個「上」方向,建出一組水平座標系的基底(以輸入座標系表示)。

    z 軸 = up;x 軸 = 原座標系的 x 投影到水平面後正規化(所以航向不會亂轉);
    y 軸 = z x x。回傳的矩陣 R 滿足 v_原 = R @ v_水平。
    """
    z = up / np.linalg.norm(up)
    x = fwd_hint - np.dot(fwd_hint, z) * z
    nx = np.linalg.norm(x)
    if nx < 1e-6:
        # 原座標系的 x 幾乎就是垂直方向(光達側躺),改用 y 當提示
        x = np.array([0.0, 1.0, 0.0])
        x = x - np.dot(x, z) * z
        nx = np.linalg.norm(x)
    x /= nx
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def pc2_to_xyz(msg):
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    raw = raw.reshape(n, msg.point_step)
    xyz = raw[:, 0:12].copy().view(np.float32).reshape(n, 3).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def fit_ground(pts, up):
    """在點雲裡找地面,回傳 (法線, 離地高度, 用到的點數)。

    做法:先用 IMU 的 up 把每個點的「高度」算出來(d = up . p),
    低於感測器一定距離、且水平距離夠近的點做直方圖,最密的那一格就是地面。
    再對那一格附近的點做最小二乘平面擬合,得到獨立於 IMU 的法線。
    """
    d = pts @ up
    horiz = np.linalg.norm(pts - np.outer(d, up), axis=1)
    cand = (d < -GROUND_MIN_DROP) & (horiz < GROUND_MAX_RADIUS)
    if cand.sum() < 500:
        return None, None, int(cand.sum())

    dc = d[cand]
    lo, hi = dc.min(), dc.max()
    nb = max(int((hi - lo) / GROUND_BIN), 1)
    hist, edges = np.histogram(dc, bins=nb, range=(lo, hi))
    peak = edges[int(np.argmax(hist))] + GROUND_BIN * 0.5

    # 取峰值上下 6cm 的點做平面擬合
    sel = cand & (np.abs(d - peak) < 0.06)
    P = pts[sel]
    if len(P) < 300:
        return None, None, int(sel.sum())

    c = P.mean(axis=0)
    # 最小的奇異向量就是平面法線
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    nrm = Vt[2]
    if np.dot(nrm, up) < 0:
        nrm = -nrm                      # 統一朝上
    height = -float(np.dot(c, nrm))     # 感測器到平面的垂直距離
    return nrm / np.linalg.norm(nrm), height, int(len(P))


def angle_deg(a, b):
    c = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1))
    return math.degrees(math.acos(c))


def main():
    rclpy.init()
    n = Node("calib_base_link")
    buf = Buffer()
    TransformListener(buf, n)

    accs = []
    cloud = {}
    n.create_subscription(Imu, IMU_TOPIC,
                          lambda m: accs.append([m.linear_acceleration.x,
                                                 m.linear_acceleration.y,
                                                 m.linear_acceleration.z]),
                          qos_profile_sensor_data)
    n.create_subscription(PointCloud2, CLOUD_TOPIC,
                          lambda m: cloud.__setitem__("m", m), qos_profile_sensor_data)

    print("量測中,請不要移動車體(%d 秒)..." % COLLECT_SEC, flush=True)
    t0 = time.time()
    while time.time() - t0 < COLLECT_SEC:
        rclpy.spin_once(n, timeout_sec=0.1)
        el = int(time.time() - t0)
        if el and el % 10 == 0 and abs(time.time() - t0 - el) < 0.12:
            print("  %ds  IMU %d 筆" % (el, len(accs)), flush=True)

    if len(accs) < 200:
        print("IMU 資料太少(%d 筆),確認 /livox/imu 有在發" % len(accs))
        return 1
    if "m" not in cloud:
        print("收不到 %s,確認 FAST-LIO 有在跑" % CLOUD_TOPIC)
        return 1

    A = np.asarray(accs, dtype=np.float64)
    mag = np.linalg.norm(A, axis=1)
    # Livox 的加速度單位是 g,不是 m/s^2。這裡不用轉換(只要方向),
    # 但把它印出來,因為單位搞錯是這顆 IMU 最常見的坑。
    unit = "g" if abs(mag.mean() - 1.0) < 0.2 else "m/s^2"
    jitter = float(mag.std() / max(mag.mean(), 1e-9))

    # 靜止時加速規量到的是「比力」,方向指向上
    up_imu = A.mean(axis=0)
    up_imu /= np.linalg.norm(up_imu)

    print()
    print("=== A. IMU 重力方向 ===")
    print("  樣本 %d 筆,|a| 平均 %.4f %s,相對抖動 %.4f" % (len(A), mag.mean(), unit, jitter))
    print("  body 座標系裡的「上」 = (%+.4f, %+.4f, %+.4f)" % tuple(up_imu))
    if jitter > 0.02:
        print("  ⚠ 抖動偏大,量測期間車可能有動。結果不可信,重量一次。")

    pts = pc2_to_xyz(cloud["m"])
    nrm, height, npts = fit_ground(pts, up_imu)

    print()
    print("=== B. 地面平面擬合 ===")
    if nrm is None:
        print("  找不到地面(候選點 %d)。周圍地面被擋住,或車架太高。" % npts)
        print("  改用 IMU 的結果,離地高度請自己量並填進 base_link_tf.sh。")
        up = up_imu
        height = None
    else:
        diff = angle_deg(nrm, up_imu)
        print("  用了 %d 個地面點" % npts)
        print("  地面法線 = (%+.4f, %+.4f, %+.4f)" % tuple(nrm))
        print("  跟 IMU 的重力方向差 %.2f 度" % diff)
        print("  光達離地高度 = %.4f m" % height)
        if diff > AGREE_WARN_DEG:
            print("  ⚠ 兩者差超過 %.1f 度。車可能沒停在水平地面,或擬合到的不是地面" % AGREE_WARN_DEG)
            print("    (例如桌面、坡道)。用 IMU 的結果,但高度值要存疑。")
            up = up_imu
        else:
            # 兩者一致,取平均降低雜訊
            up = up_imu + nrm
            up /= np.linalg.norm(up)

    # R 滿足 v_body = R @ v_base_link
    R = level_basis(up)
    roll, pitch, yaw = R_to_rpy(R)

    print()
    print("=== C. 交叉檢查:相機在新座標系裡應該接近平視 ===")
    t_end = time.time() + 8
    T_bl_cl = None
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform("base_link", "camera_link", rclpy.time.Time())
            q = tf.transform.rotation
            T_bl_cl = quat_to_R(q.x, q.y, q.z, q.w)
            break
        except Exception:
            pass
    if T_bl_cl is None:
        print("  查不到 base_link -> camera_link,跳過這項檢查")
    else:
        # 目前的 base_link 還是 identity-of-body,所以這個矩陣其實是 body->camera。
        # 換到新的水平座標系:R_new_cam = R^T @ R_body_cam
        _, p_old, _ = R_to_rpy(T_bl_cl)
        _, p_new, _ = R_to_rpy(R.T @ T_bl_cl)
        print("  相機 pitch:目前座標系 %+.2f 度  ->  新座標系 %+.2f 度"
              % (math.degrees(p_old), math.degrees(p_new)))
        if abs(math.degrees(p_new)) < 3.0:
            print("  ✓ 接近 0,和「相機平視」一致 —— 三個來源互相驗證通過")
        else:
            print("  ⚠ 離 0 還有 %.1f 度。可能相機並非真的平視,或外參本身有誤差。"
                  % abs(math.degrees(p_new)))

    # --- odom 那一段:FAST-LIO 的世界座標系有沒有對齊重力? ---
    print()
    print("=== D. odom 是不是水平的 ===")
    R_oc = np.eye(3)
    up_ci = None
    t_end = time.time() + 8
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform("camera_init", "body", rclpy.time.Time())
            q = tf.transform.rotation
            up_ci = quat_to_R(q.x, q.y, q.z, q.w) @ up
            break
        except Exception:
            pass
    if up_ci is None:
        print("  查不到 camera_init -> body,跳過。odom 的修正值請重跑一次取得。")
        tilt = None
    else:
        tilt = angle_deg(up_ci, np.array([0.0, 0.0, 1.0]))
        print("  重力方向在 camera_init 裡 = (%+.4f, %+.4f, %+.4f)" % tuple(up_ci))
        print("  跟 camera_init 的 z 軸差 %.2f 度" % tilt)
        if tilt < 3.0:
            print("  ✓ FAST-LIO 的世界座標系已經對齊重力,odom 不用改")
        else:
            print("  ✗ 世界座標系斜了 %.2f 度。二維地圖的尺度誤差約 %.1f%%"
                  % (tilt, (1 - math.cos(math.radians(tilt))) * 100))
            print("    (slam_toolbox 把軌跡投影到斜掉的 x-y 平面上)")
            R_oc = level_basis(up_ci).T     # v_odom = R_oc @ v_camera_init

    or_, op_, oy_ = R_to_rpy(R_oc)
    h = height if height is not None else 0.0

    print()
    print("=" * 62)
    print("把下面這段貼進 ~/slam2d/base_link_tf.sh 的參數區")
    print("=" * 62)
    print("# body -> base_link:把傾斜的光達座標系轉成水平車體座標系")
    print("BL_X=%.4f" % (-h * up[0]))
    print("BL_Y=%.4f" % (-h * up[1]))
    print("BL_Z=%.4f" % (-h * up[2]))
    print("BL_ROLL=%.4f" % roll)
    print("BL_PITCH=%.4f" % pitch)
    print("BL_YAW=%.4f" % yaw)
    print("# odom -> camera_init:把 FAST-LIO 的世界座標系扶正")
    print("OD_X=0.0000")
    print("OD_Y=0.0000")
    print("OD_Z=%.4f" % h)
    print("OD_ROLL=%.4f" % or_)
    print("OD_PITCH=%.4f" % op_)
    print("OD_YAW=%.4f" % oy_)
    print("=" * 62)
    print()
    print("光達傾角 = %.2f 度   離地高度 = %s"
          % (angle_deg(up, np.array([0.0, 0.0, 1.0])),
             "%.3f m" % height if height is not None else "未測得(自己量)"))
    print()
    print("套用之後必須做的兩件事:")
    print("  1. 重跑 calib_extrinsic.py —— base_link 動了,相機外參的基準也跟著變")
    print("  2. 改 start_slam2d.sh 的 min_height / max_height:")
    print("     現在是相對傾斜光達的 -0.35 / 0.60,改成相對地面的 0.10 / 1.50")
    return 0


if __name__ == "__main__":
    sys.exit(main())
