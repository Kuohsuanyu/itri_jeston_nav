#!/usr/bin/env python3
"""箱子離線校準 —— 在導航箱還沒上車時,先把「箱子內部」那一段量完。

    base_link ──(上車才知道:x/y 捲尺 + 盒底高度 0.2510)── 盒底安裝面
                                                             │
                                                       ★ 這支程式量的
                                                             │
                                                    body(Mid-360 IMU)
                                                    camera_link(D435F)

為什麼可以先量:
    盒底安裝面到 body / camera_link 是**箱體內部的剛性關係**,鎖到車上不會變。
    而盒底之後貼的是上蓋內凹平面(z=+0.2510,量自 base_link.STL),那個面是水平的,
    所以「body 相對盒底的 roll/pitch」= URDF 裡的 LIDAR_ROLL / LIDAR_PITCH。

★ 為什麼用光達擬合的平面、而不是 IMU 重力當基準:
    要的是 body 相對**盒底安裝面**的姿態,不是相對重力。箱子坐在哪個平面上,
    光達擬合出來的就是那個平面 —— 桌子/地板歪不歪都不影響結果。
    IMU 只拿來當交叉檢查(告訴你地板本身斜幾度)。

★ 量測條件:
    - 箱子以「之後上車的姿態」(底面朝下)正放在**地板**上,不要放桌上
      (光達下俯 30°、離桌面才 20~30 cm,桌面會落在盲區和箱體自遮擋裡)
    - 周圍 1~4 m 開闊,地板看得到
    - 30 秒內不要碰箱子

用法:
    python3 calib_box.py              # 量 roll/pitch/高度 + 重算相機外參
    python3 calib_box.py --yaw        # 同上,另外列出四周垂直牆面,給 yaw 用
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
CLOUD_TOPIC = "/cloud_registered_body"
COLLECT_SEC = 30.0

PLATE_Z = 0.2510        # 上蓋內凹平面(= 盒底貼的那一面),相對 base_link,量自 STL
BOX_H = 0.160           # 理線盒高度,只拿來提示支架多高

PLANE_MIN_DROP = 0.15   # 安裝面至少在 body 下方這麼多
PLANE_MAX_R = 4.0       # 只用近處,遠處點稀疏
PLANE_BIN = 0.02
PLANE_TOL = 0.04        # 擬合內點門檻


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_rpy(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        return (math.atan2(R[2, 1], R[2, 2]),
                math.atan2(-R[2, 0], sy),
                math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0)


def angle_deg(a, b):
    c = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1))
    return math.degrees(math.acos(c))


def pc2_to_xyz(msg):
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    raw = raw.reshape(n, msg.point_step)
    xyz = raw[:, 0:12].copy().view(np.float32).reshape(n, 3).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def fit_plane(P):
    """最小二乘平面。回傳 (法線, 平面上一點)。"""
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    return Vt[2], c


def find_mount_plane(pts, up_hint):
    """找箱子坐著的那個平面。

    先用 IMU 的 up 粗略算高度做直方圖,最密的那一格當種子,再對種子附近的點
    做兩輪最小二乘 —— 第二輪用第一輪自己的法線重算內點,結果就不再依賴 IMU。
    """
    d = pts @ up_hint
    horiz = np.linalg.norm(pts - np.outer(d, up_hint), axis=1)
    cand = (d < -PLANE_MIN_DROP) & (horiz < PLANE_MAX_R)
    if cand.sum() < 500:
        return None, None, int(cand.sum())

    dc = d[cand]
    nb = max(int((dc.max() - dc.min()) / PLANE_BIN), 1)
    hist, edges = np.histogram(dc, bins=nb, range=(dc.min(), dc.max()))
    peak = edges[int(np.argmax(hist))] + PLANE_BIN * 0.5

    sel = cand & (np.abs(d - peak) < 0.06)
    if sel.sum() < 300:
        return None, None, int(sel.sum())

    nrm, c = fit_plane(pts[sel])
    if np.dot(nrm, up_hint) < 0:
        nrm = -nrm
    # 第二輪:用自己的法線重選內點,擺脫 IMU 的影響
    dd = (pts - c) @ nrm
    sel2 = cand & (np.abs(dd) < PLANE_TOL)
    if sel2.sum() >= 300:
        nrm, c = fit_plane(pts[sel2])
        if np.dot(nrm, up_hint) < 0:
            nrm = -nrm
        sel = sel2
    nrm = nrm / np.linalg.norm(nrm)

    # ── 這裡一定要擋 ──────────────────────────────────────────────
    # 直方圖峰值在「沒有地板」的場景會挑到一團雜訊,而且擬合出來的法線
    # **必然**接近垂直(水平薄片切過一堆垂直牆面,SVD 的最小奇異向量就是朝上的),
    # 所以「法線很水平」根本不能當證據 —— 那是循環論證。
    # 2026-08-10 車子停在窄小雜亂處時,就這樣吐出一個看似合理的 0.3019 m。
    #
    # 真正能分辨的是兩件事:
    #   1. 地板底下不會有東西。若有一堆點在擬合面**下方**,那就不是地板。
    #   2. 地板是大片的。內點只擠在一小塊,那是桌面/箱子頂/雜訊。
    below = int((((pts - c) @ nrm) < -0.05).sum())
    span = 0.0
    if sel.sum():
        Q = pts[sel] - c
        flat = Q - np.outer(Q @ nrm, nrm)          # 投影到平面上
        span = float(np.linalg.norm(flat, axis=1).max() * 2)
    bad = []
    if below > 0.03 * cand.sum():
        bad.append("平面下方還有 %d 個點(佔候選的 %.1f%%),地板底下不該有東西"
                   % (below, 100.0 * below / max(cand.sum(), 1)))
    if span < 1.0:
        bad.append("內點只鋪滿 %.2f m 寬,太小片,比較像桌面或雜訊" % span)
    if bad:
        return None, None, -1, bad
    return nrm, c, int(sel.sum()), []


def find_walls(pts, up, limit=6):
    """找四周的垂直平面。回傳 [(離 body 原點的距離, 法線, 點數), ...] 依距離排序。

    給 LIDAR_YAW 用:把箱子某一面貼平一面直牆,最近的那個垂直平面就是它。
    """
    d = pts @ up
    P = pts[(np.abs(d) < 1.2)]              # 只取跟 body 差不多高度的一圈
    if len(P) < 1000:
        return []
    # 投影到水平面,對「方位角」分群 —— 牆在極座標下是一條直線,先用 RANSAC 抓
    out = []
    rest = P
    rng = np.random.default_rng(0)
    for _ in range(limit):
        if len(rest) < 400:
            break
        best = None
        for _ in range(500):
            i = rng.choice(len(rest), 3, replace=False)
            a, b, c = rest[i]
            nv = np.cross(b - a, c - a)
            nn = np.linalg.norm(nv)
            if nn < 1e-6:
                continue
            nv /= nn
            if abs(np.dot(nv, up)) > 0.20:   # 不是垂直面
                continue
            cnt = int((np.abs((rest - a) @ nv) < 0.04).sum())
            if best is None or cnt > best[0]:
                best = (cnt, nv, a)
        if best is None or best[0] < 400:
            break
        cnt, nv, a = best
        inl = np.abs((rest - a) @ nv) < 0.04
        nrm, c = fit_plane(rest[inl])
        nrm -= np.dot(nrm, up) * up          # 強制成水平法線
        nrm /= np.linalg.norm(nrm)
        dist = float(abs(np.dot(c, nrm)))
        if np.dot(c, nrm) > 0:
            nrm = -nrm                        # 統一成「由牆指向 body」
        out.append((dist, nrm, int(inl.sum())))
        rest = rest[~inl]
    return sorted(out, key=lambda t: t[0])


def main():
    want_yaw = "--yaw" in sys.argv
    rclpy.init()
    n = Node("calib_box")
    buf = Buffer()
    TransformListener(buf, n)

    accs = []
    clouds = []
    n.create_subscription(Imu, IMU_TOPIC,
                          lambda m: accs.append([m.linear_acceleration.x,
                                                 m.linear_acceleration.y,
                                                 m.linear_acceleration.z]),
                          qos_profile_sensor_data)
    n.create_subscription(PointCloud2, CLOUD_TOPIC,
                          lambda m: clouds.append(m), qos_profile_sensor_data)

    print("量測中,%d 秒內不要碰箱子..." % COLLECT_SEC, flush=True)
    t0 = time.time()
    while time.time() - t0 < COLLECT_SEC:
        rclpy.spin_once(n, timeout_sec=0.1)
        el = int(time.time() - t0)
        if el and el % 10 == 0 and abs(time.time() - t0 - el) < 0.12:
            print("  %ds  IMU %d 筆  雲 %d 幀" % (el, len(accs), len(clouds)), flush=True)

    if len(accs) < 200:
        print("IMU 資料太少(%d 筆)" % len(accs))
        return 1
    if not clouds:
        print("收不到 %s,確認 FAST-LIO 有在跑" % CLOUD_TOPIC)
        return 1

    A = np.asarray(accs, dtype=np.float64)
    mag = np.linalg.norm(A, axis=1)
    jitter = float(mag.std() / max(mag.mean(), 1e-9))
    up_imu = A.mean(axis=0)
    up_imu /= np.linalg.norm(up_imu)

    print()
    print("=== A. IMU(只當交叉檢查,不是基準)===")
    print("  樣本 %d 筆  |a| %.4f  相對抖動 %.4f %s"
          % (len(A), mag.mean(), jitter, "" if jitter < 0.02 else "  ⚠ 量測期間箱子有動,重來"))
    print("  body 座標系裡的「上」 = (%+.4f, %+.4f, %+.4f)" % tuple(up_imu))

    pts = np.vstack([pc2_to_xyz(m) for m in clouds[-15:]])
    nrm, c0, npts = find_mount_plane(pts, up_imu)

    print()
    print("=== B. 安裝面(★ 這才是基準)===")
    if nrm is None:
        print("  找不到平面(候選點 %d)。箱子是不是還在桌上?周圍是不是被擋住?" % npts)
        return 1
    h = -float(np.dot(c0, nrm))
    diff = angle_deg(nrm, up_imu)
    print("  用了 %d 個點(共 %d)" % (npts, len(pts)))
    print("  安裝面法線 = (%+.4f, %+.4f, %+.4f)" % tuple(nrm))
    print("  body 原點離安裝面 = %.4f m" % h)
    print("  跟 IMU 重力差 %.2f 度  →  %s"
          % (diff, "地板是平的,兩個來源互相驗證通過" if diff < 2.0
             else "⚠ 地板本身斜了 %.1f 度(結果仍然有效,因為基準是這個平面)" % diff))
    if npts < 2000:
        print("  ⚠ 內點只有 %d,建議換到更開闊的地方重量" % npts)

    # M = 安裝面座標系:z = 平面法線,x = body 的 x 投影到平面(所以 yaw 先定為 0)
    z_M = nrm
    x_M = np.array([1.0, 0.0, 0.0]) - np.dot(np.array([1.0, 0.0, 0.0]), z_M) * z_M
    x_M /= np.linalg.norm(x_M)
    y_M = np.cross(z_M, x_M)
    R_body_M = np.column_stack([x_M, y_M, z_M])     # v_body = R_body_M @ v_M
    R_M_body = R_body_M.T                            # v_M    = R_M_body @ v_body
    roll, pitch, yaw = R_to_rpy(R_M_body)

    print()
    print("=== C. 相機:換算到同一個基準 ===")
    T_cam = None
    t_end = time.time() + 8
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform("body", "camera_link", rclpy.time.Time())
            T_cam = tf
            break
        except Exception:
            pass
    cam_line = None
    if T_cam is None:
        print("  查不到 body -> camera_link。先跑 ~/slam2d/camera_extrinsic.sh 再來。")
    else:
        q = T_cam.transform.rotation
        v = T_cam.transform.translation
        R_body_cam = quat_to_R(q.x, q.y, q.z, q.w)
        p_body_cam = np.array([v.x, v.y, v.z])
        # 換到安裝面座標系(= 水平的 base_link 姿態)
        p_M = R_M_body @ p_body_cam
        R_M_cam = R_M_body @ R_body_cam
        cr, cp, cy = R_to_rpy(R_M_cam)
        _, p_old, _ = R_to_rpy(R_body_cam)
        print("  相機 pitch:body 座標系 %+.2f 度  ->  安裝面座標系 %+.2f 度"
              % (math.degrees(p_old), math.degrees(cp)))
        if abs(math.degrees(cp)) < 3.0:
            print("  ✓ 接近 0,和「相機平視」一致 —— 安裝面的角度被獨立驗證了")
        else:
            print("  ⚠ 離 0 還有 %.1f 度。相機不是真的平視,或外參本身有誤差。"
                  % abs(math.degrees(cp)))
        cam_line = (p_M, cr, cp, cy)

    if want_yaw:
        print()
        print("=== D. 四周的垂直面(給 LIDAR_YAW 用)===")
        walls = find_walls(pts, nrm)
        if not walls:
            print("  找不到垂直面")
        else:
            print("  把箱子某一面貼平直牆,則**最近**的那個面就是它,")
            print("  它的 yaw 加/減 180 度就是箱子正面在 body 座標系裡的方位。")
            for dist, wn, cnt in walls:
                wl = R_M_body @ wn
                print("    距離 %5.2f m  點數 %6d  法線方位角 %+7.2f 度" %
                      (dist, cnt, math.degrees(math.atan2(wl[1], wl[0]))))

    print()
    print("=" * 66)
    print("貼進 chassis_description/urdf/qbot_sensors.xacro")
    print("=" * 66)
    print('  <xacro:property name="LIDAR_ROLL"  value="%.4f" />   <!-- %.2f 度 -->' % (roll, math.degrees(roll)))
    print('  <xacro:property name="LIDAR_PITCH" value="%.4f" />   <!-- %.2f 度 -->' % (pitch, math.degrees(pitch)))
    print('  <xacro:property name="LIDAR_YAW"   value="%.4f" />   <!-- 需要貼牆量,見 --yaw -->' % yaw)
    print('  <xacro:property name="LIDAR_Z"     value="%.4f" />   <!-- = PLATE_Z %.4f + 離盒底 %.4f -->'
          % (PLATE_Z + h, PLATE_Z, h))
    print('  <!-- LIDAR_X / LIDAR_Y:上車後捲尺量(相對四輪中心),對精度不敏感 -->')
    if cam_line is not None:
        p_M, cr, cp, cy = cam_line
        print()
        print('  <!-- 相機:位置要加上 LIDAR_X / LIDAR_Y,角度不用 -->')
        print('  <xacro:property name="CAM_X"     value="${LIDAR_X + %.4f}" />' % p_M[0])
        print('  <xacro:property name="CAM_Y"     value="${LIDAR_Y + %.4f}" />' % p_M[1])
        print('  <xacro:property name="CAM_Z"     value="%.4f" />' % (PLATE_Z + h + p_M[2]))
        print('  <xacro:property name="CAM_ROLL"  value="%.4f" />   <!-- %.2f 度 -->' % (cr, math.degrees(cr)))
        print('  <xacro:property name="CAM_PITCH" value="%.4f" />   <!-- %.2f 度 -->' % (cp, math.degrees(cp)))
        print('  <xacro:property name="CAM_YAW"   value="%.4f" />   <!-- %.2f 度 -->' % (cy, math.degrees(cy)))
    print("=" * 66)
    print()
    print("機構對照:光達離盒底 %.4f m,盒高 %.3f m  ->  支架高度約 %.4f m"
          % (h, BOX_H, h - BOX_H))
    print("光達傾角(相對安裝面)= %.2f 度" % angle_deg(nrm, np.array([0.0, 0.0, 1.0])))
    print()
    print("上車之後還要做的:")
    print("  1. 捲尺量 LIDAR_X / LIDAR_Y(箱子中心相對四輪中心)")
    print("  2. 箱子若沒裝正,補 LIDAR_YAW")
    print("  3. 跑 base_link_tf.sh 把 body->base_link 從 identity 換成量到的值")
    print("  4. 疊圖驗證(8094 埠)確認一切照舊")
    return 0


if __name__ == "__main__":
    sys.exit(main())
