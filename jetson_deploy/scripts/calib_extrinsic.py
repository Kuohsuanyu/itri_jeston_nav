#!/usr/bin/env python3
"""用 ICP 校準 base_link -> camera_link 外參。

原理:光達和深度相機看的是同一個場景,兩者都給 3D 點雲。
解出讓兩片點雲重合的剛體變換,那就是外參 —— 不需要標定板。

流程:
  1. 收一幀光達點雲(/cloud_registered_body,在 body frame)
     ★ 用 TF 轉到 base_link。以前這裡直接當成 base_link 用,是因為當時
       body -> base_link 是 identity;現在 base_link 被扶正了(見
       calib_base_link.py),那個假設不再成立,不轉換的話會多出一個
       30 度的系統誤差而且完全看不出來。
  2. 收一幀深度影像 + camera_info,用內參反投影成 3D 點
     (在 camera_depth_optical_frame)
  3. 以目前的 TF 當初始猜測,把相機點轉到 base_link
  4. ICP 迭代對齊
  5. 把結果從 base_link->depth_optical 換算回 base_link->camera_link

★ 場地要求(很重要,擺錯會算出錯誤答案而且看不出來):
  - 要有**多個方向的結構**:牆角 + 地面 + 幾個箱子/椅子最理想
  - **不能對著單一平牆** —— 那是退化情況,沿牆面滑動都符合,解不唯一
  - 物體距離 1~3 公尺(D435 準確的範圍)
  - 校準期間整組不要動
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

try:
    from scipy.spatial import cKDTree
except ImportError:
    print("需要 scipy:  pip3 install --user scipy")
    sys.exit(1)

LIDAR_TOPIC = "/cloud_registered_body"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
INFO_TOPIC = "/camera/camera/depth/camera_info"
# 相機曾經跑在 Isaac 容器裡,namespace 是 /camera0/camera。兩個都試。
ALT_NS = "/camera0/camera"

MAX_RANGE = 3.5      # 只用 D435 準確的範圍
MIN_RANGE = 0.4
ICP_ITERS = 60
REJECT_DIST = 0.25   # 對應點距離超過這個就當外點丟掉


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


def tf_to_mat(t):
    T = np.eye(4)
    q = t.transform.rotation
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    v = t.transform.translation
    T[:3, 3] = [v.x, v.y, v.z]
    return T


def pc2_to_xyz(msg):
    """PointCloud2 -> Nx3。假設前 12 bytes 是 float32 xyz(FAST-LIO 的格式)。"""
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    raw = raw.reshape(n, msg.point_step)
    xyz = raw[:, 0:12].copy().view(np.float32).reshape(n, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]


def depth_to_xyz(img, info):
    """深度影像 + 內參 -> Nx3(在 camera_depth_optical_frame)。"""
    d = np.frombuffer(img.data, dtype=np.uint16).reshape(img.height, img.width)
    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    vs, us = np.nonzero(d)
    z = d[vs, us].astype(np.float32) / 1000.0
    keep = (z > MIN_RANGE) & (z < MAX_RANGE)
    us, vs, z = us[keep], vs[keep], z[keep]
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def icp(src, dst, T0, iters=ICP_ITERS):
    """point-to-point ICP。src 會被 T 變換去貼合 dst。"""
    tree = cKDTree(dst)
    T = T0.copy()
    prev = None
    m = np.zeros(len(src), dtype=bool)
    for i in range(iters):
        P = (T[:3, :3] @ src.T).T + T[:3, 3]
        dist, idx = tree.query(P, k=1)
        m = dist < REJECT_DIST
        if m.sum() < 100:
            print("  對應點太少(%d),場景結構可能不足" % m.sum())
            break
        A, B = P[m], dst[idx[m]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[2] *= -1
            R = Vt.T @ U.T
        dT = np.eye(4)
        dT[:3, :3] = R
        dT[:3, 3] = cb - R @ ca
        T = dT @ T
        rms = math.sqrt((dist[m] ** 2).mean())
        if prev is not None and abs(prev - rms) < 1e-6:
            break
        prev = rms
    return T, prev, int(m.sum())


def main():
    rclpy.init()
    n = Node("calib_extrinsic")
    buf = Buffer()
    TransformListener(buf, n)
    got = {}
    n.create_subscription(PointCloud2, LIDAR_TOPIC,
                          lambda m: got.setdefault("lidar", m), qos_profile_sensor_data)
    for base in (DEPTH_TOPIC, ALT_NS + "/depth/image_rect_raw"):
        n.create_subscription(Image, base,
                              lambda m: got.setdefault("depth", m), qos_profile_sensor_data)
    for base in (INFO_TOPIC, ALT_NS + "/depth/camera_info"):
        n.create_subscription(CameraInfo, base,
                              lambda m: got.setdefault("info", m), qos_profile_sensor_data)

    print("收集資料中(最多 40 秒)...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 40 and len(got) < 3:
        rclpy.spin_once(n, timeout_sec=0.1)

    missing = [k for k in ("lidar", "depth", "info") if k not in got]
    if missing:
        print("  缺少:", missing)
        print("  確認光達與相機都在跑")
        return 1

    # 相機內部的固定變換(驅動發布的)
    t_end = time.time() + 15
    T_cl_do = None
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            T_cl_do = tf_to_mat(buf.lookup_transform(
                "camera_link", "camera_depth_optical_frame", rclpy.time.Time()))
            break
        except Exception:
            pass
    if T_cl_do is None:
        print("  查不到 camera_link -> camera_depth_optical_frame")
        return 1

    try:
        T_bl_cl0 = tf_to_mat(buf.lookup_transform(
            "base_link", "camera_link", rclpy.time.Time()))
    except Exception as e:
        print("  查不到目前的 base_link -> camera_link:", e)
        return 1

    # 點雲在 body frame。base_link 被 calib_base_link.py 扶正之後就不再等於
    # body,一定要走 TF 轉過去。
    cloud_frame = got["lidar"].header.frame_id or "body"
    try:
        T_bl_body = tf_to_mat(buf.lookup_transform(
            "base_link", cloud_frame, rclpy.time.Time()))
    except Exception as e:
        print("  查不到 base_link -> %s:%s" % (cloud_frame, e))
        return 1
    tilt = math.degrees(math.acos(min(1.0, max(-1.0, T_bl_body[2, 2]))))
    print("  點雲座標系 = %s,相對 base_link 傾斜 %.2f 度" % (cloud_frame, tilt))

    lid = pc2_to_xyz(got["lidar"])
    r = np.linalg.norm(lid, axis=1)          # 距離要在感測器座標系算
    lid = lid[(r > MIN_RANGE) & (r < MAX_RANGE)]
    lid = (T_bl_body[:3, :3] @ lid.T).T + T_bl_body[:3, 3]

    cam = depth_to_xyz(got["depth"], got["info"])
    # 相機點很密,抽樣加速
    if len(cam) > 40000:
        cam = cam[np.random.choice(len(cam), 40000, replace=False)]

    print()
    print("  光達點 %d,相機點 %d(都已限制在 %.1f~%.1f m)"
          % (len(lid), len(cam), MIN_RANGE, MAX_RANGE))
    if len(lid) < 500 or len(cam) < 500:
        print("  點太少,把相機對準有結構的場景(牆角 + 物體)再試")
        return 1

    T0 = T_bl_cl0 @ T_cl_do          # 初始猜測:base_link -> depth_optical
    P0 = (T0[:3, :3] @ cam.T).T + T0[:3, 3]
    d0, _ = cKDTree(lid).query(P0, k=1)
    rms0 = math.sqrt((d0[d0 < REJECT_DIST] ** 2).mean())

    T, rms, npair = icp(cam, lid, T0)
    T_bl_cl = T @ np.linalg.inv(T_cl_do)   # 換回 base_link -> camera_link

    x, y, z = T_bl_cl[:3, 3]
    roll, pitch, yaw = R_to_rpy(T_bl_cl[:3, :3])
    ox, oy, oz = T_bl_cl0[:3, 3]
    oroll, opitch, oyaw = R_to_rpy(T_bl_cl0[:3, :3])

    print()
    print("  === 對齊誤差 ===")
    print("    校準前 RMS %.4f m" % rms0)
    print("    校準後 RMS %.4f m   (%d 組對應點)" % (rms or -1, npair))
    print()
    print("  === base_link -> camera_link ===")
    print("             目前值      校準值      差異")
    print("    X      %8.4f   %8.4f   %+7.4f m" % (ox, x, x - ox))
    print("    Y      %8.4f   %8.4f   %+7.4f m" % (oy, y, y - oy))
    print("    Z      %8.4f   %8.4f   %+7.4f m" % (oz, z, z - oz))
    print("    roll   %8.4f   %8.4f   %+7.2f deg" % (oroll, roll, math.degrees(roll - oroll)))
    print("    pitch  %8.4f   %8.4f   %+7.2f deg" % (opitch, pitch, math.degrees(pitch - opitch)))
    print("    yaw    %8.4f   %8.4f   %+7.2f deg" % (oyaw, yaw, math.degrees(yaw - oyaw)))
    print()
    print("  === 把這幾行填進 ~/slam2d/camera_extrinsic.sh ===")
    print("    X=%.4f" % x)
    print("    Y=%.4f" % y)
    print("    Z=%.4f" % z)
    print("    ROLL=%.4f" % roll)
    print("    PITCH=%.4f" % pitch)
    print("    YAW=%.4f" % yaw)
    print()
    if tilt < 3.0:
        print("  注意:base_link 相對點雲座標系只差 %.1f 度 —— 如果你已經跑過" % tilt)
        print("  calib_base_link.py,這個值應該接近 30 度。看起來 base_link 還沒扶正。")
    if rms and rms > 0.08:
        print("  ⚠ 殘差偏大(>8cm)。可能原因:場景結構不足(對著平牆)、")
        print("    初始猜測差太遠、或校準期間有東西在動。換個有牆角和物體的位置再試。")
    elif rms and rms < 0.04:
        print("  ✓ 殘差 <4cm,結果可信")
    print()
    print("  下一步:bash ~/calib_view/start_overlay.sh  然後開 :8094 用眼睛確認")
    return 0


if __name__ == "__main__":
    sys.exit(main())
