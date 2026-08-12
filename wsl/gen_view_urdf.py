#!/usr/bin/env python3
"""從 robot_tf.sh 產生一份給 RViz 看的 xacro。

為什麼不直接用 chassis_description/urdf/qbot_sensors.xacro:
那一份的 body / camera_link 都掛在 base_link 底下,是「還沒改成 box_link 為
根」之前的舊結構。實機上跑的是 robot_tf.sh,樹長這樣:

    base_link ──(fused 模式不發)── box_link ─┬─ body
                                              └─ camera_link

模型如果沿用舊結構,拿去疊實機 TF 會對不上 —— 同一個 body 有兩個父節點,
正是我們花了三次才修掉的那個 bug,只是這次發生在 RViz 這一側。

所以這支直接從 robot_tf.sh 讀數值,保證模型和實機是同一組數字。
robot_tf.sh 改了就重跑這支,不會漂。

用法(在 Windows 上跑):
    python wsl/gen_view_urdf.py
輸出:
    wsl/qbot_view.xacro
"""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parent.parent
TF_SH = ROOT / "jetson_deploy" / "scripts" / "robot_tf.sh"
OUT = ROOT / "wsl" / "qbot_view.xacro"

# 理線盒外型。這三個值在 qbot_sensors.xacro 裡,robot_tf.sh 不需要它們
# (TF 只管座標系,不管長什麼樣),所以外型參數留在這裡。
BOX_L, BOX_W, BOX_H = 0.200, 0.200, 0.160


def read_consts(path):
    txt = path.read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r"^([A-Z_]+)=(-?[0-9.]+)\s*$", txt, re.M):
        out[m.group(1)] = float(m.group(2))
    return out


C = read_consts(TF_SH)
need = ["BOX_X", "BOX_Y", "BOX_Z", "BOX_YAW",
        "BODY_X", "BODY_Y", "BODY_Z", "BODY_ROLL", "BODY_PITCH", "BODY_YAW",
        "CAM_X", "CAM_Y", "CAM_Z", "CAM_ROLL", "CAM_PITCH", "CAM_YAW"]
missing = [k for k in need if k not in C]
if missing:
    raise SystemExit("robot_tf.sh 裡找不到:%s" % ", ".join(missing))

XACRO = """<?xml version="1.0" encoding="utf-8"?>
<!--
  ★ 自動產生,不要手改 —— 改 jetson_deploy/scripts/robot_tf.sh 之後重跑
    python wsl/gen_view_urdf.py

  這一份只給 RViz 看模型用,不會佈署到車上。
  數值全部抄自 robot_tf.sh,所以模型疊在實機 TF 上會完全重合。

  結構(跟實機一致):
    base_footprint -> base_link -> box_link -+-> wiring_box (外殼,純視覺)
                                             +-> body       (Mid-360 的 IMU 系)
                                             +-> camera_link(D435F)
-->
<robot name="qbot_view" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 原廠底盤:base_footprint / base_link / 四個輪子,含 STL -->
  <xacro:include filename="$(find chassis_description)/urdf/chassis_DD-M.xacro" />

  <!-- ═══ box_link:理線盒**底面**中心,整個導航樹的根 ═══
       實機上 fused 模式不發這一段(box_link 的父節點是 EKF 的 multi_odom)。
       這裡發是為了讓模型在 RViz 裡能跟底盤接起來看整台車。
       看實機資料時把 Fixed Frame 設成 box_link 或 map,這一段就不影響。 -->
  <link name="box_link" />
  <joint name="box_link_joint" type="fixed">
    <parent link="base_link" />
    <child  link="box_link" />
    <origin xyz="{BOX_X:.4f} {BOX_Y:.4f} {BOX_Z:.4f}" rpy="0 0 {BOX_YAW:.4f}" />
  </joint>

  <!-- 盒子外殼。URDF 的 box 以 link 原點為**中心**,所以要往上抬半個盒高 -->
  <link name="wiring_box">
    <visual>
      <geometry><box size="{BOX_L} {BOX_W} {BOX_H}" /></geometry>
      <material name="box_grey"><color rgba="0.25 0.27 0.30 1" /></material>
    </visual>
  </link>
  <joint name="wiring_box_joint" type="fixed">
    <parent link="box_link" />
    <child  link="wiring_box" />
    <origin xyz="0 0 {BOX_HALF}" rpy="0 0 0" />
  </joint>

  <!-- ═══ Mid-360 ═══
       body 是 FAST-LIO 蓋的章,指的是 Mid-360 的 **IMU** 座標系,
       不是光學中心也不是安裝面。pitch 0.5181 rad = 29.69 度,機構上就是斜的。 -->
  <link name="body">
    <visual>
      <geometry><cylinder radius="0.033" length="0.062" /></geometry>
      <material name="lidar_black"><color rgba="0.15 0.15 0.17 1" /></material>
    </visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="box_link" />
    <child  link="body" />
    <origin xyz="{BODY_X:.4f} {BODY_Y:.4f} {BODY_Z:.4f}"
            rpy="{BODY_ROLL:.4f} {BODY_PITCH:.4f} {BODY_YAW:.4f}" />
  </joint>

  <!-- ═══ D435F ═══
       camera_link 底下的 camera_*_optical_frame 由 realsense2_camera 自己發,
       這裡不要重複定義 —— 重複就是雙父節點。 -->
  <link name="camera_link">
    <visual>
      <geometry><box size="0.025 0.090 0.025" /></geometry>
      <material name="cam_blue"><color rgba="0.10 0.35 0.60 1" /></material>
    </visual>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="box_link" />
    <child  link="camera_link" />
    <origin xyz="{CAM_X:.4f} {CAM_Y:.4f} {CAM_Z:.4f}"
            rpy="{CAM_ROLL:.4f} {CAM_PITCH:.4f} {CAM_YAW:.4f}" />
  </joint>

</robot>
"""

OUT.write_text(
    XACRO.format(BOX_L=BOX_L, BOX_W=BOX_W, BOX_H=BOX_H, BOX_HALF=BOX_H / 2, **C),
    encoding="utf-8", newline="\n")
print("寫出 %s" % OUT)
print("  base_link -> box_link      (%.4f, %.4f, %.4f)  yaw %.4f"
      % (C["BOX_X"], C["BOX_Y"], C["BOX_Z"], C["BOX_YAW"]))
print("  box_link  -> body          (%.4f, %.4f, %.4f)  pitch %.4f (%.2f 度)"
      % (C["BODY_X"], C["BODY_Y"], C["BODY_Z"], C["BODY_PITCH"],
         C["BODY_PITCH"] * 57.29578))
print("  box_link  -> camera_link   (%.4f, %.4f, %.4f)"
      % (C["CAM_X"], C["CAM_Y"], C["CAM_Z"]))
