#!/usr/bin/env python3
"""把 qbot_sensors.xacro 展開成純 URDF,給 Windows 這邊做視覺預覽用。

Jetson 上不需要這支 —— 那邊有真正的 xacro 工具。這裡只是為了讓你在改完
感測器數值之後,馬上在電腦上看到光達和相機掛在車上的樣子,不用等傳到
Jetson 再開 RViz。

mesh 路徑改成相對的 meshes/DD-M/*.STL,所以輸出的 .urdf 要放在
chassis_description/ 底下才找得到檔案。

用法:  python chassis/gen_preview_urdf.py
"""
import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent
DESC = HERE / "chassis-ros2-driver" / "chassis_description"
SRC = DESC / "urdf" / "qbot_sensors.xacro"
CHASSIS = DESC / "urdf" / "chassis_DD-M.xacro"
OUT = DESC / "qbot_preview.urdf"

PROP = re.compile(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"')


def props(path):
    return {m.group(1): m.group(2) for m in PROP.finditer(path.read_text(encoding="utf-8"))}


def wheel(name, x, y, z, axis_y, mesh_dir):
    return f"""
  <link name="wheel_{name}_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/{mesh_dir}/wheel_{name}_link.STL"/></geometry>
      <material name="wheel_grey"><color rgba="0.50196 0.50196 0.50196 1"/></material>
    </visual>
  </link>
  <joint name="wheel_{name}_joint" type="continuous">
    <origin xyz="{x} {y} {z}" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_{name}_link"/>
    <axis xyz="0 {axis_y} 0"/>
  </joint>"""


def main():
    c = props(CHASSIS)
    s = props(SRC)
    md = c["mesh_dir"]
    fx, rx = c["front_x"], c["rear_x"]
    ty, wz = c["track_y"], c["wheel_z"]
    wr = float(c["wheel_radius"])
    bf_z = wr - float(wz)

    body = "".join([
        wheel("front_left", fx, ty, wz, "1", md),
        wheel("front_right", fx, f"-{ty}", wz, "-1", md),
        wheel("rear_left", rx, ty, wz, "1", md),
        wheel("rear_right", rx, f"-{ty}", wz, "-1", md),
    ])

    urdf = f"""<?xml version="1.0" encoding="utf-8"?>
<!-- 自動產生,不要手改。改 urdf/qbot_sensors.xacro 之後重跑 gen_preview_urdf.py -->
<robot name="qbot">

  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 {bf_z}" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/{md}/base_link.STL"/></geometry>
      <material name="white"><color rgba="0.85 0.86 0.88 1"/></material>
    </visual>
  </link>
{body}

  <link name="wiring_box">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="{s['BOX_L']} {s['BOX_W']} {s['BOX_H']}"/></geometry>
      <material name="box_grey"><color rgba="0.25 0.27 0.30 1"/></material>
    </visual>
  </link>
  <joint name="wiring_box_joint" type="fixed">
    <parent link="base_link"/>
    <child link="wiring_box"/>
    <origin xyz="0 0 {float(s['PLATE_Z']) + float(s['BOX_H'])/2:.4f}" rpy="0 0 0"/>
  </joint>

  <link name="body">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="0.033" length="0.062"/></geometry>
      <material name="lidar_black"><color rgba="0.15 0.15 0.17 1"/></material>
    </visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="body"/>
    <origin xyz="{s['LIDAR_X']} {s['LIDAR_Y']} {s['LIDAR_Z']}"
            rpy="{s['LIDAR_ROLL']} {s['LIDAR_PITCH']} {s['LIDAR_YAW']}"/>
  </joint>

  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.025 0.090 0.025"/></geometry>
      <material name="cam_blue"><color rgba="0.10 0.35 0.60 1"/></material>
    </visual>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="{s['CAM_X']} {s['CAM_Y']} {s['CAM_Z']}"
            rpy="{s['CAM_ROLL']} {s['CAM_PITCH']} {s['CAM_YAW']}"/>
  </joint>

</robot>
"""
    OUT.write_text(urdf, encoding="utf-8")
    print("wrote", OUT)
    print("  base_footprint -> base_link  z = %.4f  (輪半徑)" % bf_z)
    print("  光達 body    xyz=(%s, %s, %s)  rpy=(%s, %s, %s)"
          % (s["LIDAR_X"], s["LIDAR_Y"], s["LIDAR_Z"],
             s["LIDAR_ROLL"], s["LIDAR_PITCH"], s["LIDAR_YAW"]))
    print("  相機 camera  xyz=(%s, %s, %s)  rpy=(%s, %s, %s)"
          % (s["CAM_X"], s["CAM_Y"], s["CAM_Z"],
             s["CAM_ROLL"], s["CAM_PITCH"], s["CAM_YAW"]))
    box_top = float(s["PLATE_Z"]) + float(s["BOX_H"])
    print()
    print("  上蓋平面   離地 %.4f m" % (bf_z + float(s["PLATE_Z"])))
    print("  理線盒頂   離地 %.4f m   (%s x %s x %s m)"
          % (bf_z + box_top, s["BOX_L"], s["BOX_W"], s["BOX_H"]))
    print("  光達       離地 %.4f m   -> 支架高 %.4f m(盒頂到光達)"
          % (bf_z + float(s["LIDAR_Z"]), float(s["LIDAR_Z"]) - box_top))
    print("  相機       離地 %.4f m   -> 支架高 %.4f m"
          % (bf_z + float(s["CAM_Z"]), float(s["CAM_Z"]) - box_top))


if __name__ == "__main__":
    main()
