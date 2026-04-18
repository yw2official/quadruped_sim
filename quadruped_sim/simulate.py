#!/usr/bin/env python3
"""
四轮足机器人 MuJoCo 仿真
两个 rh1 双轮足机器人前后拼接，轮子驱动前进。

键盘控制（在 MuJoCo viewer 窗口中按键）：
  W / S  — 前进 / 后退
  A / D  — 左转 / 右转
  Q / E  — 腿高 ↑ / ↓
  Space  — 停止（速度清零）

运行方式：
  cd /home/yanwang/quadruped_sim/quadruped_sim
  pixi run python simulate.py
"""

import time
import xml.etree.ElementTree as ET
import mujoco
import mujoco.viewer

URDF_PATH     = "rh1.urdf"
REAR_OFFSET_X = -0.17   # 后机器人相对前机器人的 X 轴偏移（m）
INIT_HEIGHT   = 0.5     # 启动时底盘高度（m），自然落地后由物理引擎决定
TIMESTEP      = 0.002   # 仿真步长（s）

# 车轮 link 名称（用于添加球形碰撞体，替代网格碰撞）
WHEEL_LINKS = {"Link4", "Link10"}
WHEEL_RADIUS = 0.034    # 车轮半径（m），由网格顶点范围推算

# 固定关节：膝关节(joint2/joint8) + 平行链下段(joint6/joint12)
# joint5/joint11 不固定，改用等式约束跟随主髋关节，保持5-bar视觉完整性
FIXED_JOINTS = {"joint6", "joint12", "joint2", "joint8"}

# 车轮关节：使用较高阻尼防止自由旋转
WHEEL_JOINTS = {"joint4", "joint10"}

# ─── 控制状态 ─────────────────────────────────────────────────────────────────
state = {"forward": 0.0, "steer": 0.0, "leg_q": 0.0}
# 配重后总质量 ≈ 2.1 kg，每轮最大摩擦力矩 ≈ 0.13 N*m
TORQUE_STEP = 0.01   # 每次按键力矩增量（N*m）
ANGLE_STEP  = 0.05   # 每次按键角度增量（rad）


# ─── URDF 解析 ────────────────────────────────────────────────────────────────

def parse_urdf(filepath):
    """
    解析 rh1.urdf，返回 (links, joints) 两个有序字典。

    links  : { link_name -> {pos, mass, inertia(MuJoCo顺序), mesh} }
    joints : { joint_name -> {type, parent, child, xyz, rpy, axis, lower, upper} }

    注意：
      - 仅解析顶层 <link>/<joint>，跳过 <gazebo> 内的 Gazebo 专属定义
      - 注释掉的 Link6_3/Link12_9 不会出现在 XML 输出中
      - inertia 顺序转换为 MuJoCo fullinertia 格式：ixx iyy izz ixy ixz iyz
    """
    tree = ET.parse(filepath)
    root = tree.getroot()

    # ── Links ──────────────────────────────────────────────────────────────
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            continue

        origin = inertial.find("origin")
        if origin is not None:
            pos = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        else:
            pos = [0.0, 0.0, 0.0]

        mass_el = inertial.find("mass")
        mass = float(mass_el.get("value")) if mass_el is not None else 1e-3

        ine = inertial.find("inertia")
        if ine is not None:
            ixx = float(ine.get("ixx", 1e-6))
            ixy = float(ine.get("ixy", 0))
            ixz = float(ine.get("ixz", 0))
            iyy = float(ine.get("iyy", 1e-6))
            iyz = float(ine.get("iyz", 0))
            izz = float(ine.get("izz", 1e-6))
        else:
            ixx = iyy = izz = 1e-6
            ixy = ixz = iyz = 0.0

        mesh_file = None
        visual = link.find("visual")
        if visual is not None:
            geo = visual.find("geometry")
            if geo is not None:
                mel = geo.find("mesh")
                if mel is not None:
                    fn = mel.get("filename", "")
                    mesh_file = fn.split("/")[-1]

        links[name] = {
            "pos":  pos,
            "mass": mass,
            # MuJoCo fullinertia 顺序：ixx iyy izz ixy ixz iyz
            "inertia": (ixx, iyy, izz, ixy, ixz, iyz),
            "mesh": mesh_file,
        }

    # ── Joints ─────────────────────────────────────────────────────────────
    joints = {}
    for joint in root.findall("joint"):
        name  = joint.get("name")
        jtype = joint.get("type")
        if jtype is None or jtype == "fixed":
            continue

        parent_el = joint.find("parent")
        child_el  = joint.find("child")
        if parent_el is None or child_el is None:
            continue

        parent = parent_el.get("link")
        child  = child_el.get("link")

        # 跳过指向不存在 link（被注释掉的 Link6_3 / Link12_9）的 joint
        if child not in links:
            continue

        origin = joint.find("origin")
        if origin is not None:
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        else:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]

        axis_el = joint.find("axis")
        if axis_el is not None:
            axis = [float(v) for v in axis_el.get("xyz", "0 0 1").split()]
        else:
            axis = [0.0, 0.0, 1.0]

        limit_el = joint.find("limit")
        lower = upper = None
        if limit_el is not None:
            ls = limit_el.get("lower")
            us = limit_el.get("upper")
            if ls:
                lower = float(ls)
            if us:
                upper = float(us)

        joints[name] = {
            "type":   jtype,
            "parent": parent,
            "child":  child,
            "xyz":    xyz,
            "rpy":    rpy,
            "axis":   axis,
            "lower":  lower,
            "upper":  upper,
        }

    return links, joints


# ─── MuJoCo XML 生成 ──────────────────────────────────────────────────────────

def _build_children_map(joints):
    """返回 {parent_link: [joint_name, ...]} 的映射（保持 URDF 中的顺序）。"""
    m = {}
    for jname, j in joints.items():
        m.setdefault(j["parent"], []).append(jname)
    return m


def _body_xml(prefix, link_name, links, joints, children_map, depth):
    """
    递归生成 link_name 对应 body 的内部 XML 行列表。
    depth : 当前缩进层级（每层 2 个空格）。
    返回的每行已包含正确的前置空格。
    """
    ind  = "  " * depth
    ind2 = "  " * (depth + 1)
    link = links.get(link_name, {})
    lines = []

    # <inertial>
    if "mass" in link:
        p = link["pos"]
        m = link["mass"]
        i = link["inertia"]
        lines.append(
            f'{ind}<inertial'
            f' pos="{p[0]:.6g} {p[1]:.6g} {p[2]:.6g}"'
            f' mass="{m:.6g}"'
            f' fullinertia="{i[0]:.6g} {i[1]:.6g} {i[2]:.6g}'
            f' {i[3]:.6g} {i[4]:.6g} {i[5]:.6g}"/>'
        )

    # <geom>（使用网格，关闭碰撞以避免初始穿透爆炸）
    if link.get("mesh"):
        lines.append(
            f'{ind}<geom type="mesh" mesh="{prefix}_{link_name}"'
            f' contype="0" conaffinity="0"/>'
        )
    # 车轮：用球体替代网格做碰撞检测
    if link_name in WHEEL_LINKS:
        lines.append(
            f'{ind}<geom type="sphere" size="{WHEEL_RADIUS}"'
            f' contype="1" conaffinity="1" rgba="0.1 0.1 0.1 1"/>'
        )

    # 子 body（按 URDF 顺序遍历）
    for jname in children_map.get(link_name, []):
        j     = joints[jname]
        child = j["child"]
        xyz   = j["xyz"]
        rpy   = j["rpy"]
        axis  = j["axis"]
        jtype = j["type"]

        # <body> 开始标签
        lines.append(
            f'{ind}<body name="{prefix}_{child}"'
            f' pos="{xyz[0]:.6g} {xyz[1]:.6g} {xyz[2]:.6g}"'
            f' euler="{rpy[0]:.6g} {rpy[1]:.6g} {rpy[2]:.6g}">'
        )

        # <joint>（revolute → 有限位；continuous → 无限位；平行链关节 → 固定无 DOF）
        if jname not in FIXED_JOINTS and jtype in ("revolute", "continuous"):
            if jtype == "revolute" and j["lower"] is not None and j["upper"] is not None:
                range_attr = f'limited="true" range="{j["lower"]:.6g} {j["upper"]:.6g}"'
            else:
                range_attr = 'limited="false"'
            wheel_damping = "0.001" if jname in WHEEL_JOINTS else "0.1"
            lines.append(
                f'{ind2}<joint name="{prefix}_{jname}" type="hinge"'
                f' axis="{axis[0]:.6g} {axis[1]:.6g} {axis[2]:.6g}"'
                f' {range_attr} damping="{wheel_damping}"/>'
            )

        # 递归生成子 body 内容
        lines.extend(_body_xml(prefix, child, links, joints, children_map, depth + 1))
        lines.append(f"{ind}</body>")

    return lines


def generate_xml(links, joints):
    """
    生成完整的 MuJoCo XML 字符串。

    结构：
      F_base_link（带 freejoint，前机器人根节点）
        └─ F_Link1 … F_Link12（前机器人所有关节/连杆）
        └─ R_base_link（刚性固连，后机器人，X 轴偏移 REAR_OFFSET_X）
              └─ R_Link1 … R_Link12（后机器人所有关节/连杆）

    执行器（共 12 个，顺序固定）：
      [0..3]  髋关节位置伺服（Q/E 调节腿高）
      [4..7]  膝关节位置伺服（固定为 0，稳定腿型）
      [8..11] 轮子速度控制（W/S/A/D）
    """
    children_map = _build_children_map(joints)

    # ── 资产（每个 STL 生成 F_/R_ 两份） ──────────────────────────────────
    asset_lines = []
    for lname, ldata in links.items():
        mesh = ldata.get("mesh")
        if mesh:
            asset_lines.append(f'    <mesh name="F_{lname}" file="{mesh}"/>')
            asset_lines.append(f'    <mesh name="R_{lname}" file="{mesh}"/>')

    # ── body XML（前机器人内容在 depth=3，后机器人内容在 depth=4） ─────────
    front_contents = _body_xml("F", "base_link", links, joints, children_map, depth=3)
    rear_contents  = _body_xml("R", "base_link", links, joints, children_map, depth=4)

    xml_lines = [
        '<mujoco model="rh1_quadruped">',
        '  <compiler meshdir="meshes/visual" angle="radian"/>',
        f'  <option gravity="0 0 -9.81" timestep="{TIMESTEP}"/>',
        '  <default>',
        '    <joint damping="0.1"/>',
        '    <geom contype="1" conaffinity="1" friction="1.0 0.005 0.0001" rgba="0.75 0.75 0.75 1"/>',
        '  </default>',
        '',
        '  <asset>',
        *asset_lines,
        '  </asset>',
        '',
        '  <worldbody>',
        '    <light directional="true" pos="0 0 3" dir="0 0 -1"/>',
        '    <geom name="ground" type="plane" size="20 20 0.1" rgba="0.3 0.6 0.3 1"/>',
        '',
        f'    <body name="F_base_link" pos="0 0 {INIT_HEIGHT}">',
        '      <freejoint name="root"/>',
        '      <!-- 配重：增加正压力使轮子获得足够摩擦力 -->',
        '      <body name="chassis_weight" pos="-0.085 0 0">',
        '        <inertial pos="0 0 0" mass="1.5"'
        ' fullinertia="0.01 0.01 0.01 0 0 0"/>',
        '      </body>',
        *front_contents,
        '',
        f'      <body name="R_base_link" pos="{REAR_OFFSET_X} 0 0">',
        *rear_contents,
        '      </body>',
        '',
        '    </body>',
        '  </worldbody>',
        '',
        '  <!-- 5-bar 平行链等式约束：joint5/joint11 跟随主髋关节同角度转动 -->',
        '  <equality>',
        '    <joint joint1="F_joint5"  joint2="F_joint1"  polycoef="0 1 0 0 0"/>',
        '    <joint joint1="F_joint11" joint2="F_joint7"  polycoef="0 1 0 0 0"/>',
        '    <joint joint1="R_joint5"  joint2="R_joint1"  polycoef="0 1 0 0 0"/>',
        '    <joint joint1="R_joint11" joint2="R_joint7"  polycoef="0 1 0 0 0"/>',
        '  </equality>',
        '',
        '  <actuator>',
        '    <!-- 髋关节位置伺服 (ctrl[0..3])：Q/E 腿高控制 -->',
        '    <position name="F_hip_L"  joint="F_joint1"  kp="100" ctrlrange="-0.8 0.8"/>',
        '    <position name="F_hip_R"  joint="F_joint7"  kp="100" ctrlrange="-0.8 0.8"/>',
        '    <position name="R_hip_L"  joint="R_joint1"  kp="100" ctrlrange="-0.8 0.8"/>',
        '    <position name="R_hip_R"  joint="R_joint7"  kp="100" ctrlrange="-0.8 0.8"/>',
        '    <!-- 轮子力矩执行器 (ctrl[4..7])：W/S 前进/后退，A/D 转向 -->',
        '    <!-- motor 执行器直接施加力矩，不产生速度反馈的爆炸性制动力 -->',
        '    <motor name="F_wheel_L" joint="F_joint4"  ctrlrange="-0.1 0.1"/>',
        '    <motor name="F_wheel_R" joint="F_joint10" ctrlrange="-0.1 0.1"/>',
        '    <motor name="R_wheel_L" joint="R_joint4"  ctrlrange="-0.1 0.1"/>',
        '    <motor name="R_wheel_R" joint="R_joint10" ctrlrange="-0.1 0.1"/>',
        '  </actuator>',
        '</mujoco>',
    ]

    return "\n".join(xml_lines)


# ─── 键盘回调 ─────────────────────────────────────────────────────────────────

def key_callback(keycode):
    """MuJoCo viewer 按键回调，更新全局控制状态。"""
    ch = chr(keycode).upper() if 32 <= keycode < 128 else ""
    if   ch == "W":
        state["forward"] = min(state["forward"] + TORQUE_STEP,  0.1)
    elif ch == "S":
        state["forward"] = max(state["forward"] - TORQUE_STEP, -0.1)
    elif ch == "A":
        state["steer"]   = min(state["steer"]   + TORQUE_STEP,  0.05)  # 左转
    elif ch == "D":
        state["steer"]   = max(state["steer"]   - TORQUE_STEP, -0.05)  # 右转
    elif ch == "Q":
        state["leg_q"]   = min(state["leg_q"]   + ANGLE_STEP, 0.8)
    elif ch == "E":
        state["leg_q"]   = max(state["leg_q"]   - ANGLE_STEP, 0.0)
    elif keycode == 32:   # Space — 停止
        state["forward"] = 0.0
        state["steer"]   = 0.0


# ─── 控制更新 ─────────────────────────────────────────────────────────────────
# 执行器索引（膝关节固定后移除，轮子前移）：
#   [0] F_hip_L   [1] F_hip_R   [2] R_hip_L   [3] R_hip_R
#   [4] F_wheel_L [5] F_wheel_R [6] R_wheel_L  [7] R_wheel_R

def apply_control(data):
    """将控制状态写入 data.ctrl。"""
    q     = state["leg_q"]
    fwd   = state["forward"]
    steer = state["steer"]

    # 髋关节：左腿正角，右腿负角，实现左右对称展开
    data.ctrl[0] =  q   # F_hip_L  (F_joint1)
    data.ctrl[1] = -q   # F_hip_R  (F_joint7)
    data.ctrl[2] =  q   # R_hip_L  (R_joint1)
    data.ctrl[3] = -q   # R_hip_R  (R_joint7)

    # 差速驱动（motor 执行器，ctrl 单位 N*m）
    # 左轮轴 +Y (joint1 Rx+90°)：正力矩=前进
    # 右轮轴 -Y (joint7 Rx-90°)：负力矩=前进（轴镜像）
    # D键增大 steer → 左轮力矩大/右轮小 → 右转
    left_torque  = fwd + steer
    right_torque = fwd - steer
    data.ctrl[4] =  left_torque    # F_wheel_L (F_joint4,  axis +Y)
    data.ctrl[5] = -right_torque   # F_wheel_R (F_joint10, axis -Y, 取反)
    data.ctrl[6] =  left_torque    # R_wheel_L (R_joint4,  axis +Y)
    data.ctrl[7] = -right_torque   # R_wheel_R (R_joint10, axis -Y, 取反)


# ─── 主程序 ──────────────────────────────────────────────────────────────────

def main():
    # 1. 解析 URDF
    links, joints = parse_urdf(URDF_PATH)
    print(f"解析完成：{len(links)} 个 link，{len(joints)} 个 joint")

    # 2. 生成 MuJoCo XML
    xml = generate_xml(links, joints)

    # 保存生成的 XML 供调试检查
    with open("quadruped_generated.xml", "w") as f:
        f.write(xml)
    print("XML 已保存至 quadruped_generated.xml")

    # 3. 加载模型
    model = mujoco.MjModel.from_xml_string(xml)
    data  = mujoco.MjData(model)

    # freejoint qpos 布局：[x, y, z, qw, qx, qy, qz, joint_angles...]
    # 初始 z 高度让机器人悬空后自然落地
    data.qpos[2] = INIT_HEIGHT

    # 4. 启动仿真
    print("\n四轮足机器人 MuJoCo 仿真已启动")
    print("─" * 42)
    print("  W / S   — 前进 / 后退")
    print("  A / D   — 左转 / 右转")
    print("  Q / E   — 腿高 ↑ / ↓")
    print("  Space   — 停止（速度清零）")
    print("─" * 42)

    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback
    ) as viewer:
        while viewer.is_running():
            apply_control(data)
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
