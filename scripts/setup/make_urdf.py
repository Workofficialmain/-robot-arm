import os, shutil

P = "/home/pavan/robot_arm_ws/src/robot_arm_description/urdf/robot_arm.urdf"
if os.path.exists(P):
    shutil.copy(P, P + ".bak")

ORIGIN = (0.0, 0.0, 0.0)
TURN = (0.0, 0.0, 12.5)
SH = (0.0, 0.0, 52.0)
EL = (30.0, 0.0, 288.8)

def f(v):
    return " ".join("%.6g" % (x / 1000.0 + 0.0) for x in v)

def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))

def visuals(names, pivot):
    neg = tuple(-x for x in pivot)
    out = ""
    for n in names:
        out += f'''
    <visual>
      <origin xyz="{f(neg)}" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_arm_description/meshes/{n}.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
'''
    return out

def joint(name, parent, child, origin, axis, lo, hi):
    return f'''
  <joint name="{name}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{f(origin)}" rpy="0 0 0"/>
    <axis xyz="{axis}"/>
    <limit lower="{lo}" upper="{hi}" effort="5.0" velocity="1.5"/>
  </joint>
'''

base = ["base", "base_leg_1", "base_leg_2", "base_leg_3", "base_leg_4", "motor_base"]
turn = ["arm_base", "motor_shoulder_1", "motor_shoulder_2"]
upper = ["shoulder", "upper_arm", "elbow_a", "motor_elbow"]
fore = ["elbow_b", "forearm", "motor_wrist", "leg_holder_elbow", "leg_holder_wrist", "screws_forearm"]

s = '<?xml version="1.0"?>\n<robot name="robot_arm">\n'
s += '\n  <link name="world"/>\n'
s += '''
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="robot_base"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
'''
s += '\n  <link name="robot_base">' + visuals(base, ORIGIN) + '  </link>\n'
s += joint("joint_base_rotate", "robot_base", "link_turntable", sub(TURN, ORIGIN), "0 0 1", -3.14, 3.14)
s += '\n  <link name="link_turntable">' + visuals(turn, TURN) + '  </link>\n'
s += joint("joint_shoulder", "link_turntable", "link_upper_arm", sub(SH, TURN), "0 1 0", -1.57, 1.57)
s += '\n  <link name="link_upper_arm">' + visuals(upper, SH) + '  </link>\n'
s += joint("joint_elbow", "link_upper_arm", "link_forearm", sub(EL, SH), "0 1 0", -1.57, 1.57)
s += '\n  <link name="link_forearm">' + visuals(fore, EL) + '  </link>\n'
s += '\n</robot>\n'

open(P, "w").write(s)
print("written", P)
