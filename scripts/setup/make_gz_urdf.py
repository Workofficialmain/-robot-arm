import os
PKG = "/home/pavan/robot_arm_ws/src/robot_arm_description"
os.makedirs(PKG + "/config", exist_ok=True)
os.makedirs(PKG + "/launch", exist_ok=True)

ORIGIN = (0.0, 0.0, 0.0)
TURN = (0.0, 0.0, 12.5)
SH = (0.0, 0.0, 52.0)
EL = (30.0, 0.0, 288.8)
TIP = (378.0, 0.0, 293.0)
LIFT = 80.0  # base bottom is at z=-80 mm, lift so it sits on the ground

def m(v):
    return " ".join("%.6g" % (x / 1000.0) for x in v)

def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))

def visuals(names, pivot):
    o = ""
    for n in names:
        o += ('\n    <visual>\n      <origin xyz="%s" rpy="0 0 0"/>\n      <geometry>\n'
              '        <mesh filename="package://robot_arm_description/meshes/%s.stl" scale="0.001 0.001 0.001"/>\n'
              '      </geometry>\n    </visual>\n') % (m(tuple(-x for x in pivot)), n)
    return o

def collisions(boxes):
    o = ""
    for c, s in boxes:
        o += ('\n    <collision>\n      <origin xyz="%s" rpy="0 0 0"/>\n      <geometry><box size="%s"/></geometry>\n'
              '    </collision>\n') % (m(c), m(s))
    return o

def inertial(mass, c, s):
    x, y, z = [v / 1000.0 for v in s]
    ixx = mass / 12.0 * (y * y + z * z)
    iyy = mass / 12.0 * (x * x + z * z)
    izz = mass / 12.0 * (x * x + y * y)
    return ('\n    <inertial>\n      <origin xyz="%s" rpy="0 0 0"/>\n      <mass value="%g"/>\n'
            '      <inertia ixx="%.6g" ixy="0" ixz="0" iyy="%.6g" iyz="0" izz="%.6g"/>\n    </inertial>\n') % (m(c), mass, ixx, iyy, izz)

def joint(name, parent, child, origin, axis, lo, hi):
    return ('\n  <joint name="%s" type="revolute">\n    <parent link="%s"/>\n    <child link="%s"/>\n'
            '    <origin xyz="%s" rpy="0 0 0"/>\n    <axis xyz="%s"/>\n'
            '    <limit lower="%s" upper="%s" effort="10.0" velocity="2.0"/>\n'
            '    <dynamics damping="0.5" friction="0.1"/>\n  </joint>\n') % (name, parent, child, m(origin), axis, lo, hi)

def link(name, mass, inbox, boxes, names, pivot):
    return '\n  <link name="%s">%s%s%s  </link>\n' % (name, inertial(mass, inbox[0], inbox[1]), visuals(names, pivot), collisions(boxes))

base_boxes = [((0, 0, -41.5), (146, 146, 77)), ((113.2, 0, -70), (100.6, 40, 20)), ((-113.2, 0, -70), (100.6, 40, 20)),
              ((0, 113.2, -70), (40, 100.6, 20)), ((0, -113.2, -70), (40, 100.6, 20))]
turn_boxes = [((0, 0, 24.75), (55, 76, 49.5))]
upper_boxes = [((0, 0, 124), (20, 38, 200)), ((42.5, 0, 236.5), (45, 38, 25))]
fore_boxes = [((101, 0, 5.2), (140, 16, 16)), ((262.5, -1.5, 4.2), (171, 97, 50))]

s = '<?xml version="1.0"?>\n<robot name="robot_arm">\n\n  <link name="world"/>\n'
s += ('\n  <joint name="world_to_base" type="fixed">\n    <parent link="world"/>\n    <child link="robot_base"/>\n'
      '    <origin xyz="0 0 %g" rpy="0 0 0"/>\n  </joint>\n') % (LIFT / 1000.0)
s += link("robot_base", 1.0, ((0, 0, -40), (326, 326, 80)), base_boxes,
          ["base", "base_leg_1", "base_leg_2", "base_leg_3", "base_leg_4", "motor_base"], ORIGIN)
s += joint("joint_base_rotate", "robot_base", "link_turntable", sub(TURN, ORIGIN), "0 0 1", -3.14, 3.14)
s += link("link_turntable", 0.4, turn_boxes[0], turn_boxes, ["arm_base", "motor_shoulder_1", "motor_shoulder_2"], TURN)
s += joint("joint_shoulder", "link_turntable", "link_upper_arm", sub(SH, TURN), "0 1 0", -1.57, 1.57)
s += link("link_upper_arm", 0.5, upper_boxes[0], upper_boxes, ["shoulder", "upper_arm", "elbow_a", "motor_elbow"], SH)
s += joint("joint_elbow", "link_upper_arm", "link_forearm", sub(EL, SH), "0 1 0", -1.57, 1.57)
s += link("link_forearm", 0.5, fore_boxes[1], fore_boxes, ["elbow_b", "forearm", "motor_wrist", "leg_holder_elbow", "leg_holder_wrist", "screws_forearm"], EL)
s += ('\n  <joint name="tip_joint" type="fixed">\n    <parent link="link_forearm"/>\n    <child link="end_effector"/>\n'
      '    <origin xyz="%s" rpy="0 0 0"/>\n  </joint>\n') % m(sub(TIP, EL))
s += ('\n  <link name="end_effector">\n    <inertial>\n      <mass value="0.001"/>\n'
      '      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>\n    </inertial>\n  </link>\n')

s += '\n  <ros2_control name="GazeboSystem" type="system">\n    <hardware>\n      <plugin>gazebo_ros2_control/GazeboSystem</plugin>\n    </hardware>\n'
for j in ["joint_base_rotate", "joint_shoulder", "joint_elbow"]:
    s += ('    <joint name="%s">\n      <command_interface name="position"/>\n'
          '      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>\n'
          '      <state_interface name="velocity"/>\n    </joint>\n') % j
s += '  </ros2_control>\n'
s += ('\n  <gazebo>\n    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">\n'
      '      <parameters>%s/config/controllers.yaml</parameters>\n    </plugin>\n  </gazebo>\n\n</robot>\n') % PKG

open(PKG + "/urdf/robot_arm_gz.urdf", "w").write(s)
print("urdf written")
