# 3-DOF Robotic Manipulator with Adaptive Gripper — Computer Vision and Pick-and-Place Automation

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_Hawksbill-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo_Classic-orange?logo=gazebo&logoColor=white)](https://classic.gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-blue?logo=python&logoColor=white)](https://www.python.org/)
[![GitHub](https://img.shields.io/badge/Repository-Workofficialmain%2F-robot--arm-181717?logo=github&logoColor=white)](https://github.com/Workofficialmain/-robot-arm)

A complete end-to-end robotics project developing a **3-Degree-of-Freedom (3-DOF) articulated robotic arm**. The system covers the entire robotics lifecycle: from FreeCAD joint extraction and custom URDF physics modeling to analytical forward/inverse kinematics, Jacobian differential velocity mapping, damped least squares (DLS) singularity handling, quintic polynomial trajectory interpolation, dynamic safety validation, and ROS 2 `FollowJointTrajectory` execution and joint-state-based TCP tracking-error measurement in Gazebo.

---

## Table of Contents

1. [Project Workflow (Step-by-Step in Chronological Order)](#1-project-workflow-step-by-step-in-chronological-order)
2. [Robot Mechanical & Kinematic Specifications](#2-robot-mechanical--kinematic-specifications)
3. [Kinematics & Mathematics Deep-Dive](#3-kinematics--mathematics-deep-dive)
   - [3.1 Forward Kinematics (FK)](#31-forward-kinematics-fk)
   - [3.2 Inverse Kinematics (IK) & Dual Configurations](#32-inverse-kinematics-ik--dual-configurations)
   - [3.3 Jacobian Matrix (Differential Kinematics)](#33-jacobian-matrix-differential-kinematics)
   - [3.4 Inverse Jacobian & Damped Least Squares (DLS)](#34-inverse-jacobian--damped-least-squares-dls)
   - [3.5 How Everything Links Together (System Pipeline)](#35-how-everything-links-together-system-pipeline)
   - [3.6 Implementation in Code: From Equations to `kinematics.py`](#36-implementation-in-code-from-equations-to-kinematicspy)
4. [Cartesian Shape Tracing: Square and Circle](#4-cartesian-shape-tracing-square-and-circle)
5. [Testing, Dry-Runs, and Verification](#5-testing-dry-runs-and-verification)
6. [Repository Architecture & File Map (Tabular Guide)](#6-repository-architecture--file-map-tabular-guide)
7. [How to Build and Run the Project](#7-how-to-build-and-run-the-project)
8. [ROS 2 Controllers and Control Stack Used](#8-ros-2-controllers-and-control-stack-used)

---

## 1. Project Workflow (Step-by-Step in Chronological Order)

The project was constructed following a structured 12-step engineering lifecycle. Each step builds directly upon the outputs of the previous step.

```mermaid
flowchart TD
    S1["Step 1: CAD & Axis Extraction<br/>(FreeCAD)"] --> S2["Step 2: URDF & Gazebo Simulation<br/>(Physics & ros2_control)"]
    S2 --> S3["Step 3: Forward Kinematics<br/>(Joint Space -> Cartesian)"]
    S3 --> S4["Step 4: Inverse Kinematics<br/>(Cartesian -> Joint Space)"]
    S4 --> S5["Step 5: Jacobian Derivation<br/>(Joint Velocity -> TCP Velocity)"]
    S5 --> S6["Step 6: Inverse Jacobian & DLS<br/>(Singularity-Robust Control)"]
    S6 --> S7["Step 7: Quintic Trajectory<br/>(Smooth C² Boundary Conditions)"]
    S7 --> S8["Step 8: Shape Path Planners<br/>(Lines, Circles, Rounded Squares)"]
    S8 --> S9["Step 9: Safety & Time Scaling<br/>(Position, Velocity, Accel Limits)"]
    S9 --> S10["Step 10: ROS 2 Controller Action<br/>(FollowJointTrajectory)"]
    S10 --> S11["Step 11: Real-Time Visualization<br/>(Gazebo SDF Trail Spawning)"]
    S11 --> S12["Step 12: Dry-Runs & Error Analysis<br/>(Closed-Loop Validation)"]
```

---

### Step 1: CAD Modeling & Joint Axis Extraction
* **What we did**: Analyzed 19 custom 3D STL parts in FreeCAD. Using Python inspection scripts (`shoulder_axes.py`, `elbow_wrist_axes.py`, `wrist_axes.py`), we extracted the exact geometric rotational pivot centers, link offsets, and joint axes of the arm.
* **Why we did it**: Real-world robots cannot be simulated accurately using guessed dimensions. Every joint rotation axis and link center of mass must align precisely with the physical mechanical CAD assemblies.
* **What it gives us / Output**:
  - Shoulder joint pivot: $(X=0.0, Y=0.0, Z=52.0)\text{ mm}$
  - Elbow joint pivot: $(X=30.0, Y=0.0, Z=288.8)\text{ mm}$
  - Tool Center Point (TCP / Tip): $(X=378.0, Y=0.0, Z=293.0)\text{ mm}$
  - Clean, correctly scaled millimeter STL mesh files for 3D visualization.

---

### Step 2: URDF Simulation & ros2_control Setup
* **What we did**: Authored automated URDF generation scripts (`make_urdf.py` and `make_gz_urdf.py`). These scripts assemble the STL meshes into a kinematic chain (`world` $\to$ `robot_base` $\to$ `link_turntable` $\to$ `link_upper_arm` $\to$ `link_forearm` $\to$ `end_effector`), configure mass and inertia tensors, set joint limits, lift the base by $80\text{ mm}$ so it rests on the Gazebo floor, and configure `gazebo_ros2_control` tags.
* **Why we did it**: Gazebo requires a dynamic description format with inertia tensors, collision bounding boxes, and transmission controllers to simulate physics (gravity, torque, friction) realistically.
* **What it gives us / Output**:
  - `robot_arm.urdf`: Visual URDF for RViz visualization.
  - `robot_arm_gz.urdf`: Full dynamic physics URDF for Gazebo simulation.
  - `controllers.yaml`: Hardware controller definitions for `joint_state_broadcaster` and `arm_controller` (`JointTrajectoryController`).

---

### Step 3: Analytical Forward Kinematics (FK)
* **What we did**: Derived closed-form trigonometric equations that take the 3 joint angles $q = [q_1, q_2, q_3]^T$ and output the exact $(x, y, z)$ position of the tool tip in 3D Cartesian space.
* **Why we did it**: The robot needs to know where its end-effector is in real-world coordinates based on live motor encoder readings.
* **What it gives us / Output**:
  - Function `forward_kinematics(q)` returning $(x, y, z)$ in millimeters with sub-nanometer numerical precision.

---

### Step 4: Analytical Inverse Kinematics (IK) & Dual Configurations
* **What we did**: Formulated geometric inverse kinematics using the Law of Cosines. Implemented reachability checking (workspace boundary validation) and solved for both **Elbow-Up** ($\delta < 0$) and **Elbow-Down** ($\delta > 0$) kinematic configurations.
* **Why we did it**: When planning tasks (such as tracing a square or line), the user commands where the tool tip should be in 3D space. The robot's motors, however, only rotate in joint space. IK translates desired 3D coordinates into required motor angles.
* **What it gives us / Output**:
  - Function `inverse_kinematics(x, y, z, elbow="up")` providing instant analytical joint configurations without iterative optimization drift.

---

### Step 5: Differential Kinematics & Analytical Jacobian
* **What we did**: Computed the exact partial derivatives of the forward kinematics functions with respect to each joint angle, creating the $3 \times 3$ Jacobian matrix $J(q)$.
* **Why we did it**: While position kinematics determines the final target, continuous motion control requires knowing how joint velocities $(\dot{q}_1, \dot{q}_2, \dot{q}_3)$ relate to linear tip velocities $(\dot{x}, \dot{y}, \dot{z})$. It is also the fundamental tool used to identify kinematic singularities.
* **What it gives us / Output**:
  - Matrix equation $V = J(q) \dot{q}$ evaluated at any robot pose.
  - Analytical singularity metric $\mu(q)$ identifying when the arm is near full stretch or self-folding.

---

### Step 6: Inverse Jacobian & Damped Least Squares (DLS)
* **What we did**: Implemented the Damped Least Squares (Levenberg-Marquardt pseudoinverse) inversion algorithm:
  $$J_{\text{damped}}^+ = (J^T J + \lambda^2 I)^{-1} J^T$$
  with adaptive damping $\lambda(q)$ calculated from the local singularity index.
* **Why we did it**: If the robot arm approaches a kinematic singularity, the standard inverse matrix $J^{-1}$ experiences a division by near-zero ($\det(J) \to 0$). Direct inversion commands near-infinite joint speeds, causing controller crashes and dangerous physical oscillations. DLS prevents speed blow-ups while maintaining path direction.
* **What it gives us / Output**:
  - Function `inverse_jacobian(q, damping)` that produces stable, safe joint velocities even when passing through or near singularities.

---

### Step 7: Quintic Polynomial Trajectory Generation
* **What we did**: Implemented 5th-order (quintic) polynomial interpolation for trajectories:
  $$s(u) = 10u^3 - 15u^4 + 6u^5, \quad u = \frac{t}{T} \in [0, 1]$$
* **Why we did it**: Linear interpolation creates infinite jerk and abrupt velocity steps at start and stop. Cubic polynomials have discontinuous accelerations. Quintic polynomials enforce zero velocity and zero acceleration at both the start and end ($C^2$ continuity), preventing mechanical wear, motor stalling, and structural vibration.
* **What it gives us / Output**:
  - Trajectory module `trajectories.py` outputting smooth joint profiles with zero velocity/acceleration boundary conditions.

---

### Step 8: Cartesian Shape Path Planning (Line, Circle, Rounded Square)
* **What we did**: Created path generators that discretize 2D/3D shapes into dense trajectory waypoints (50 Hz) and combine Cartesian path geometry, quintic time-scaling, IK, and DLS velocity mapping:
  1. **Straight Line**: Smooth linear interpolation between two 3D points.
  2. **Circle**: Parametric circular path with constant curvature.
  3. **Rounded Square**: Four straight segments joined by four circular blend arcs ($r_{\text{blend}} = 4.0\text{ mm}$) to avoid infinite corner acceleration.
* **Why we did it**: Real applications (dispensing, inspection, welding) require following geometric shapes in physical space.
* **What it gives us / Output**:
  - Modules `cartesian_planner.py` and `shape_planner.py` generating complete time-parameterized waypoint streams.

---

### Step 9: Safety Validation & Dynamic Time Scaling
* **What we did**: Built a dedicated safety validation engine (`safety.py`) checking every waypoint against:
  - Position limits (URDF joint bounds)
  - Velocity limits ($\le 2.0\text{ rad/s}$)
  - Acceleration limits ($\le 4.0\text{ rad/s}^2$)
  If any sample exceeds limits, an automated time-scaling algorithm computes a slowdown factor $s \le 1.0$ and stretches the execution duration without altering the geometric path.
* **Why we did it**: Helps keep planned joint positions, velocities, and configured acceleration limits within software limits. The configured acceleration ceiling is a planning limit, not a verified actuator torque or hardware rating.
* **What it gives us / Output**:
  - Fail-safe trajectory verification and automated retiming before execution.

---

### Step 10: ROS 2 Controller Action Integration
* **What we did**: Implemented `ros_trajectory.py` and `trajectory_publisher.py` interfacing with the ROS 2 `control_msgs/action/FollowJointTrajectory` action server on `/arm_controller/follow_joint_trajectory`.
* **Why we did it**: Connects our custom motion planner to standard ROS 2 and Gazebo control infrastructure, supporting asynchronous execution, preemption, and real-time state feedback.
* **What it gives us / Output**:
  - Seamless ROS 2 action client node capable of dry-run validation and live execution.

---

### Step 11: Real-Time Gazebo Path Tracing
* **What we did**: Implemented interactive Gazebo SDF marker spawners (`square_guide.py`, `circle_guide.py`, `clear_marks.py`, and demo scripts) that dynamically call the `/spawn_entity` service to drop colored spheres at the end-effector tip in real-time.
* **Why we did it**: Provides clear visual proof in 3D simulation that the robot accurately followed the prescribed square or circular path on the work surface.
* **What it gives us / Output**:
  - Blue sphere at home position, red sphere at approach height, black guide discs on the ground, and continuous red trail dots along the path.

---

### Step 12: Dry-Run Testing & Tracking Error Measurement
* **What we did**: Validated the entire pipeline through dry-run tests (all math and safety checks executed without moving the robot) followed by an explicitly authorized straight-line Gazebo simulation execution. The node sampled `/joint_states` and used forward kinematics to calculate Root Mean Square (RMS) Cartesian tracking errors. This measures tracking performance; it does not perform closed-loop Cartesian correction.
* **Why we did it**: Provides numerical evidence of mathematical consistency, software validation, and tracking performance in simulation. It does not establish real-hardware accuracy.
* **What it gives us / Output**:
  - Quantitative accuracy report (RMS tracking error: $0.1207\text{ mm}$, maximum error: $0.1878\text{ mm}$, final position error: $0.0115\text{ mm}$).

---

## 2. Robot Mechanical & Kinematic Specifications

### 2.1 Link Lengths and Pivot Points

The robot is a 3-DOF vertical articulated arm (RRR configuration: 1 horizontal azimuth rotation, 2 planar elevation pitches).

```
          [Base Z-Axis]
                |
           (0, 0, 52) Shoulder Joint [J2]
               / \
              /   \
             /     \  Link 1 (L1 = 238.693 mm)
            /       \
           /         \
          /           (30, 0, 288.8) Elbow Joint [J3]
      (0, 0, 12.5)         \
     Turntable [J1]          \  Link 2 (L2 = 348.025 mm)
          |                   \
   (0, 0, 0) Origin            \
  =================             (378, 0, 293) Tool Tip (TCP)
  [Ground: Z = -80]
```

| Parameter | Value (mm) | Description |
|---|---|---|
| **Base Origin** | $(0.0, 0.0, 0.0)$ | CAD reference origin |
| **Turntable Joint Origin** | $(0.0, 0.0, 12.5)$ | Base rotation axis origin |
| **Shoulder Joint Pivot** | $(0.0, 0.0, 52.0)$ | Shoulder rotation axis origin |
| **Elbow Joint Pivot** | $(30.0, 0.0, 288.8)$ | Forearm pitch rotation axis origin |
| **Zero-State End-Effector (TCP)** | $(378.0, 0.0, 293.0)$ | Position when $q_1=0, q_2=0, q_3=0$ |
| **Link 1 Length ($L_1$)** | $\sqrt{30.0^2 + 236.8^2} = \mathbf{238.693\text{ mm}}$ | Distance from shoulder pivot to elbow pivot |
| **Link 2 Length ($L_2$)** | $\sqrt{348.0^2 + 4.2^2} = \mathbf{348.025\text{ mm}}$ | Distance from elbow pivot to tool tip |
| **Base Elevation Lift** | $80.0\text{ mm}$ | Lift above ground plane in Gazebo |

#### Kinematic Reference Angles
Because the CAD geometry contains fixed structural offsets, the analytical equations use zero-reference angles:
- $\Phi_{1,0} = \operatorname{atan2}(236.8, 30.0) \approx 1.44478\text{ rad} \approx 82.78^\circ$
- $\Phi_{2,0} = \operatorname{atan2}(4.2, 348.0) \approx 0.012068\text{ rad} \approx 0.691^\circ$

---

### 2.2 Joint Limits and Motion Constraints

| Joint Name | URDF Link | Lower Limit | Upper Limit | Max Velocity | Max Acceleration |
|---|---|---:|---:|---:|---:|
| `joint_base_rotate` | Turntable ($q_1$) | $-3.14\text{ rad}$ ($-180^\circ$) | $+3.14\text{ rad}$ ($+180^\circ$) | $2.0\text{ rad/s}$ | $4.0\text{ rad/s}^2$ |
| `joint_shoulder` | Upper Arm ($q_2$) | $-1.57\text{ rad}$ ($-90^\circ$) | $+1.57\text{ rad}$ ($+90^\circ$) | $2.0\text{ rad/s}$ | $4.0\text{ rad/s}^2$ |
| `joint_elbow` | Forearm ($q_3$) | $-1.57\text{ rad}$ ($-90^\circ$) | $+1.57\text{ rad}$ ($+90^\circ$) | $2.0\text{ rad/s}$ | $4.0\text{ rad/s}^2$ |

---

## 3. Kinematics & Mathematics Deep-Dive

### 3.1 Forward Kinematics (FK)

#### Why We Need It
Forward kinematics maps the joint space configurations $(q_1, q_2, q_3)$ into 3D Cartesian coordinates $(x, y, z)$. It is required to:
1. Determine the current tool tip position from motor encoder feedback.
2. Verify that planned joint paths reach the desired Cartesian coordinates.
3. Compute tracking error between commanded and actual robot poses.

#### Step-by-Step Mathematical Derivation
1. **Link elevation angles**:
   $$\phi_1 = \Phi_{1,0} - q_2$$
   $$\phi_2 = \Phi_{2,0} - q_2 - q_3$$
2. **Radial reach in the planar arm plane**:
   $$r = L_1 \cos(\phi_1) + L_2 \cos(\phi_2)$$
3. **Cartesian coordinates $(x, y, z)$**:
   $$x = r \cos(q_1)$$
   $$y = r \sin(q_1)$$
   $$z = Z_{\text{shoulder}} + L_1 \sin(\phi_1) + L_2 \sin(\phi_2) = 52.0 + L_1 \sin(\phi_1) + L_2 \sin(\phi_2)$$

#### Concrete Calculation Example
Let $q = [0.0, 0.0, 0.0]\text{ rad}$ (zero position):
1. $\phi_1 = 1.44478 - 0 = 1.44478\text{ rad} \implies \cos(\phi_1) = \frac{30.0}{238.693} \approx 0.12568, \; \sin(\phi_1) = \frac{236.8}{238.693} \approx 0.99207$
2. $\phi_2 = 0.012068 - 0 - 0 = 0.012068\text{ rad} \implies \cos(\phi_2) = \frac{348.0}{348.025} \approx 0.99993, \; \sin(\phi_2) = \frac{4.2}{348.025} \approx 0.01207$
3. Radial distance:
   $$r = 238.693 \times 0.12568 + 348.025 \times 0.99993 = 30.0 + 348.0 = 378.0\text{ mm}$$
4. 3D Coordinates:
   $$x = 378.0 \times \cos(0) = 378.0\text{ mm}$$
   $$y = 378.0 \times \sin(0) = 0.0\text{ mm}$$
   $$z = 52.0 + (238.693 \times 0.99207) + (348.025 \times 0.01207) = 52.0 + 236.8 + 4.2 = 293.0\text{ mm}$$
**Output**: $(X=378.0\text{ mm}, Y=0.0\text{ mm}, Z=293.0\text{ mm})$. Exactly matches the CAD reference!

---

### 3.2 Inverse Kinematics (IK) & Dual Configurations

#### Why We Need It
Cartesian path planning produces target waypoints $(x, y, z)$ along lines, squares, and circles. The robot motors, however, only accept joint position angles $(q_1, q_2, q_3)$. Inverse kinematics solves the geometric transformation from 3D space to joint space.

#### Step-by-Step Mathematical Derivation
Given a target coordinate $(x, y, z)$:

1. **Base rotation angle $q_1$**:
   $$q_1 = \operatorname{atan2}(y, x)$$
2. **Cylindrical projection relative to shoulder**:
   $$r = \sqrt{x^2 + y^2}$$
   $$z_t = z - Z_{\text{shoulder}} = z - 52.0$$
3. **Distance from shoulder to target ($d$) and Law of Cosines**:
   $$d^2 = r^2 + z_t^2$$
   By the Law of Cosines on the triangle formed by $L_1$, $L_2$, and $d$:
   $$D = \cos(\delta) = \frac{r^2 + z_t^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$
4. **Reachability Check**:
   If $D < -1.0$ or $D > 1.0$, the requested point is mathematically outside the physical reach of the arm.
5. **Elbow Angle Solutions**:
   The arm can reach the point in two configurations:
   - **Elbow-Up**: $\delta = -\arccos(D)$
   - **Elbow-Down**: $\delta = +\arccos(D)$
6. **Shoulder Link Angle $\phi_1$**:
   $$\phi_1 = \operatorname{atan2}(z_t, r) - \operatorname{atan2}\left(L_2 \sin \delta, \; L_1 + L_2 \cos \delta\right)$$
7. **Forearm Link Angle $\phi_2$**:
   $$\phi_2 = \phi_1 + \delta$$
8. **URDF Joint Angles**:
   $$q_2 = \Phi_{1,0} - \phi_1$$
   $$q_3 = (\Phi_{2,0} - \phi_2) - q_2$$

#### Concrete Calculation Example
Target: $(x=378.0, y=0.0, z=293.0)$ with `elbow="up"`:
1. $q_1 = \operatorname{atan2}(0, 378) = 0.0\text{ rad}$
2. $r = 378.0\text{ mm}, \; z_t = 293.0 - 52.0 = 241.0\text{ mm}$
3. $d^2 = 378^2 + 241^2 = 142884 + 58081 = 200965$
4. $L_1^2 = 56974.24, \; L_2^2 = 121121.64 \implies L_1^2 + L_2^2 = 178095.88$
5. $D = \frac{200965 - 178095.88}{2 \times 238.693 \times 348.025} = \frac{22869.12}{166142.1} \approx 0.137648$
6. $\delta = -\arccos(0.137648) \approx -1.43271\text{ rad}$
7. $\phi_1 = \operatorname{atan2}(241, 378) - \operatorname{atan2}(348.025 \sin(-1.43271), 238.693 + 348.025 \cos(-1.43271)) \approx 1.44478\text{ rad}$
8. $\phi_2 = 1.44478 - 1.43271 = 0.01207\text{ rad}$
9. $q_2 = 1.44478 - 1.44478 = 0.0\text{ rad}$
10. $q_3 = (0.01207 - 0.01207) - 0.0 = 0.0\text{ rad}$
**Solution**: $q = [0.0, 0.0, 0.0]\text{ rad}$. Perfect agreement with Forward Kinematics!

---

### 3.3 Jacobian Matrix (Differential Kinematics)

#### Purpose

Forward kinematics maps joint positions to the TCP position. The Jacobian describes how small joint motions produce TCP linear motion.

Let

\[
\mathbf q =
\begin{bmatrix}
q_1\\q_2\\q_3
\end{bmatrix},
\qquad
\dot{\mathbf q} =
\begin{bmatrix}
\dot q_1\\\dot q_2\\\dot q_3
\end{bmatrix}
\]

and

\[
\mathbf p =
\begin{bmatrix}
x\\y\\z
\end{bmatrix},
\qquad
\dot{\mathbf p} =
\begin{bmatrix}
\dot x\\\dot y\\\dot z
\end{bmatrix}.
\]

The differential-kinematics relationship is

\[
\boxed{\dot{\mathbf p}=J(\mathbf q)\dot{\mathbf q}}
\]

Here, \(J(\mathbf q)\) is a \(3\times3\) linear-velocity Jacobian. It maps the three joint velocities to the TCP's Cartesian velocity in the \(X,Y,Z\) directions. It does not map forces or torques.

#### Derivation from the Forward-Kinematics Equations

The arm geometry uses

\[
\phi_1=\Phi_{1,0}-q_2,
\qquad
\phi_2=\Phi_{2,0}-q_2-q_3
\]

and

\[
r=L_1\cos\phi_1+L_2\cos\phi_2.
\]

The TCP position is

\[
x=r\cos q_1,\qquad
y=r\sin q_1,
\]

\[
z=z_0+L_1\sin\phi_1+L_2\sin\phi_2,
\qquad z_0=52\text{ mm}.
\]

Differentiate the radial distance with respect to the shoulder and elbow joint variables:

\[
\frac{\partial r}{\partial q_2}
=L_1\sin\phi_1+L_2\sin\phi_2
\equiv d_{r2}
\]

\[
\frac{\partial r}{\partial q_3}
=L_2\sin\phi_2
\equiv d_{r3}.
\]

The individual partial derivatives are therefore

\[
\frac{\partial x}{\partial q_1}=-r\sin q_1,
\quad
\frac{\partial x}{\partial q_2}=d_{r2}\cos q_1,
\quad
\frac{\partial x}{\partial q_3}=d_{r3}\cos q_1,
\]

\[
\frac{\partial y}{\partial q_1}=r\cos q_1,
\quad
\frac{\partial y}{\partial q_2}=d_{r2}\sin q_1,
\quad
\frac{\partial y}{\partial q_3}=d_{r3}\sin q_1,
\]

\[
\frac{\partial z}{\partial q_1}=0,
\quad
\frac{\partial z}{\partial q_2}=-r,
\quad
\frac{\partial z}{\partial q_3}=-L_2\cos\phi_2.
\]

Arranging these derivatives gives the Jacobian:

\[
\boxed{
J(\mathbf q)=
\begin{bmatrix}
-r\sin q_1 & d_{r2}\cos q_1 & d_{r3}\cos q_1\\
 r\cos q_1 & d_{r2}\sin q_1 & d_{r3}\sin q_1\\
 0 & -r & -L_2\cos\phi_2
\end{bmatrix}
}
\]

#### How This Equation Is Implemented

In `scripts/motion_planning/kinematics.py`, the `jacobian(q)` function:

1. Reads the three joint positions.
2. Calculates \(\phi_1\), \(\phi_2\), and the radial distance \(r\).
3. Calculates \(d_{r2}\) and \(d_{r3}\).
4. Constructs the matrix above and returns it as a \(3\times3\) matrix.

The Jacobian is then used to relate a requested TCP velocity to the joint velocities needed to produce it. The inverse-velocity calculation is described in the next section.

### 3.4 Inverse Jacobian & Damped Least Squares (DLS)

#### Why an Ordinary Inverse Can Fail

For a nonsingular square Jacobian, the direct relationship would be

\[
\dot{\mathbf q}=J^{-1}\dot{\mathbf p}.
\]

However, at or near a singular configuration, the Jacobian becomes singular or poorly conditioned. Direct inversion can then fail or produce excessively large joint velocities.

The project therefore uses a damped least-squares (DLS) pseudoinverse for practical Cartesian velocity planning.

#### Damped Least-Squares Derivation

DLS finds joint velocity values that minimise the Cartesian velocity error while penalising unnecessarily large joint velocities:

\[
\min_{\dot{\mathbf q}}
\left(
\left\|J\dot{\mathbf q}-\dot{\mathbf p}\right\|^2
+
\lambda^2\left\|\dot{\mathbf q}\right\|^2
\right).
\]

The resulting damped pseudoinverse is

\[
\boxed{
J^+_\lambda=
\left(J^T J+\lambda^2 I\right)^{-1}J^T
}
\]

and the joint velocity command is

\[
\boxed{
\dot{\mathbf q}=J^+_\lambda\dot{\mathbf p}.
}
\]

Here, \(I\) is the \(3\times3\) identity matrix and \(\lambda\) is the damping factor. Increasing damping reduces the effect of poorly conditioned directions, at the cost of some Cartesian velocity accuracy.

#### Adaptive Damping Used by the Planner

The code calculates a normalised singularity measure:

\[
\mu=
\frac{|\det(J)|}
{\|J_{:,1}\|\,\|J_{:,2}\|\,\|J_{:,3}\|}
\]

where \(J_{:,i}\) denotes column \(i\) of the Jacobian. A small value indicates that the Jacobian is approaching a singular configuration.

The planner selects damping using

\[
\boxed{
\lambda=0.1+
5\max\left(0,1-\frac{\mu}{0.1}\right).
}
\]

This gives greater damping as the singularity measure approaches zero.

#### Exact Analytical Inverse Versus the DLS Planner Method

The source also contains `analytical_inverse_jacobian(q)` and `analytical_inverse_joint_velocity(...)`, which implement an exact analytical inverse for the project's alternative kinematic-angle convention. This inverse is useful for mathematical verification, but its denominators approach zero at singular configurations.

The practical Cartesian trajectory planners use `inverse_jacobian(q, damping=...)`, which computes the DLS expression above. These are two distinct implementations and should not be confused.

### 3.5 How Everything Links Together (System Pipeline)

The runtime planning and execution flow is:

```text
[Line / Square / Circle Target]
              |
              v
     [Cartesian Path Planner]
       50 Hz + Quintic Scaling
              |
       +------+------+
       |             |
       v             v
 [TCP Position] [TCP Velocity]
       |             |
       v             v
      [IK]    [Singularity Measure]
       |             |
       v             v
 [Joint Position] [Adaptive DLS]
       |             |
       +------+------+
              |
              v
     [Trajectory Validation]
              |
              v
      [ROS JointTrajectory]
              |
              v
 [FollowJointTrajectory Action]
              |
              v
    [ros2_control + Gazebo]
              |
              v
 [Joint-State / Controller Feedback]
              |
              v
 [FK-Based TCP Tracking Error]
              |
              v
 [Optional Trail Visualization]
```

The publisher checks joint-position limits, joint-velocity limits, estimated acceleration limits, timestamps and the initial joint state before execution. The `safety.py` module also provides time-retiming utilities, but the main publisher does not automatically call `make_trajectory_safe()` when validation fails.

TCP tracking error is measured using controller feedback and forward kinematics. This is monitoring, not Cartesian closed-loop correction. The \(4.0\text{ rad/s}^2\) acceleration limit is a software planning constraint, not a hardware-rated limit.
In `cartesian_planner.py` and `shape_planner.py`, each Cartesian sample provides:

- A desired TCP position.
- A desired TCP linear velocity.
- An inverse-kinematics solution for the joint positions.
- A singularity measure and corresponding damping value.
- A DLS calculation that converts the Cartesian velocity into three joint velocities.

The resulting joint positions and velocities are passed to the trajectory-building stage. Joint limits and trajectory constraints are checked separately; DLS is not itself a substitute for safety validation.


### 3.6 Implementation in Code: From Equations to `kinematics.py`

Sections 3.1 to 3.4 derive the mathematics. This section shows where each equation lives in the source. Everything below is in [`scripts/motion_planning/kinematics.py`](scripts/motion_planning/kinematics.py), written in plain Python (`math` module only, no NumPy). Lengths are in **millimetres** and angles in **radians**. The code snippets are copied from the repository.

#### 3.6.1 Constants (Section 2 geometry as code)

```python
SHOULDER = (0.0, 0.0, 52.0)
ELBOW = (30.0, 0.0, 288.8)
TIP = (378.0, 0.0, 293.0)

L1 = math.hypot(30.0, 236.8)      # shoulder pivot -> elbow pivot
L2 = math.hypot(348.0, 4.2)       # elbow pivot -> tool tip

PHI1_0 = math.atan2(236.8, 30.0)  # zero-reference angle of link 1
PHI2_0 = math.atan2(4.2, 348.0)   # zero-reference angle of link 2

JOINT_LIMITS = ((-3.14, 3.14), (-1.57, 1.57), (-1.57, 1.57))
```

| Math symbol | Code name | Value |
|---|---|---|
| $L_1$ | `L1` | 238.693 mm |
| $L_2$ | `L2` | 348.025 mm |
| $\Phi_{1,0}$ | `PHI1_0` | 1.44478 rad |
| $\Phi_{2,0}$ | `PHI2_0` | 0.012068 rad |
| $Z_{\text{shoulder}}$ | `SHOULDER[2]` | 52.0 mm |

`math.hypot` computes $\sqrt{a^2+b^2}$, which is exactly how $L_1$ and $L_2$ are defined in Section 2.1.

#### 3.6.2 Forward kinematics: `forward_kinematics(q)`

```python
def forward_kinematics(q):
    """Return end-effector (x, y, z) in millimetres."""
    q1, q2, q3 = q

    phi1 = PHI1_0 - q2
    phi2 = PHI2_0 - q2 - q3

    r = L1 * math.cos(phi1) + L2 * math.cos(phi2)

    x = r * math.cos(q1)
    y = r * math.sin(q1)
    z = SHOULDER[2] + L1 * math.sin(phi1) + L2 * math.sin(phi2)

    return x, y, z
```

| Equation (Section 3.1) | Line of code |
|---|---|
| $\phi_1 = \Phi_{1,0} - q_2$ | `phi1 = PHI1_0 - q2` |
| $\phi_2 = \Phi_{2,0} - q_2 - q_3$ | `phi2 = PHI2_0 - q2 - q3` |
| $r = L_1\cos\phi_1 + L_2\cos\phi_2$ | `r = L1*cos(phi1) + L2*cos(phi2)` |
| $x = r\cos q_1,\; y = r\sin q_1$ | `x = r*cos(q1)`, `y = r*sin(q1)` |
| $z = 52 + L_1\sin\phi_1 + L_2\sin\phi_2$ | `z = SHOULDER[2] + L1*sin(phi1) + L2*sin(phi2)` |

Running it at the zero pose returns `(378.0, 0.0, 293.0)`, the CAD tool-tip position.

#### 3.6.3 Inverse kinematics: `inverse_kinematics(x, y, z, elbow="up", enforce_limits=False)`

```python
r = math.hypot(x, y)
q1 = math.atan2(y, x)
zt = z - SHOULDER[2]

D = (r**2 + zt**2 - L1**2 - L2**2) / (2.0 * L1 * L2)

tolerance = 1e-9
if D < -1.0 - tolerance or D > 1.0 + tolerance:
    raise ValueError("Target is outside the robot workspace")
D = max(-1.0, min(1.0, D))

if elbow == "up":
    delta = -math.acos(D)
else:
    delta = math.acos(D)

phi1 = math.atan2(zt, r) - math.atan2(
    L2 * math.sin(delta),
    L1 + L2 * math.cos(delta)
)
phi2 = phi1 + delta

q2 = PHI1_0 - phi1
q3 = (PHI2_0 - phi2) - q2
```

| Equation (Section 3.2) | Code |
|---|---|
| $q_1 = \operatorname{atan2}(y, x)$ | `q1 = math.atan2(y, x)` |
| $r = \sqrt{x^2+y^2}$ | `r = math.hypot(x, y)` |
| $z_t = z - 52$ | `zt = z - SHOULDER[2]` |
| $D = \dfrac{r^2 + z_t^2 - L_1^2 - L_2^2}{2L_1L_2}$ | `D = (...) / (2.0 * L1 * L2)` |
| Reachability: $-1 \le D \le 1$ | `raise ValueError("Target is outside the robot workspace")` |
| Elbow-up: $\delta = -\arccos D$ | `delta = -math.acos(D)` |
| Elbow-down: $\delta = +\arccos D$ | `delta = math.acos(D)` |
| $\phi_1 = \operatorname{atan2}(z_t, r) - \operatorname{atan2}(L_2\sin\delta,\, L_1 + L_2\cos\delta)$ | `phi1 = atan2(zt, r) - atan2(...)` |
| $\phi_2 = \phi_1 + \delta$ | `phi2 = phi1 + delta` |
| $q_2 = \Phi_{1,0} - \phi_1$ | `q2 = PHI1_0 - phi1` |
| $q_3 = (\Phi_{2,0} - \phi_2) - q_2$ | `q3 = (PHI2_0 - phi2) - q2` |

Implementation details worth knowing:

- **Numerical safety:** `D` may land at `1.0000000001` because of floating-point rounding for a target exactly at full stretch. The code accepts values within `1e-9` of the limits and then clamps `D` into $[-1, 1]$ so `math.acos` never fails.
- **Workspace check:** a target that is too far away or too close raises `ValueError`. The planners treat this as a failed plan.
- **Joint limits:** with `enforce_limits=True`, the solution is passed through `check_joint_limits()` and a `ValueError` is raised if any angle is outside `JOINT_LIMITS`. The shape planner always calls IK with `enforce_limits=True`.
- **Verified round trip:** for `q = (0.3, 0.2, -0.4)`, forward kinematics gives `(398.065, 123.136, 351.373)`. Running IK on that point with `elbow="up"` returns `(0.3, 0.2, -0.4)`. With `elbow="down"` it returns `(0.3, 1.4435, -2.4654)`, which reaches the same point but violates the $\pm 1.57$ rad elbow limit. This is why the planners use `elbow="up"` by default.

#### 3.6.4 Jacobian: `jacobian(q)`

```python
def jacobian(q):
    q1, q2, q3 = q

    phi1 = PHI1_0 - q2
    phi2 = PHI2_0 - q2 - q3

    r = L1 * math.cos(phi1) + L2 * math.cos(phi2)

    dr2 = L1 * math.sin(phi1) + L2 * math.sin(phi2)   # d_r2 = dr/dq2
    dr3 = L2 * math.sin(phi2)                         # d_r3 = dr/dq3

    return [
        [-r * math.sin(q1), dr2 * math.cos(q1), dr3 * math.cos(q1)],
        [ r * math.cos(q1), dr2 * math.sin(q1), dr3 * math.sin(q1)],
        [ 0.0,              -r,                 -L2 * math.cos(phi2)],
    ]
```

This is a line-by-line transcription of the boxed matrix in Section 3.3: `dr2` is $d_{r2}$, `dr3` is $d_{r3}$, and the returned nested list is the $3\times3$ matrix $J(\mathbf q)$ (row $i$ = $x$, $y$, $z$; column $j$ = $q_1$, $q_2$, $q_3$). At the zero pose it evaluates to:

```text
[[   0.0,  241.0,    4.2],
 [ 378.0,    0.0,    0.0],
 [   0.0, -378.0, -348.0]]
```

The first column says that rotating the base at 1 rad/s moves the tip at 378 mm/s in $+Y$, which matches the radial reach $r = 378$ mm.

#### 3.6.5 Singularity measure and adaptive damping

```python
def singularity_measure(q):
    j = jacobian(q)
    columns = [[j[row][col] for row in range(3)] for col in range(3)]
    norms = [math.sqrt(sum(v * v for v in column)) for column in columns]

    if any(norm == 0.0 for norm in norms):
        return 0.0

    determinant = (
        j[0][0] * (j[1][1]*j[2][2] - j[1][2]*j[2][1])
        - j[0][1] * (j[1][0]*j[2][2] - j[1][2]*j[2][0])
        + j[0][2] * (j[1][0]*j[2][1] - j[1][1]*j[2][0])
    )
    return abs(determinant) / (norms[0] * norms[1] * norms[2])
```

This implements

$$\mu = \frac{|\det J|}{\|J_{:,1}\|\,\|J_{:,2}\|\,\|J_{:,3}\|}$$

The determinant is written out by cofactor expansion along the first row, so no linear-algebra library is needed. The result is between 0 (singular) and 1 (perfectly orthogonal columns). At the zero pose, $\mu \approx 0.527$.

The damping value is computed by the planners, not by `kinematics.py`. In `shape_planner.py`:

```python
measure = singularity_measure(q)
damping = 0.1 + 5.0 * max(0.0, 1.0 - measure / 0.1)
```

which is the boxed equation $\lambda = 0.1 + 5\max(0, 1 - \mu/0.1)$ from Section 3.4:

| Singularity measure $\mu$ | Damping $\lambda$ |
|---|---|
| $\mu \ge 0.1$ (healthy pose) | 0.1 (the minimum) |
| $\mu = 0.05$ | 2.6 |
| $\mu \to 0$ (singular) | 5.1 (the maximum) |

`cartesian_planner.py` uses the same formula for straight lines. The separate `check_singularity(q)` function returns `True` when either $\mu \le 10^{-6}$ or the analytical conditions from Section 3.6.9 report a singular pose.

#### 3.6.6 Damped least-squares inverse: `inverse_jacobian(q, damping)`

```python
def inverse_jacobian(q, damping=1.0):
    if damping <= 0.0:
        raise ValueError("Damping must be positive")

    j = jacobian(q)
    jt = _transpose(j)

    normal = _matrix_multiply(jt, j)          # J^T J

    for i in range(3):
        normal[i][i] += damping ** 2          # + lambda^2 I

    return _matrix_multiply(_inverse_3x3(normal), jt)   # (J^T J + lambda^2 I)^-1 J^T
```

| Equation (Section 3.4) | Code |
|---|---|
| $J^T$ | `jt = _transpose(j)` |
| $J^TJ$ | `normal = _matrix_multiply(jt, j)` |
| $J^TJ + \lambda^2 I$ | `normal[i][i] += damping ** 2` |
| $(J^TJ + \lambda^2 I)^{-1}J^T$ | `_matrix_multiply(_inverse_3x3(normal), jt)` |

Three small helper functions replace NumPy:

- `_transpose(matrix)` swaps rows and columns.
- `_matrix_multiply(a, b)` is a triple-loop matrix product.
- `_inverse_3x3(matrix)` is Gauss-Jordan elimination with partial pivoting on a $3\times6$ augmented matrix. It raises `ValueError("Matrix is singular")` if a pivot falls below `1e-15`.

Adding $\lambda^2$ to the diagonal makes $J^TJ + \lambda^2I$ positive definite for any $\lambda > 0$, so the inversion stays well-conditioned even when $J$ itself is singular. That is why `damping <= 0` is rejected.

**Worked example.** At the zero pose with $\lambda = 0.1$, commanding a tip velocity of $(0, 10, 0)$ mm/s gives joint velocities of about $(0.02646, 0, 0)$ rad/s. Only the base rotates, which is correct because the first Jacobian column is $(0, 378, 0)$ and $10/378 = 0.02646$.

#### 3.6.7 How the planners call these functions

The core loop is `_plan_path()` in [`shape_planner.py`](scripts/motion_planning/shape_planner.py). For every 50 Hz sample it runs this chain:

```python
# 1. Quintic progress along the path (Step 7)
blend      = 10*u**3 - 15*u**4 + 6*u**5
blend_rate = (30*u**2 - 60*u**3 + 30*u**4) / duration

distance        = total_length * blend
cartesian_speed = total_length * blend_rate

# 2. Position and unit tangent on the current segment (line or arc)
position, tangent = evaluate(remaining)
velocity = tuple(v * cartesian_speed for v in tangent)      # TCP velocity (mm/s)

# 3. Inverse kinematics -> joint angles
q = inverse_kinematics(*position, elbow=elbow, enforce_limits=True)

# 4. Singularity measure -> adaptive damping
measure = singularity_measure(q)
damping = 0.1 + 5.0 * max(0.0, 1.0 - measure / 0.1)

# 5. Damped inverse Jacobian -> joint velocities (rad/s)
inverse = inverse_jacobian(q, damping=damping)
qdot = tuple(sum(inverse[row][col] * velocity[col] for col in range(3))
             for row in range(3))
```

Each sample is stored as `(t, position, q, qdot, damping, measure)`. Position and velocity therefore come from two different routes: joint **positions** come from exact analytical IK, while joint **velocities** come from the damped Jacobian. The line planners in `cartesian_planner.py` follow the same IK, damping and DLS sequence.

#### 3.6.8 Joint mapping and TCP tracking error

The math functions use $q = [q_1, q_2, q_3]$, while the ROS 2 controller uses named joints. [`joint_mapping.py`](scripts/motion_planning/joint_mapping.py) converts between them with per-joint signs and offsets (currently all `+1` and `0`, so the two conventions match):

```python
def ros_to_math(q_ros):
    return tuple((position - offset) / sign
                 for position, sign, offset in zip(q_ros, JOINT_SIGNS, JOINT_OFFSETS))
```

Forward kinematics is then reused to measure tracking error in `feedback_callback()` of [`trajectory_publisher.py`](scripts/motion_planning/trajectory_publisher.py):

```python
desired_tcp = forward_kinematics(ros_to_math(desired))
actual_tcp  = forward_kinematics(ros_to_math(actual))

error = math.sqrt(sum((a - d) ** 2 for a, d in zip(actual_tcp, desired_tcp)))
self.tracking_errors.append(error)
```

The error is the Euclidean distance between the TCP computed from the **desired** joint positions and the TCP computed from the **actual** joint positions reported by the controller. `report_tracking_accuracy()` then computes the RMS, maximum and final error:

$$e_{\text{RMS}} = \sqrt{\frac{1}{N}\sum_{k=1}^{N} e_k^2}$$

These are the numbers reported in Section 5.5. They measure tracking in simulation and are not used to correct the motion.

#### 3.6.9 The analytical alternative (`analytical_*` functions)

The file also contains an exact analytical derivation that uses a different angle convention:

$$\theta_1 = q_1,\qquad \theta_2 = \Phi_{1,0} - q_2 - \tfrac{\pi}{2},\qquad \theta_3 = \Phi_{2,0} - \Phi_{1,0} - q_3$$

| Function | Purpose |
|---|---|
| `analytical_joint_angles(q)` | Converts $q$ to $\theta$ using the equations above |
| `analytical_forward_kinematics_transform(q)` | Returns the exact $4\times4$ homogeneous transform |
| `analytical_full_jacobian(q)` | Returns the $6\times3$ geometric Jacobian (linear and angular rows) |
| `analytical_linear_jacobian(q)` | Returns the top $3\times3$ linear part |
| `analytical_jacobian_determinant(q)` | Returns $\det J_v = A\,L_2L_3\sin\theta_3$ |
| `analytical_singularity_conditions(q)` | Reports whether $A = 0$ (arm over the base axis) or $\sin\theta_3 = 0$ (elbow fully stretched or folded) |
| `analytical_inverse_jacobian(q)` | Exact inverse of $J_v$ (raises `ValueError` at a singularity) |
| `analytical_inverse_joint_velocity(q, v)` | Applies the exact inverse and converts back with $\dot q_2 = -\dot\theta_2,\ \dot q_3 = -\dot\theta_3$ |

Here $A = L_2\sin\theta_2 + L_3\sin(\theta_2+\theta_3)$ and $B = L_2\cos\theta_2 + L_3\cos(\theta_2+\theta_3)$, with `ANALYTICAL_L2 = L1` and `ANALYTICAL_L3 = L2`. The two singularity conditions are physically meaningful: the tip is on the base axis, or the arm is fully extended or folded.

The exact inverse has $\sin\theta_3$ and $A$ in its denominators, so it fails at singular poses by design. The trajectory planners therefore use the damped `inverse_jacobian()` from Section 3.6.6, and the analytical functions serve as an independent mathematical check.

#### 3.6.10 Function reference

| Function | Input | Output | Math section | Used by |
|---|---|---|---|---|
| `forward_kinematics(q)` | $q$ (rad) | $(x,y,z)$ (mm) | 3.1 | `trajectory_publisher.py` (start pose, tracking error) |
| `inverse_kinematics(x,y,z,elbow,enforce_limits)` | $(x,y,z)$ | $q$ (rad) | 3.2 | `cartesian_planner.py`, `shape_planner.py` |
| `jacobian(q)` | $q$ | $3\times3$ matrix | 3.3 | `singularity_measure`, `inverse_jacobian` |
| `singularity_measure(q)` | $q$ | $\mu \in [0,1]$ | 3.4 | `cartesian_planner.py`, `shape_planner.py` |
| `check_singularity(q)` | $q$ | `True`/`False` | 3.4, 3.6.9 | Standalone check (not called by the planners) |
| `inverse_jacobian(q, damping)` | $q$, $\lambda$ | $3\times3$ DLS inverse | 3.4 | `cartesian_planner.py`, `shape_planner.py` |
| `check_joint_limits(q)` | $q$ | `True`/`False` | 2.2 | `inverse_kinematics`, `shape_planner.py`, `trajectory_publisher.py` |

---

## 4. Cartesian Shape Tracing: Square and Circle

### 4.1 Circle Path Generation

The standalone `plan_cartesian_circle()` function in `shape_planner.py` generates a complete circular path in the XY plane.

For a starting point \((x_s,y_s,z_s)\) and radius \(R\), the centre is:

\[
c_x=x_s,\qquad c_y=y_s-R
\]

The Cartesian path is:

\[
x(\theta)=c_x+R\cos\theta
\]

\[
y(\theta)=c_y+R\sin\theta
\]

\[
z(\theta)=z_s
\]

where:

\[
\frac{\pi}{2}\leq\theta\leq\frac{5\pi}{2}
\]

The tangent direction is:

\[
\mathbf{t}(\theta)=
\begin{bmatrix}
-\sin\theta\\
\cos\theta\\
0
\end{bmatrix}
\]

Quintic time scaling controls the progress along the path, starting and finishing at zero speed.

**Actual execution:** `trajectory_publisher.py` constructs the circle using `_arc_segment()` and `_plan_path()`, with the supplied centre \((c_x,c_y)\) and angular range \(0\) to \(2\pi\). Therefore, execution starts at the rightmost point of the circle. The demonstration has five stages: approach, move to the tracing start, trace the circle, retract and return.

### 4.2 Rounded Square Path Generation

A square with sharp corners requires an instantaneous change in velocity direction if the TCP moves continuously at nonzero speed. This creates demanding acceleration requirements. To avoid sharp directional changes, the planner combines four straight edges with four quarter-circle corner blends.

The `plan_cartesian_square()` function in `shape_planner.py` accepts the starting point \(P_0=(x,y,z)\), square size \(S\), and blend radius \(r\), subject to:

\[
S>0,\qquad 0<r<\frac{S}{2}
\]

The eight path segments are:

1. Bottom edge: \((x,y,z)\) to \((x+S-2r,y,z)\).
2. Bottom-right arc: centre \((x+S-2r,y+r)\), angle \(-\pi/2\) to \(0\).
3. Right edge: \((x+S-r,y+r,z)\) to \((x+S-r,y+S-r,z)\).
4. Top-right arc: centre \((x+S-2r,y+S-r)\), angle \(0\) to \(\pi/2\).
5. Top edge: \((x+S-2r,y+S,z)\) to \((x,y+S,z)\).
6. Top-left arc: centre \((x,y+S-r)\), angle \(\pi/2\) to \(\pi\).
7. Left edge: \((x-r,y+S-r,z)\) to \((x-r,y+r,z)\).
8. Bottom-left arc: centre \((x,y+r)\), angle \(\pi\) to \(3\pi/2\), closing the path.

The resulting coordinate bounds are:

\[
\boxed{x-r\leq X\leq x+S-r}
\]

\[
\boxed{y\leq Y\leq y+S}
\]

The line and arc segments have matching tangent directions at their joins, producing a geometrically tangent-continuous path. This does not imply continuous curvature or acceleration at every join.

The `_plan_path()` function uses quintic path progress:

\[
s(u)=10u^3-15u^4+6u^5,\qquad u=\frac{t}{T}
\]

At each sample, it calculates the Cartesian position and tangent velocity, solves inverse kinematics, evaluates the singularity measure, applies adaptive DLS, and checks joint limits. The complete path starts and ends at zero speed without stopping at every rounded corner.
## 5. Testing, Dry-Runs, and Verification

The following tests document software-level verification, trajectory dry runs, and Gazebo simulation execution. Physical-hardware validation has not yet been performed.

### 5.1 Kinematics Consistency Test (1000 Configurations)
* **Objective**: Confirm that inverse kinematics reconstructs the Cartesian positions produced by forward kinematics for sampled joint configurations.* **Method**: Evaluated 1,000 random configurations covering all joints and tested both elbow configurations (2,000 IK solutions).
* **Result**:
  $$\text{Worst-case Cartesian error} \le 3.75 \times 10^{-13}\text{ mm}$$
  **Status**: **PASS**

### 5.2 Jacobian Gradient Test
* **Objective**: Ensure the analytical Jacobian partial derivatives match numerical gradients.
* **Method**: Compared analytical $J(q)$ against finite-difference evaluations with step size $\epsilon = 10^{-6}$.
* **Result**: Finite-difference residual approximately $4.25 \times 10^{-8}$.  **Status**: **PASS**

### 5.3 Square Trajectory Dry-Run Test
* **Configuration**: Side length = $20\text{ mm}$, Blend radius = $4\text{ mm}$, Duration = $8.0\text{ s}$, Frequency = $50\text{ Hz}$.
* **Results**:
  - Waypoints generated: 401 points.
  - Path closure error: $0.0000\text{ mm}$ (perfect loop).
  - Maximum joint velocity: $0.0816\text{ rad/s}$ (well within limit of $2.0\text{ rad/s}$).
  - Maximum estimated acceleration: $0.3400\text{ rad/s}^2$ (well within limit of $4.0\text{ rad/s}^2$).
  - Safety checks: **All passed**. No command sent (dry-run mode).
  **Status**: **PASS**

### 5.4 Circle Trajectory Dry-Run Test
* **Configuration**: Radius = $10\text{ mm}$, Duration = $8.0\text{ s}$, Frequency = $50\text{ Hz}$.
* **Results**:
  - Waypoints generated: 401 points.
  - Path closure error: $0.0000\text{ mm}$ (perfect loop).
  - Maximum joint velocity: $0.0675\text{ rad/s}$ (limit $2.0\text{ rad/s}$).
  - Maximum estimated acceleration: $0.0831\text{ rad/s}^2$ (limit $4.0\text{ rad/s}^2$).
  - Safety checks: **All passed**. No command sent (dry-run mode).
  **Status**: **PASS**

### 5.5 Gazebo Simulation Execution & Tracking-Error Measurement
* **Configuration**: Cartesian straight-line movement of $\Delta Y = 20\text{ mm}$ and return, executed through ROS 2 `FollowJointTrajectory`.
* **Execution Metrics**:
  - Action Goal Accepted: Yes
  - Action Result Status: `SUCCESS` (`error_code = 0`)
  - Feedback samples collected: 101 samples.

| Accuracy Metric | Measured Error |
|---|---:|
| **RMS Tracking Error** | $\mathbf{0.1207\text{ mm}}$ |
| **Maximum Peak Tracking Error** | $\mathbf{0.1878\text{ mm}}$ |
| **Final Settled Position Error** | $\mathbf{0.0115\text{ mm}}$ |

**Status**: **PASS** (Sub-millimeter tracking error measured in Gazebo physics simulation; no real-hardware test is claimed).

---

## 6. Repository Architecture & File Map (Tabular Guide)

This section explains the important project folders and source files, their responsibilities, and how they connect to the motion-planning examples. Each file or folder name links to its location in this repository.

### 6.1 Robot Model, Simulation and Configuration

| File / Folder | What it does | Related files / examples |
|---|---|---|
| [`urdf/robot_arm.urdf`](urdf/robot_arm.urdf) | Describes the robot's links, joints, visual geometry and kinematic structure. Used as the visualization-oriented robot description. | [`meshes/`](meshes/), [`make_urdf.py`](scripts/setup/make_urdf.py) |
| [`urdf/robot_arm_gz.urdf`](urdf/robot_arm_gz.urdf) | Gazebo-oriented robot description containing the simulation-related inertial, collision and control configuration. | [`gazebo.launch.py`](launch/gazebo.launch.py), [`controllers.yaml`](config/controllers.yaml) |
| [`urdf/robot_arm.urdf.bak`](urdf/robot_arm.urdf.bak) | Backup copy of the visual URDF. It is not the primary URDF referenced by the normal run instructions. | [`robot_arm.urdf`](urdf/robot_arm.urdf) |
| [`meshes/`](meshes/) | Contains the STL geometry assets used to visually represent the robot's base, legs, motors, shoulder, elbow, upper arm, forearm and related mechanical parts. | Both URDF files |
| [`config/controllers.yaml`](config/controllers.yaml) | Defines the ROS 2 control configuration for the joint-state broadcaster and arm trajectory controller. | [`gazebo.launch.py`](launch/gazebo.launch.py), [`start_gazebo.sh`](run/start_gazebo.sh) |
| [`launch/gazebo.launch.py`](launch/gazebo.launch.py) | Launches and coordinates the Gazebo robot simulation, robot state publishing, entity spawning and controller startup. | [`robot_arm_gz.urdf`](urdf/robot_arm_gz.urdf), [`controllers.yaml`](config/controllers.yaml) |

### 6.2 Core Motion-Planning Modules

These modules form the main planning and execution pipeline.

| File | What it does | Related example / connection |
|---|---|---|
| [`kinematics.py`](scripts/motion_planning/kinematics.py) | Implements analytical forward kinematics (joint angles to TCP position), inverse kinematics (TCP position to joint angles), the Jacobian, inverse Jacobian and singularity calculations. | Used by the Cartesian and shape planners. |
| [`joint_mapping.py`](scripts/motion_planning/joint_mapping.py) | Converts between the mathematical joint-vector convention and the ROS 2 controller joint names and ordering. | Connects planning output to ROS trajectory messages. |
| [`trajectories.py`](scripts/motion_planning/trajectories.py) | Generates smooth quintic trajectory interpolation and provides trajectory time-scaling functionality. | Used when generating smooth motion profiles. |
| [`cartesian_planner.py`](scripts/motion_planning/cartesian_planner.py) | Generates Cartesian straight-line paths and maps Cartesian motion into joint-space motion using the robot kinematics. | Straight-line trajectory example through [`trajectory_publisher.py`](scripts/motion_planning/trajectory_publisher.py). |
| [`shape_planner.py`](scripts/motion_planning/shape_planner.py) | Generates Cartesian square and circle paths, including rounded-square path geometry. | Used by [`square_demo.py`](scripts/motion_planning/square_demo.py) and [`circle_demo.py`](scripts/motion_planning/circle_demo.py). |
| [`safety.py`](scripts/motion_planning/safety.py) | Provides standalone utilities for trajectory safety checks and dynamic time scaling. The main trajectory publisher performs its own sample validation and does not automatically call `make_trajectory_safe()`. | Available for trajectory-planning workflows. |
| [`ros_trajectory.py`](scripts/motion_planning/ros_trajectory.py) | Converts planned joint waypoints and timing information into ROS 2 `JointTrajectory` messages. | Connects planning data to the ROS 2 controller interface. |
| [`trajectory_publisher.py`](scripts/motion_planning/trajectory_publisher.py) | Provides the ROS 2 trajectory-publishing node. It supports dry-run validation and explicitly requested execution, and can sample `/joint_states` to measure TCP tracking error using forward kinematics. | Used for straight-line, square and circle trajectory-publisher workflows. |

### 6.3 Motion-Planning Demonstrations

| File | What it does | Related example |
|---|---|---|
| [`square_demo.py`](scripts/motion_planning/square_demo.py) | Runs the square demonstration using the motion-planning and Gazebo visualization components. | [`run_square.sh`](run/run_square.sh) |
| [`circle_demo.py`](scripts/motion_planning/circle_demo.py) | Runs the circle demonstration using the motion-planning and Gazebo visualization components. | [`run_circle.sh`](run/run_circle.sh) |

### 6.4 Gazebo Visualization and Marker Utilities

These scripts create visual guides or traces in Gazebo. A guide marker is a visual reference, not a commanded robot trajectory.

| File / Folder | What it does | Related example |
|---|---|---|
| [`scripts/square/square_guide.py`](scripts/square/square_guide.py) | Spawns small black disc markers on the Gazebo ground to show the outline of a square. | Used by [`run_square.sh`](run/run_square.sh). |
| [`scripts/circle/circle_guide.py`](scripts/circle/circle_guide.py) | Spawns small black disc markers on the Gazebo ground to show a circular outline. | Used by [`run_circle.sh`](run/run_circle.sh). |
| [`scripts/circle/legacy/circle_trace.py`](scripts/circle/legacy/circle_trace.py) | Contains an earlier, legacy circle-tracing implementation. It is separate from the newer modular shape-planning pipeline. | Historical circle-tracing implementation. |
| [`scripts/circle/legacy/tip_trail.py`](scripts/circle/legacy/tip_trail.py) | Legacy visualization node that monitors the end-effector transform and spawns trail markers when the tip is near the configured drawing height. | Historical Gazebo tip-trail visualization. |
| [`scripts/common/clear_marks.py`](scripts/common/clear_marks.py) | Finds project guide and trail marker models in Gazebo and requests their deletion. | Called by the square and circle runner scripts before creating new guides. |

### 6.5 Setup and CAD Extraction Scripts

| File / Folder | What it does | Related files |
|---|---|---|
| [`scripts/setup/freecad/shoulder_axes.py`](scripts/setup/freecad/shoulder_axes.py) | FreeCAD inspection script for extracting shoulder joint-axis geometry from the CAD model. | Robot CAD and URDF geometry |
| [`scripts/setup/freecad/elbow_wrist_axes.py`](scripts/setup/freecad/elbow_wrist_axes.py) | FreeCAD inspection script for extracting elbow and wrist axis geometry. | Robot CAD and URDF geometry |
| [`scripts/setup/freecad/wrist_axes.py`](scripts/setup/freecad/wrist_axes.py) | FreeCAD inspection script for extracting wrist-axis geometry. | Robot CAD and URDF geometry |
| [`scripts/setup/make_urdf.py`](scripts/setup/make_urdf.py) | Generates the visualization-oriented `robot_arm.urdf` from the defined mesh and joint geometry. The current script contains a machine-specific workspace path. | [`robot_arm.urdf`](urdf/robot_arm.urdf), [`meshes/`](meshes/) |
| [`scripts/setup/make_gz_urdf.py`](scripts/setup/make_gz_urdf.py) | Generates the Gazebo-oriented URDF, including visual geometry, collision geometry and inertial properties. The current script contains a machine-specific workspace path. | [`robot_arm_gz.urdf`](urdf/robot_arm_gz.urdf) |
| [`scripts/setup/move_joints.py`](scripts/setup/move_joints.py) | Small test utility that publishes `JointState` messages using supplied joint positions. It is not the main trajectory-controller execution node. | ROS 2 joint-state testing |

### 6.6 Runner Scripts and Package Files

| File | What it does | Related example |
|---|---|---|
| [`run/start_gazebo.sh`](run/start_gazebo.sh) | Sources the Gazebo and ROS 2 environments and launches the project's Gazebo launch file. It also terminates selected existing processes before launching. | Starts the simulation used by the demonstrations. |
| [`run/run_square.sh`](run/run_square.sh) | Runs the square demo. In its normal path it clears old markers, creates the square guide and starts the square demo; it also has a `--dry` path. | [`square_demo.py`](scripts/motion_planning/square_demo.py) |
| [`run/run_circle.sh`](run/run_circle.sh) | Runs the circle demo. In its normal path it clears old markers, creates the circle guide and starts the circle demo; it also has a `--dry` path. | [`circle_demo.py`](scripts/motion_planning/circle_demo.py) |
| [`CMakeLists.txt`](CMakeLists.txt) | Defines the `ament_cmake` package build and installation rules for the robot description assets and motion-planning Python files. | ROS 2 package build |
| [`package.xml`](package.xml) | Declares the ROS 2 package metadata and its build, runtime and test dependencies. | ROS 2 package setup |
| [`.gitignore`](.gitignore) | Lists files and generated artifacts Git should ignore. | Repository maintenance |

### 6.7 How the Main Motion-Planning Files Work Together

The following relationships show how the source files connect in the project's examples:

`cartesian_planner.py` → `kinematics.py` → `joint_mapping.py` → `ros_trajectory.py` → `trajectory_publisher.py`
`cartesian_planner.py` → `kinematics.py` → `joint_mapping.py` → `safety.py` → `trajectories.py` (where applicable) → `ros_trajectory.py` → `trajectory_publisher.py`

**Square demonstration**

`run_square.sh` → `clear_marks.py` → `square_guide.py` → `square_demo.py` → `shape_planner.py` → core kinematics and safety modules

**Circle demonstration**

`run_circle.sh` → `clear_marks.py` → `circle_guide.py` → `circle_demo.py` → `shape_planner.py` → core kinematics and safety modules

The guide scripts draw visual outlines in Gazebo. The planner generates the robot path, while the ROS trajectory components handle conversion and controller communication. Tracking-error measurement uses joint-state feedback and forward kinematics; it does not imply closed-loop Cartesian correction.

## 7. How to Build and Run the Project

### 7.1 Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill (`ros-humble-desktop`)
- Gazebo Classic 11 (`gazebo`, `ros-humble-gazebo-ros-pkgs`)
- ROS 2 Control (`ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-gazebo-ros2-control`)

### 7.2 Build the Workspace
```bash
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 7.3 Launch Gazebo Simulation
In a dedicated terminal:
```bash
cd ~/robot_arm_ws/src/robot_arm_description
./run/start_gazebo.sh
```
*Wait for Gazebo to launch and for both `joint_state_broadcaster` and `arm_controller` to reach active state.*

### 7.4 Run Dry-Run Validation (Safe Mode — No Robot Movement)
Run trajectory safety checks and verify timing without commanding the motors:
```bash
cd ~/robot_arm_ws

# Dry-run for a 20 mm square trajectory:
python3 src/robot_arm_description/scripts/motion_planning/trajectory_publisher.py --shape square --size 20 --blend-radius 4 --dry
# Dry-run for a 10 mm radius circle trajectory:
python3 src/robot_arm_description/scripts/motion_planning/trajectory_publisher.py --shape circle --radius 10 --dry
```

### 7.5 Execute Live Trajectories in Gazebo
To command actual motion on the simulated robot:
```bash
cd ~/robot_arm_ws

# 1. Execute smooth Cartesian straight line (+20 mm along Y-axis):
python3 src/robot_arm_description/scripts/motion_planning/trajectory_publisher.py --shape line --dy 20 --execute

# 2. Execute full square tracing with real-time Gazebo markers:
cd ~/robot_arm_ws/src/robot_arm_description
./run/run_square.sh

# 3. Execute full circle tracing with real-time Gazebo markers:
cd ~/robot_arm_ws/src/robot_arm_description
./run/run_circle.sh
```

### 7.6 Clear Gazebo Markers
To clean up trail dots from previous runs:
```bash
cd ~/robot_arm_ws
python3 src/robot_arm_description/scripts/common/clear_marks.py
```

---

## 8. ROS 2 Controllers and Control Stack Used

This section lists every ROS 2 controller and control component the project uses, what each one does, and how it is configured. The configuration comes from [`config/controllers.yaml`](config/controllers.yaml), [`launch/gazebo.launch.py`](launch/gazebo.launch.py) and [`urdf/robot_arm_gz.urdf`](urdf/robot_arm_gz.urdf). Everything runs in **Gazebo Classic simulation** on ROS 2 Humble; no real hardware driver is used.

### 8.1 Components and What Each One Does

| Component | Type / plugin | Defined in | What it does in this project |
|---|---|---|---|
| **Controller manager** | `controller_manager` (ros2_control) | [`controllers.yaml`](config/controllers.yaml) | Loads and runs the controllers inside the control loop at 100 Hz. Each controller is started with a `spawner` node. |
| **`joint_state_broadcaster`** | `joint_state_broadcaster/JointStateBroadcaster` | [`controllers.yaml`](config/controllers.yaml) | Reads the joint positions and velocities from the hardware interface and publishes them on `/joint_states`. The trajectory publisher uses this topic for the start pose and for TCP tracking-error measurement. |
| **`arm_controller`** | `joint_trajectory_controller/JointTrajectoryController` | [`controllers.yaml`](config/controllers.yaml) | The only controller that moves the arm. It hosts the `FollowJointTrajectory` action server, receives the planned waypoints, interpolates between them in time, and sends position commands to the three joints. |
| **`GazeboSystem`** | `gazebo_ros2_control/GazeboSystem` | `<ros2_control>` block in [`robot_arm_gz.urdf`](urdf/robot_arm_gz.urdf) | The simulated hardware interface. It exposes each joint's command and state interfaces to ros2_control and connects them to the Gazebo physics joints. |
| **Gazebo plugin** | `libgazebo_ros2_control.so` | `<gazebo>` block in [`robot_arm_gz.urdf`](urdf/robot_arm_gz.urdf) | Starts the controller manager inside Gazebo and loads the parameters from `controllers.yaml`. |
| **`robot_state_publisher`** | `robot_state_publisher` | [`gazebo.launch.py`](launch/gazebo.launch.py) | Publishes the URDF on `/robot_description` and the link transforms, with `use_sim_time: True`. |
| **`spawn_entity.py`** | `gazebo_ros` | [`gazebo.launch.py`](launch/gazebo.launch.py) | Spawns the robot into Gazebo (entity name `robot_arm`) from the `robot_description` topic. |
| **Action client** | `FollowJointTrajectory` client | [`trajectory_publisher.py`](scripts/motion_planning/trajectory_publisher.py) | Sends the planned trajectory to `/arm_controller/follow_joint_trajectory` and receives feedback while the arm moves. |

Only two controllers are used: `joint_state_broadcaster` (read-only, publishes state) and `arm_controller` (commands motion). No velocity, effort or Cartesian controller is used; Cartesian motion is produced by the planner (Sections 3 and 4) and executed as joint positions.

### 8.2 Controller Configuration (`config/controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint_base_rotate
      - joint_shoulder
      - joint_elbow
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
```

| Parameter | Value | What it means |
|---|---|---|
| `update_rate` | `100` Hz | How often the controller manager runs the control loop |
| `use_sim_time` | `true` | Uses the Gazebo simulation clock instead of the wall clock |
| `joints` | `joint_base_rotate`, `joint_shoulder`, `joint_elbow` | The three joints controlled by `arm_controller`, in this order (matches [`joint_mapping.py`](scripts/motion_planning/joint_mapping.py)) |
| `command_interfaces` | `position` | The controller sends position targets only (no velocity or effort commands) |
| `state_interfaces` | `position`, `velocity` | The controller reads joint position and velocity as feedback |
| `state_publish_rate` | `50.0` Hz | How often the controller publishes its state |
| `action_monitor_rate` | `20.0` Hz | How often the action server monitors an executing goal |
| `allow_partial_joints_goal` | `false` | A goal must include all three joints; goals with only some joints are rejected |

### 8.3 Hardware Interface in the URDF

The simulated hardware is declared in `robot_arm_gz.urdf`. Each of the three joints exposes one command interface and two state interfaces, matching `controllers.yaml`:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="joint_base_rotate">
    <command_interface name="position"/>
    <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
    <state_interface name="velocity"/>
  </joint>
  <!-- joint_shoulder and joint_elbow are declared identically -->
</ros2_control>

<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>/home/pavan/robot_arm_ws/src/robot_arm_description/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

| Item | Value | Meaning |
|---|---|---|
| Initial joint position | `0.0` rad for all three joints | The arm starts at the zero pose, the TCP at (378, 0, 293) mm |
| URDF joint limits | position $\pm3.14$ / $\pm1.57$ / $\pm1.57$ rad, velocity 2.0 rad/s, effort 10.0 | Hard limits declared per joint (see Section 2.2) |
| Acceleration limit (4.0 rad/s²) | Not in the URDF | A software planning limit checked by the trajectory publisher, not a controller setting |

> **Portability note:** the `<parameters>` path above is an absolute path (`/home/pavan/...`), written by `make_gz_urdf.py`. If the repository is cloned to a different location or user account, update this path (or regenerate the URDF) or the controllers will not load their parameters.

### 8.4 Topics and Actions

| Name | Type | Published / served by | Used by | Purpose |
|---|---|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `arm_controller` (action server) | `trajectory_publisher.py` (action client) | Sends a planned trajectory and returns accept/result status and feedback (desired vs actual joint positions) |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `trajectory_publisher.py` | Start pose, tracking-error measurement and the actual-trail recording |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | `spawn_entity.py` | Supplies the URDF used to spawn the robot |

### 8.5 Startup Order

[`gazebo.launch.py`](launch/gazebo.launch.py) starts the pieces in a fixed order using `OnProcessExit` event handlers, so each controller is spawned only after the step it depends on has finished:

| Step | Action | Starts after |
|---|---|---|
| 1 | Gazebo launches with `worlds/white.world` | Launch start |
| 2 | `robot_state_publisher` starts | Launch start |
| 3 | `spawn_entity.py` spawns `robot_arm` into Gazebo | Launch start |
| 4 | `joint_state_broadcaster` spawner runs | `spawn_entity.py` has exited |
| 5 | `arm_controller` spawner runs | The `joint_state_broadcaster` spawner has exited |

When both controllers are active, the arm is ready to accept trajectories.

### 8.6 Control Data Flow

```mermaid
flowchart LR
    P["Planner<br/>(IK + DLS, 50 Hz)"] --> TP["trajectory_publisher.py<br/>(action client)"]
    TP -->|"FollowJointTrajectory goal"| AC["arm_controller<br/>(JointTrajectoryController)"]
    AC -->|"position commands"| GS["GazeboSystem<br/>(hardware interface)"]
    GS --> J["Gazebo joints<br/>(3 DOF arm)"]
    J -->|"joint position / velocity"| GS
    GS --> JSB["joint_state_broadcaster"]
    JSB -->|"/joint_states"| TP
    AC -->|"action feedback<br/>(desired vs actual)"| TP
    TP --> FK["Forward kinematics<br/>TCP tracking error"]
```

The loop is **open-loop in Cartesian space**: the planner computes the whole trajectory first, `arm_controller` tracks the joint positions, and the feedback is used only to measure TCP tracking error with forward kinematics (Section 3.6.8). It is not used to correct the path.

### 8.7 Checking the Controllers While the Simulation Runs

Run these in a second terminal after `./run/start_gazebo.sh` (source the workspace first):

```bash
ros2 control list_controllers          # both controllers should show "active"
ros2 control list_hardware_interfaces  # 3 joints, position command + position/velocity state
ros2 action list                       # shows /arm_controller/follow_joint_trajectory
ros2 topic echo /joint_states --once   # one joint-state message
```

---

## License & Attribution
Developed by **Pavan N K** ([Workofficialmain](https://github.com/Workofficialmain)).  
Repository: [https://github.com/Workofficialmain/-robot-arm](https://github.com/Workofficialmain/-robot-arm)
