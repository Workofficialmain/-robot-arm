# 3-DOF Robotic Manipulator — CAD-to-ROS 2 Motion Planning

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_Hawksbill-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo_Classic_11-orange)](http://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3-blue?logo=python&logoColor=white)](https://www.python.org/)

An end-to-end robotics project for a 3-degree-of-freedom (3-DOF) articulated arm. The workflow covers CAD-based joint geometry extraction, URDF modelling, analytical forward and inverse kinematics, differential kinematics, damped least-squares (DLS) velocity mapping, quintic trajectory generation, trajectory validation, and ROS 2 `FollowJointTrajectory` execution in Gazebo.

The project also measures simulated TCP tracking error from joint-state feedback and forward kinematics. This is a monitoring measurement; it is not closed-loop Cartesian correction. No real-hardware accuracy is claimed.

---

## Table of Contents

1. [Project Workflow](#1-project-workflow)
2. [Robot Mechanical and Kinematic Specifications](#2-robot-mechanical-and-kinematic-specifications)
3. [Kinematics and Mathematical Formulation](#3-kinematics-and-mathematical-formulation)
   - [3.1 Forward Kinematics](#31-forward-kinematics-fk)
   - [3.2 Inverse Kinematics](#32-inverse-kinematics-ik)
   - [3.3 Jacobian and Differential Kinematics](#33-jacobian-and-differential-kinematics)
   - [3.4 Inverse Jacobian and Damped Least Squares](#34-inverse-jacobian-and-damped-least-squares-dls)
   - [3.5 Planning and Execution Pipeline](#35-planning-and-execution-pipeline)
4. [Cartesian Shape Planning](#4-cartesian-shape-planning)
5. [Testing and Verification](#5-testing-and-verification)
6. [Repository Architecture](#6-repository-architecture)
7. [Build and Run](#7-build-and-run)

---

# 1. Project Workflow

The project follows a 12-step engineering workflow. Each stage produces the geometry, model, or software interface required by the next stage.

### Step 1 — CAD modelling and joint-axis extraction

- Inspected the robot's STL parts and FreeCAD assembly.
- Extracted the shoulder and elbow pivot locations and the tool-tip reference point.
- Used the CAD geometry rather than estimated link dimensions.

**Extracted reference points**

| Point | CAD coordinate (mm) |
|---|---:|
| Shoulder pivot | \((0,\;0,\;52.0)\) |
| Elbow pivot | \((30.0,\;0,\;288.8)\) |
| Tool centre point (TCP) | \((378.0,\;0,\;293.0)\) |

### Step 2 — URDF model and `ros2_control`

- Generated the visualization URDF and Gazebo-oriented URDF.
- Assembled the chain `world → robot_base → link_turntable → link_upper_arm → link_forearm → end_effector`.
- Configured visual geometry, collision geometry, inertial properties, joint limits, and the Gazebo control plugin.
- The base is positioned 80 mm above the world origin so the base mesh rests on the Gazebo ground plane.

| File | Purpose |
|---|---|
| `urdf/robot_arm.urdf` | Robot description for visualization |
| `urdf/robot_arm_gz.urdf` | Gazebo-oriented robot description with simulation properties |
| `config/controllers.yaml` | Joint-state broadcaster and trajectory-controller configuration |

### Step 3 — Analytical forward kinematics

Derived closed-form equations mapping the joint vector \(\mathbf q=[q_1,q_2,q_3]^T\) to the TCP position \(\mathbf p=[x,y,z]^T\).

**Implementation:** `forward_kinematics(q)` in `scripts/motion_planning/kinematics.py`.

### Step 4 — Analytical inverse kinematics

Implemented geometric inverse kinematics using the law of cosines, reachability checks, and both elbow configurations.

**Implementation:** `inverse_kinematics(x, y, z, elbow=...)`.

### Step 5 — Differential kinematics

Differentiated the forward-kinematics equations to obtain the \(3\times3\) translational Jacobian. The Jacobian maps joint velocities to TCP linear velocity and is also used in singularity analysis.

### Step 6 — Inverse Jacobian and DLS

Implemented damped least-squares velocity mapping for Cartesian planning near poorly conditioned configurations. Adaptive damping is based on the normalized singularity measure.

### Step 7 — Quintic trajectory generation

Implemented fifth-order time scaling with zero velocity and acceleration at the beginning and end of a segment.

### Step 8 — Cartesian path planning

Implemented straight-line, circular, and rounded-square paths. The path generators combine Cartesian geometry, time scaling, inverse kinematics, and differential velocity mapping.

### Step 9 — Trajectory validation

Checks joint-position limits, joint-velocity limits, estimated joint acceleration, timestamps, and initial-state consistency. The configured acceleration ceiling is a software planning constraint, not a measured actuator rating.

### Step 10 — ROS 2 action integration

The trajectory publisher sends `trajectory_msgs/JointTrajectory` goals through the `control_msgs/action/FollowJointTrajectory` action server at:

`/arm_controller/follow_joint_trajectory`

### Step 11 — Gazebo path visualization

Gazebo marker scripts display square and circular guides and the simulated TCP trail. Guide markers are visual references; they are not robot motion commands.

### Step 12 — Dry-run tests and tracking-error measurement

Validated the planning pipeline in dry-run mode and measured simulated tracking error during an explicitly executed straight-line trajectory. The measurement uses `/joint_states` and forward kinematics to estimate the actual TCP position.

Reported simulation result from the recorded run:

| Metric | Measured value |
|---|---:|
| RMS TCP tracking error | 0.1207 mm |
| Maximum TCP tracking error | 0.1878 mm |
| Final settled position error | 0.0115 mm |

These values describe that Gazebo simulation run only.

---

# 2. Robot Mechanical and Kinematic Specifications

## 2.1 Link lengths and pivot points

The robot is a 3-DOF vertical articulated arm with an RRR joint arrangement:

- \(q_1\): base azimuth rotation about the vertical axis.
- \(q_2\): shoulder pitch.
- \(q_3\): elbow pitch.

The two pitch joints move in the arm's vertical plane.

### Geometric reference

| Parameter | Value | Description |
|---|---:|---|
| CAD origin | \((0,\;0,\;0)\) mm | Base reference origin |
| Turntable axis | \((0,\;0,\;12.5)\) mm | Base rotation axis |
| Shoulder pivot | \((0,\;0,\;52.0)\) mm | Shoulder pitch axis |
| Elbow pivot | \((30.0,\;0,\;288.8)\) mm | Elbow pitch axis |
| Zero-state TCP | \((378.0,\;0,\;293.0)\) mm | Tool-tip reference point |

### Link dimensions

The first link is the distance from the shoulder pivot to the elbow pivot:

\[
\begin{aligned}
L_1
&=\sqrt{(30.0-0)^2+(288.8-52.0)^2}\\
&=\sqrt{30.0^2+236.8^2}\\
&\approx 238.693\ \text{mm}.
\end{aligned}
\]

The second link is the distance from the elbow pivot to the TCP:

\[
\begin{aligned}
L_2
&=\sqrt{(378.0-30.0)^2+(293.0-288.8)^2}\\
&=\sqrt{348.0^2+4.2^2}\\
&\approx 348.025\ \text{mm}.
\end{aligned}
\]

### Kinematic reference angles

The CAD links are not perfectly horizontal or vertical in the zero joint configuration. Their fixed geometric offsets are represented by the reference angles:

\[
\Phi_{1,0}
=\mathrm{atan2}(236.8,\;30.0)
\approx 1.44474\ \text{rad}
\approx 82.78^\circ
\]

\[
\Phi_{2,0}
=\mathrm{atan2}(4.2,\;348.0)
\approx 0.012068\ \text{rad}
\approx 0.691^\circ.
\]

Here, \(\mathrm{atan2}(y,x)\) denotes the four-quadrant inverse tangent.

### Joint limits and motion constraints

| Joint | Physical motion | Lower limit | Upper limit | Maximum velocity | Planning acceleration limit |
|---|---|---:|---:|---:|---:|
| `joint_base_rotate` | \(q_1\) | \(-3.14\) rad | \(+3.14\) rad | 2.0 rad/s | 4.0 rad/s² |
| `joint_shoulder` | \(q_2\) | \(-1.57\) rad | \(+1.57\) rad | 2.0 rad/s | 4.0 rad/s² |
| `joint_elbow` | \(q_3\) | \(-1.57\) rad | \(+1.57\) rad | 2.0 rad/s | 4.0 rad/s² |

The velocity limits are defined in the URDF. The acceleration limit is applied by the software planner; it is not a hardware-rated acceleration specification.

---

# 3. Kinematics and Mathematical Formulation

All lengths in the equations below are expressed in millimetres, all joint angles in radians, and all joint velocities in radians per second.

Let the joint vector be

\[
\mathbf q=
\begin{bmatrix}
q_1\\q_2\\q_3
\end{bmatrix}.
\]

Define the two link elevation angles:

\[
\boxed{
\begin{aligned}
\phi_1 &= \Phi_{1,0}-q_2,\\
\phi_2 &= \Phi_{2,0}-q_2-q_3.
\end{aligned}}
\]

The shoulder height is \(z_s=52.0\) mm.

## 3.1 Forward Kinematics (FK)

### Purpose

Forward kinematics maps the three joint angles to the Cartesian position of the TCP. It is used to:

- Calculate the TCP position from joint-state feedback.
- Verify the endpoint of a planned joint trajectory.
- Calculate Cartesian tracking error from the measured joint state.

### Radial reach

The horizontal radial distance from the base axis to the TCP is

\[
\boxed{
r=L_1\cos\phi_1+L_2\cos\phi_2.
}
\]

### Cartesian position

The TCP coordinates are

\[
\boxed{
\begin{aligned}
x &= r\cos q_1,\\
y &= r\sin q_1,\\
z &= z_s+L_1\sin\phi_1+L_2\sin\phi_2.
\end{aligned}}
\]

Equivalently, substituting the radial reach,

\[
\boxed{
\mathbf p(\mathbf q)=
\begin{bmatrix}
\left(L_1\cos\phi_1+L_2\cos\phi_2\right)\cos q_1\\
\left(L_1\cos\phi_1+L_2\cos\phi_2\right)\sin q_1\\
z_s+L_1\sin\phi_1+L_2\sin\phi_2
\end{bmatrix}.
}
\]

### Zero-position calculation

For the zero joint vector,

\[
\mathbf q=
\begin{bmatrix}
0&0&0
\end{bmatrix}^{T},
\]

the link angles are

\[
\phi_1=\Phi_{1,0}\approx1.44474\ \text{rad},
\qquad
\phi_2=\Phi_{2,0}\approx0.012068\ \text{rad}.
\]

From the CAD dimensions,

\[
\cos\phi_1=\frac{30.0}{238.693},
\qquad
\sin\phi_1=\frac{236.8}{238.693},
\]

\[
\cos\phi_2=\frac{348.0}{348.025},
\qquad
\sin\phi_2=\frac{4.2}{348.025}.
\]

Therefore,

\[
\begin{aligned}
r
&=238.693\left(\frac{30.0}{238.693}\right)
 +348.025\left(\frac{348.0}{348.025}\right)\\
&=30.0+348.0\\
&=378.0\ \text{mm}.
\end{aligned}
\]

The Cartesian coordinates are

\[
x=378.0\cos 0=378.0\ \text{mm},
\]

\[
y=378.0\sin 0=0.0\ \text{mm},
\]

\[
\begin{aligned}
z
&=52.0+238.693\left(\frac{236.8}{238.693}\right)
 +348.025\left(\frac{4.2}{348.025}\right)\\
&=52.0+236.8+4.2\\
&=293.0\ \text{mm}.
\end{aligned}
\]

Thus,

\[
\boxed{
\mathbf p(0,0,0)=
\begin{bmatrix}
378.0\\0.0\\293.0
\end{bmatrix}
\ \text{mm}.
}
\]

This matches the CAD TCP reference point.

## 3.2 Inverse Kinematics (IK)

### Purpose

Inverse kinematics calculates joint angles for a requested Cartesian TCP position. It is used by the Cartesian path planners to convert position waypoints into joint-space targets.

Given a target point

\[
\mathbf p_d=
\begin{bmatrix}
x&y&z
\end{bmatrix}^{T},
\]

the solution is calculated geometrically.

### Step 1 — Base rotation

\[
\boxed{q_1=\mathrm{atan2}(y,x).}
\]

### Step 2 — Project into the arm plane

The radial distance and vertical displacement relative to the shoulder are

\[
\boxed{
r=\sqrt{x^2+y^2},
\qquad
z_t=z-z_s=z-52.0.
}
\]

The squared shoulder-to-target distance is

\[
d^2=r^2+z_t^2.
\]

### Step 3 — Law of cosines

Applying the law of cosines to the triangle with sides \(L_1\), \(L_2\), and \(d\),

\[
\boxed{
D=\frac{d^2-L_1^2-L_2^2}{2L_1L_2}.
}
\]

A target is reachable by this planar two-link geometry only if

\[
-1\leq D\leq1.
\]

Values outside this interval indicate that the target lies outside the geometric reach of the arm, subject also to the configured joint limits.

### Step 4 — Elbow configurations

The two geometric elbow solutions are

\[
\boxed{
\delta_{\text{up}}=-\cos^{-1}(D),
\qquad
\delta_{\text{down}}=+\cos^{-1}(D).
}
\]

The selected solution depends on the requested `elbow` configuration.

### Step 5 — Shoulder and forearm elevation angles

For the selected \(\delta\),

\[
\boxed{
\phi_1=
\mathrm{atan2}(z_t,r)
-
\mathrm{atan2}
\left(
L_2\sin\delta,\;
L_1+L_2\cos\delta
\right).
}
\]

Then

\[
\boxed{\phi_2=\phi_1+\delta.}
\]

### Step 6 — Convert to URDF joint angles

Using the fixed CAD reference angles,

\[
\boxed{
\begin{aligned}
q_2 &= \Phi_{1,0}-\phi_1,\\
q_3 &= \Phi_{2,0}-\phi_2-q_2.
\end{aligned}}
\]

The resulting joint vector is

\[
\boxed{
\mathbf q=
\begin{bmatrix}
q_1\\q_2\\q_3
\end{bmatrix}.
}
\]

The implementation checks reachability and joint limits before returning a usable solution.

### Zero-position inverse-kinematics check

For the target

\[
(x,y,z)=(378.0,\;0.0,\;293.0)\ \text{mm},
\]

the planar target coordinates are

\[
r=378.0\ \text{mm},
\qquad
z_t=293.0-52.0=241.0\ \text{mm}.
\]

The squared distance is

\[
d^2=378.0^2+241.0^2=200965.
\]

Using the link lengths,

\[
L_1^2+L_2^2
=238.693^2+348.025^2
\approx178095.88.
\]

Therefore,

\[
D=
\frac{200965-178095.88}
{2(238.693)(348.025)}
\approx0.13765.
\]

For the elbow-up solution,

\[
\delta=-\cos^{-1}(0.13765)\approx-1.4327\ \text{rad}.
\]

Substitution into the shoulder-angle equation gives approximately

\[
\phi_1\approx1.44474\ \text{rad},
\qquad
\phi_2=\phi_1+\delta\approx0.01207\ \text{rad}.
\]

The joint angles are consequently

\[
q_1=0,\qquad q_2\approx0,\qquad q_3\approx0.
\]

Hence the inverse-kinematics result agrees with the zero-position forward-kinematics result, within numerical precision.

## 3.3 Jacobian and Differential Kinematics

### Purpose

The translational Jacobian describes how joint velocities produce TCP linear velocity. It is used by the Cartesian planners to convert a desired TCP velocity into joint velocities.

Define

\[
\dot{\mathbf q}=
\begin{bmatrix}
\dot q_1\\\dot q_2\\\dot q_3
\end{bmatrix},
\qquad
\dot{\mathbf p}=
\begin{bmatrix}
\dot x\\\dot y\\\dot z
\end{bmatrix}.
\]

The differential-kinematics relationship is

\[
\boxed{
\dot{\mathbf p}=J_v(\mathbf q)\dot{\mathbf q}.
}
\]

Here \(J_v\) is the \(3\times3\) translational Jacobian.

### Derivation

Recall

\[
r=L_1\cos\phi_1+L_2\cos\phi_2,
\]

with

\[
\phi_1=\Phi_{1,0}-q_2,
\qquad
\phi_2=\Phi_{2,0}-q_2-q_3.
\]

Differentiating \(r\) with respect to the shoulder and elbow variables gives

\[
\boxed{
\begin{aligned}
d_{r2}
&=\frac{\partial r}{\partial q_2}
=L_1\sin\phi_1+L_2\sin\phi_2,\\[2mm]
d_{r3}
&=\frac{\partial r}{\partial q_3}
=L_2\sin\phi_2.
\end{aligned}}
\]

Since

\[
x=r\cos q_1,\qquad y=r\sin q_1,
\]

their partial derivatives are

\[
\begin{aligned}
\frac{\partial x}{\partial q_1}&=-r\sin q_1,
&
\frac{\partial x}{\partial q_2}&=d_{r2}\cos q_1,
&
\frac{\partial x}{\partial q_3}&=d_{r3}\cos q_1,\\[1mm]
\frac{\partial y}{\partial q_1}&=r\cos q_1,
&
\frac{\partial y}{\partial q_2}&=d_{r2}\sin q_1,
&
\frac{\partial y}{\partial q_3}&=d_{r3}\sin q_1.
\end{aligned}
\]

For the vertical coordinate,

\[
z=z_s+L_1\sin\phi_1+L_2\sin\phi_2,
\]

so

\[
\frac{\partial z}{\partial q_1}=0,
\qquad
\frac{\partial z}{\partial q_2}=-r,
\qquad
\frac{\partial z}{\partial q_3}=-L_2\cos\phi_2.
\]

Arranging the partial derivatives gives

\[
\boxed{
J_v(\mathbf q)=
\begin{bmatrix}
-r\sin q_1 & d_{r2}\cos q_1 & d_{r3}\cos q_1\\
 r\cos q_1 & d_{r2}\sin q_1 & d_{r3}\sin q_1\\
 0         & -r             & -L_2\cos\phi_2
\end{bmatrix}.
}
\]

Therefore,

\[
\boxed{
\begin{bmatrix}
\dot x\\\dot y\\\dot z
\end{bmatrix}
=
\begin{bmatrix}
-r\sin q_1 & d_{r2}\cos q_1 & d_{r3}\cos q_1\\
 r\cos q_1 & d_{r2}\sin q_1 & d_{r3}\sin q_1\\
 0         & -r             & -L_2\cos\phi_2
\end{bmatrix}
\begin{bmatrix}
\dot q_1\\\dot q_2\\\dot q_3
\end{bmatrix}.
}
\]

### Full geometric Jacobian

For completeness, the full geometric Jacobian also includes angular velocity. Let

\[
\mathbf e_\theta=
\begin{bmatrix}
-\sin q_1\\
\cos q_1\\
0
\end{bmatrix}.
\]

The base joint axis is the world \(z\)-axis, while the shoulder and elbow pitch axes are parallel to \(\mathbf e_\theta\). The full \(6\times3\) geometric Jacobian is therefore

\[
\boxed{
J_g(\mathbf q)=
\begin{bmatrix}
-r\sin q_1 & d_{r2}\cos q_1 & d_{r3}\cos q_1\\
 r\cos q_1 & d_{r2}\sin q_1 & d_{r3}\sin q_1\\
 0         & -r             & -L_2\cos\phi_2\\[1mm]
 0         & -\sin q_1     & -\sin q_1\\
 0         & \cos q_1      & \cos q_1\\
 1         & 0             & 0
\end{bmatrix}.
}
\]

Its upper three rows map joint velocity to TCP linear velocity; its lower three rows map joint velocity to TCP angular velocity. The current Cartesian planner uses the \(3\times3\) translational Jacobian \(J_v\), not the full \(6\times3\) matrix.

### Implementation

In `scripts/motion_planning/kinematics.py`, `jacobian(q)`:

1. Reads the three joint angles.
2. Calculates \(\phi_1\), \(\phi_2\), and \(r\).
3. Calculates \(d_{r2}\) and \(d_{r3}\).
4. Constructs and returns the \(3\times3\) translational Jacobian.

## 3.4 Inverse Jacobian and Damped Least Squares (DLS)

### Why direct inversion can fail

For a nonsingular square Jacobian, joint velocity could be calculated using

\[
\dot{\mathbf q}=J_v^{-1}\dot{\mathbf p}.
\]

Near a singular configuration, however, the Jacobian becomes singular or poorly conditioned. Direct inversion can become numerically unstable and may produce excessively large joint velocities.

The planner therefore uses a damped least-squares pseudoinverse.

### Damped least-squares formulation

DLS finds joint velocities that minimise Cartesian velocity error while penalising large joint velocities:

\[
\boxed{
\min_{\dot{\mathbf q}}
\left[
\left\|J_v\dot{\mathbf q}-\dot{\mathbf p}\right\|_2^2
+\lambda^2\left\|\dot{\mathbf q}\right\|_2^2
\right].
}
\]

The corresponding damped pseudoinverse is

\[
\boxed{
J^+_\lambda=
\left(J_v^T J_v+\lambda^2 I\right)^{-1}J_v^T.
}
\]

The joint-velocity command is

\[
\boxed{
\dot{\mathbf q}=J^+_\lambda\dot{\mathbf p}.
}
\]

Here \(I\) is the \(3\times3\) identity matrix and \(\lambda\) is the damping factor. Larger damping reduces the effect of poorly conditioned directions, at the cost of some Cartesian velocity accuracy.

### Normalized singularity measure

The project calculates a normalized determinant measure:

\[
\boxed{
\mu=
\frac{|\det(J_v)|}
{\|J_{v,:,1}\|_2\,
 \|J_{v,:,2}\|_2\,
 \|J_{v,:,3}\|_2}.
}
\]

Here \(J_{v,:,i}\) denotes column \(i\) of the translational Jacobian. A small \(\mu\) indicates that the Jacobian columns are approaching linear dependence.

The adaptive damping rule used by the planner is

\[
\boxed{
\lambda
=0.1+5\max\left(0,\;1-\frac{\mu}{0.1}\right).
}
\]

Thus, damping increases as the normalized singularity measure approaches zero.

### Analytical inverse and DLS are different methods

The source also contains `analytical_inverse_jacobian(q)` and `analytical_inverse_joint_velocity(...)` for the project's analytical inverse formulation. These are useful for mathematical verification, but their denominators can approach zero at singular configurations.

The practical Cartesian trajectory planners use `inverse_jacobian(q, damping=...)`, which evaluates the DLS expression. The analytical inverse and the DLS pseudoinverse are separate implementations and should not be treated as interchangeable near singularities.

## 3.5 Planning and Execution Pipeline

The runtime flow is:

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
             [IK]   [Singularity Measure]
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

The trajectory publisher validates joint positions, joint velocities, estimated accelerations, timestamps, and the initial joint state before execution. The `safety.py` module provides separate validation and retiming utilities; the main publisher does not automatically call `make_trajectory_safe()` when validation fails.

TCP tracking error is calculated from joint-state feedback and forward kinematics. This is a measurement of simulated tracking performance, not Cartesian closed-loop correction. The \(4.0\ \text{rad/s}^2\) acceleration ceiling is a software planning constraint, not a verified hardware rating.

For each Cartesian sample, the planner calculates:

- Desired TCP position.
- Desired TCP linear velocity.
- An inverse-kinematics joint-position solution.
- A singularity measure and damping value.
- A DLS joint-velocity solution.

The resulting joint positions and velocities are passed to trajectory construction. Joint limits and trajectory constraints are checked separately; DLS is not a substitute for safety validation.

---

# 4. Cartesian Shape Planning

## 4.1 Circle path generation

The standalone `plan_cartesian_circle()` function in `shape_planner.py` generates a circular path in the XY plane.

For a starting point \((x_s,y_s,z_s)\) and radius \(R\), the circle centre is

\[
c_x=x_s,\qquad c_y=y_s-R.
\]

The path is

\[
\boxed{
\begin{aligned}
x(\theta)&=c_x+R\cos\theta,\\
y(\theta)&=c_y+R\sin\theta,\\
z(\theta)&=z_s,
\end{aligned}
\qquad
\frac{\pi}{2}\leq\theta\leq\frac{5\pi}{2}.
}
\]

The tangent direction is

\[
\mathbf t(\theta)=
\begin{bmatrix}
-\sin\theta\\
\cos\theta\\
0
\end{bmatrix}.
\]

Quintic time scaling controls progress along the path, with zero speed at the start and end.

**Execution note:** `trajectory_publisher.py` constructs the executed circle using `_arc_segment()` and `_plan_path()`, with the supplied centre and an angular range from \(0\) to \(2\pi\). The execution path therefore starts at the rightmost point of the circle. The demonstration includes approach, move to tracing start, circle tracing, retract, and return stages.

## 4.2 Rounded-square path generation

A sharp square has discontinuous tangent direction at each corner if traversed continuously. The planner rounds the corners by combining four straight edges with four quarter-circle arcs.

The `plan_cartesian_square()` function accepts a starting point

\[
P_0=(x,y,z),
\]

a square side length \(S\), and a blend radius \(r_b\), subject to

\[
\boxed{
S>0,\qquad 0<r_b<\frac{S}{2}.
}
\]

The path consists of four line segments and four quarter-circle arcs:

1. Bottom edge: \((x,y,z)\) to \((x+S-2r_b,y,z)\).
2. Bottom-right arc: centre \((x+S-2r_b,y+r_b)\), angle \(-\pi/2\) to \(0\).
3. Right edge: \((x+S-r_b,y+r_b,z)\) to \((x+S-r_b,y+S-r_b,z)\).
4. Top-right arc: centre \((x+S-2r_b,y+S-r_b)\), angle \(0\) to \(\pi/2\).
5. Top edge: \((x+S-2r_b,y+S,z)\) to \((x,y+S,z)\).
6. Top-left arc: centre \((x,y+S-r_b)\), angle \(\pi/2\) to \(\pi\).
7. Left edge: \((x-r_b,y+S-r_b,z)\) to \((x-r_b,y+r_b,z)\).
8. Bottom-left arc: centre \((x,y+r_b)\), angle \(\pi\) to \(3\pi/2\).

The resulting coordinate bounds are

\[
\boxed{
x-r_b\leq X\leq x+S-r_b,
\qquad
y\leq Y\leq y+S.
}
\]

The line and arc segments meet with matching tangent directions. This provides geometric tangent continuity, but does not imply continuous curvature or acceleration at every join.

### Quintic time scaling

The path progress variable is

\[
\boxed{
s(u)=10u^3-15u^4+6u^5,
\qquad
u=\frac{t}{T},
\qquad 0\leq u\leq1.
}
\]

This scaling starts and finishes with zero velocity and zero acceleration. At each sample, the planner calculates the Cartesian position and tangent velocity, solves inverse kinematics, evaluates the singularity measure, applies adaptive DLS, and checks joint limits.

---

# 5. Testing and Verification

The tests below document software-level mathematical checks, trajectory dry runs, and Gazebo simulation execution. Physical-hardware validation has not been performed.

## 5.1 Kinematics consistency test

- **Objective:** Check that inverse kinematics reconstructs positions generated by forward kinematics.
- **Method:** Sampled 1,000 joint configurations and tested both elbow configurations where applicable.
- **Recorded result:** Worst-case Cartesian reconstruction error \(\leq 3.75\times10^{-13}\) mm.
- **Status:** PASS.

## 5.2 Jacobian gradient test

- **Objective:** Compare analytical Jacobian entries with numerical finite-difference gradients.
- **Method:** Finite-difference step size \(\epsilon=10^{-6}\).
- **Recorded result:** Residual approximately \(4.25\times10^{-8}\).
- **Status:** PASS.

## 5.3 Square trajectory dry run

| Parameter | Value |
|---|---:|
| Side length | 20 mm |
| Blend radius | 4 mm |
| Duration | 8.0 s |
| Frequency | 50 Hz |
| Waypoints | 401 |
| Path closure error | 0.0000 mm |
| Maximum joint velocity | 0.0816 rad/s |
| Joint velocity limit | 2.0 rad/s |
| Maximum estimated acceleration | 0.3400 rad/s² |
| Planning acceleration limit | 4.0 rad/s² |

All recorded safety checks passed. Dry-run mode did not command robot motion.

**Status: PASS.**

## 5.4 Circle trajectory dry run

| Parameter | Value |
|---|---:|
| Radius | 10 mm |
| Duration | 8.0 s |
| Frequency | 50 Hz |
| Waypoints | 401 |
| Path closure error | 0.0000 mm |
| Maximum joint velocity | 0.0675 rad/s |
| Joint velocity limit | 2.0 rad/s |
| Maximum estimated acceleration | 0.0831 rad/s² |
| Planning acceleration limit | 4.0 rad/s² |

All recorded safety checks passed. Dry-run mode did not command robot motion.

**Status: PASS.**

## 5.5 Gazebo execution and TCP tracking-error measurement

A Cartesian straight-line movement of \(\Delta Y=20\) mm and return was executed through ROS 2 `FollowJointTrajectory`.

| Metric | Recorded result |
|---|---:|
| Action goal accepted | Yes |
| Action result | SUCCESS (`error_code = 0`) |
| Feedback samples | 101 |
| RMS tracking error | 0.1207 mm |
| Maximum tracking error | 0.1878 mm |
| Final settled position error | 0.0115 mm |

**Status: PASS.** This is a sub-millimetre tracking-error measurement from Gazebo physics simulation. It is not a real-hardware result.

---

# 6. Repository Architecture

The following tables describe the principal files and their responsibilities.

## 6.1 Robot model, simulation, and configuration

| File / folder | Responsibility |
|---|---|
| `urdf/robot_arm.urdf` | Visualization-oriented robot description: links, joints, and visual geometry |
| `urdf/robot_arm_gz.urdf` | Gazebo robot description with inertial, collision, and control configuration |
| `meshes/` | STL geometry used by the URDF files |
| `config/controllers.yaml` | Joint-state broadcaster and arm trajectory-controller configuration |
| `launch/gazebo.launch.py` | Starts Gazebo, robot-state publishing, entity spawning, and controller startup |

## 6.2 Core motion-planning modules

| File | Responsibility |
|---|---|
| `kinematics.py` | Analytical FK, IK, translational Jacobian, inverse Jacobian, and singularity calculations |
| `joint_mapping.py` | Maps mathematical joint-vector order to ROS controller joint names and order |
| `trajectories.py` | Quintic interpolation and trajectory time-scaling utilities |
| `cartesian_planner.py` | Cartesian straight-line path planning and joint-space conversion |
| `shape_planner.py` | Circle and rounded-square Cartesian path generation |
| `safety.py` | Standalone trajectory checks and dynamic time-scaling utilities |
| `ros_trajectory.py` | Converts planned samples into ROS trajectory messages |
| `trajectory_publisher.py` | Dry-run validation, action execution, and joint-state-based TCP tracking-error measurement |

## 6.3 Motion-planning demonstrations

| File | Responsibility |
|---|---|
| `square_demo.py` | Square demonstration using the shape planner and Gazebo visualization |
| `circle_demo.py` | Circle demonstration using the shape planner and Gazebo visualization |

## 6.4 Gazebo visualization and marker utilities

| File | Responsibility |
|---|---|
| `scripts/square/square_guide.py` | Spawns visual guide markers for the square outline |
| `scripts/circle/circle_guide.py` | Spawns visual guide markers for the circle outline |
| `scripts/circle/legacy/circle_trace.py` | Earlier circle-tracing implementation retained for reference |
| `scripts/circle/legacy/tip_trail.py` | Legacy TCP trail visualization node |
| `scripts/common/clear_marks.py` | Removes project guide and trail markers from Gazebo |

Guide scripts create visual references in Gazebo. The motion planner independently generates the robot trajectory.

## 6.5 Setup and CAD extraction scripts

| File | Responsibility |
|---|---|
| `scripts/setup/freecad/shoulder_axes.py` | Extracts shoulder joint-axis geometry from the CAD model |
| `scripts/setup/freecad/elbow_wrist_axes.py` | Extracts elbow and wrist axis geometry |
| `scripts/setup/freecad/wrist_axes.py` | Extracts wrist-axis geometry |
| `scripts/setup/make_urdf.py` | Generates the visualization-oriented URDF; currently contains a machine-specific workspace path |
| `scripts/setup/make_gz_urdf.py` | Generates the Gazebo-oriented URDF; currently contains a machine-specific workspace path |
| `scripts/setup/move_joints.py` | Small joint-state publishing utility for testing; not the main trajectory execution node |

## 6.6 Runner scripts and package files

| File | Responsibility |
|---|---|
| `run/start_gazebo.sh` | Sources Gazebo and ROS 2 environments and launches the simulation |
| `run/run_square.sh` | Clears markers, creates the square guide, and runs the square demonstration; supports `--dry` |
| `run/run_circle.sh` | Clears markers, creates the circle guide, and runs the circle demonstration; supports `--dry` |
| `CMakeLists.txt` | Defines package build and installation rules |
| `package.xml` | Declares ROS 2 package metadata and dependencies |
| `.gitignore` | Excludes generated and local-only files from Git |

## 6.7 Main module relationships

**Straight-line trajectory**

```text
cartesian_planner.py
        |
        v
  kinematics.py
        |
        v
 joint_mapping.py
        |
        v
 ros_trajectory.py
        |
        v
trajectory_publisher.py
```

**Square demonstration**

```text
run/run_square.sh
       |
       +--> scripts/common/clear_marks.py
       |
       +--> scripts/square/square_guide.py
       |
       +--> square_demo.py
                  |
                  v
           shape_planner.py
                  |
                  v
       Kinematics and trajectory modules
```

**Circle demonstration**

```text
run/run_circle.sh
       |
       +--> scripts/common/clear_marks.py
       |
       +--> scripts/circle/circle_guide.py
       |
       +--> circle_demo.py
                  |
                  v
           shape_planner.py
                  |
                  v
       Kinematics and trajectory modules
```

The guide scripts draw visual outlines. The planner generates the robot path, and the ROS trajectory components convert and send the trajectory to the controller. Tracking-error measurement uses joint-state feedback and forward kinematics; it does not implement Cartesian closed-loop correction.

---

# 7. Build and Run

## 7.1 Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill (`ros-humble-desktop`)
- Gazebo Classic 11
- `ros-humble-gazebo-ros-pkgs`
- `ros-humble-ros2-control`
- `ros-humble-ros2-controllers`
- `ros-humble-gazebo-ros2-control`

## 7.2 Build the workspace

```bash
cd ~/robot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 7.3 Launch Gazebo

In a dedicated terminal:

```bash
cd ~/robot_arm_ws/src/robot_arm_description
./run/start_gazebo.sh
```

Wait for Gazebo to launch and for both `joint_state_broadcaster` and `arm_controller` to become active.

## 7.4 Run dry-run validation

Dry-run mode performs planning and validation without commanding robot movement.

```bash
cd ~/robot_arm_ws

# 20 mm rounded-square path
python3 src/robot_arm_description/scripts/motion_planning/trajectory_publisher.py \
  --shape square --size 20 --blend-radius 4 --dry

# 10 mm radius circle
python3 src/robot_arm_description/scripts/motion_planning/trajectory_publisher.py \
  --shape circle --radius 10 --dry
```

## 7.5 Execute trajectories in Gazebo

Execution requires the explicit `--execute` option for the trajectory publisher.

```bash
cd ~/robot_arm_ws

# Straight-line movement: +20 mm along the Y axis
python3 src/robot_arm_description/scripts/motion_planning/trajectory_publisher.py \
  --shape line --dy 20 --execute
```

For the marker-based demonstrations:

```bash
cd ~/robot_arm_ws/src/robot_arm_description

# Square demonstration
./run/run_square.sh

# Circle demonstration
./run/run_circle.sh
```

## 7.6 Clear Gazebo markers

```bash
cd ~/robot_arm_ws
python3 src/robot_arm_description/scripts/common/clear_marks.py
```

---

## License and Attribution

Developed by Pavan N K (Workofficialmain).

Repository: [Workofficialmain/-robot-arm](https://github.com/Workofficialmain/-robot-arm)

ROS 2 Humble robot arm simulation using Gazebo Classic.
