# DYROS WHOLE-BODY CONTROL

---

> The `dyros_wholebody_control` project is a C++ whole-body control (WBC) framework for the  
> [**TOCABI humanoid robot**](https://ieeexplore.ieee.org/document/10000102).  
> This package provides **safety-critical kinematic and dynamic WBC**, trajectory managers,  
> and teleoperation interfaces.

---

# Prerequisites

- This project depends on the TOCABI software ecosystem.
- Please refer to the [TOCABI controller repository](https://github.com/saga0619/dyros_tocabi_v2.git) for detailed instructions on installing and configuring all required library dependencies.  

---

# How to Use

This is a ROS1 `catkin` package.

```bash
cd ~/catkin_ws/src
git clone https://github.com/dyroshumanoid/dyros_wholebody_control.git
cd ~/catkin_ws
catkin_make
# or
catkin build
```
---

# How to Launch
  * simulation with gui : `roslaunch tocabi_controller simulation.launch` 

  * to launch simulation or realrobot with hands, launch with hand:=true :   * `roslaunch tocabi_controller simulation.launch hand:=true`



# Main Components

## 1. CustomController (`cc.cpp`, `cc.h`)
The top-level controller interface.

It receives `RobotData` (from `tocabi_lib`), performs task management, calls:

- ControlManager  
- TaskManager  
- KinWBC  
- DynWBC  
- WalkingManager  
- TeleOperationManager  

and outputs **joint torque commands** (`Eigen::VectorQd`).

---

## 2. ControlManager
Responsible for:

- Preprocessing robot state for WBC  
- Frame transformations to the base or support foot frame  
- Managing contact state information  

---

## 3. TaskManager
Generates task trajectories depending on the selected **TaskMotionType**:

| Enum Value | Motion Mode |
|------------|-------------|
| `PelvHand` | Pelvis–hand motion tracking |
| `Taichi` | Taichi-style motion |
| `Walking` | Bipedal walking |
| `TeleOperation` | VR-based whole-body retargeting |

Task-related parameters are loaded from `tocabi_cc/setting/setting_cc_param.yaml`, including:

- Trajectory duration  
- Pelvis/hand displacement  
- Walking step length & height  
- Double and single support phase durations  

---

## 4. KinWBC (Kinematic Whole-Body Control)
A hierarchical inverse-kinematics-based controller.

Features:

- Supports task hierarchy definition  
- Computes joint velocities/positions that satisfy prioritized tasks  
- Generates safe joint trajectories to follow high-level task commands  
- Ensures safety using **input-to-state safety (ISS)–based Control Barrier Functions (CBFs)**  
- Used for teleoperation, joint tasks, pelvis tasks, hand tasks, and more  

Safety-related parameters are loaded from `tocabi_cc/setting/setting_cc_param.yaml`, including:

- Joint position and velocity limits  
- Distance limits between each hand and shoulder  
- CBF-related parameters for ensuring input-to-state safety  

## 5. DynWBC (Dynamic Whole-Body Control)
A QP-based dynamic whole-body torque controller implemented with **qpOASES**.

DynWBC computes joint torques by solving a constrained optimization problem that tracks
desired joint accelerations and contact wrenches while enforcing whole-body dynamics and safety constraints.

---

### **Tracking Objectives**

The QP tracks the following quantities:

- **Desired joint accelerations**: $\ddot{q}_{des}$
- **Desired contact wrenches**: $\boldsymbol{F}_{des}$

---

### **Regularization Terms**

The controller includes regularization to ensure smooth and stable control signals:

- **Torque smoothing**: minimizing $\|\boldsymbol{\tau}\|_{R}^{2}$
- **Acceleration energy minimization**: minimizing $\|\ddot{q}\|_{Q}^{2}$

---

### **Constraints**

The optimization enforces:

- **Whole-body rigid-body dynamics**  
  $$
  M(q)\ddot{q} + h(q,\dot{q}) = S^\top \boldsymbol{\tau} + J_c^\top \boldsymbol{F}
  $$
- **Friction cone constraints**  
  $$
  \boldsymbol{F}_c \in \mathcal{K}_\mu
  $$
- **Torque, joint, and contact limits**

---

### **QP Formulation**

DynWBC solves the following quadratic program:

$$
\begin{aligned}
\min_{\ddot{q},\, \boldsymbol{\tau},\, \boldsymbol{F}}
\quad &
\underbrace{\|\ddot{q} - \ddot{q}_{des}\|_{Q_{\ddot{q}}}^{2}}_{\text{Joint acceleration tracking}}
+
\underbrace{\|\boldsymbol{F} - \boldsymbol{F}_{des}\|_{Q_F}^{2}}_{\text{Contact wrench tracking}}
+
\underbrace{\|\boldsymbol{\tau}\|_{R_\tau}^{2}}_{\text{Torque smoothing}}
\\[6pt]
\text{subject to:} \quad &
M(q)\ddot{q} + h(q,\dot{q}) = S^\top \boldsymbol{\tau} + J_c^\top \boldsymbol{F}
\quad (\text{Whole-body dynamics})
\\[6pt]
& \boldsymbol{F}_c \in \mathcal{K}_{\mu}
\quad (\text{Friction cone constraints})
\\[6pt]
& \boldsymbol{\tau}_{min} \le \boldsymbol{\tau} \le \boldsymbol{\tau}_{max}
\\[4pt]
& \dot{q}_{min} \le \dot{q} \le \dot{q}_{max}
\end{aligned}
$$

---

### **Notation**

- $M(q)$: Mass matrix  
- $h(q,\dot{q})$: Coriolis, gravity, and nonlinear terms  
- $S$: Actuation selection matrix  
- $J_c$: Contact Jacobian  
- $\mathcal{K}_{\mu}$: Friction cone defined by coefficient $\mu$

### **Output**
The QP returns:

- **Joint torques** \( \boldsymbol{\tau} \) consistent with the full-body dynamics  
- Contact wrenches and joint accelerations that respect task priorities and constraints  



---

## 6. WalkingManager
Generates walking-related trajectories, including:

- Footstep sequence and scheduling  
- CoM, ZMP, and Capture Point trajectories  
- Swing foot motion using polynomial interpolation  

---

## 7. TeleOperationManager

TODO

---