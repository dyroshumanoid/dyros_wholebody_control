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

## 6. WalkingManager
Generates walking-related trajectories, including:

- Footstep sequence and scheduling  
- CoM, ZMP, and Capture Point trajectories  
- Swing foot motion using polynomial interpolation  

---

## 7. TeleOperationManager

TODO

---