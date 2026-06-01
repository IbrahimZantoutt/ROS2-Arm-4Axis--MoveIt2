# MoveIt 2 + Gazebo Classic integration (ROS 2 Humble)

Reference for how this arm was wired up to run in Gazebo Classic with MoveIt 2.
Project: 6-axis arm, packages `moveit_arm_package` (description + nodes) and
`moveit_config` (MoveIt Setup Assistant output).

> Key idea: in ROS 2, Gazebo runs the `ros2_control` `controller_manager`
> *inside* the simulation (via the `gazebo_ros2_control` plugin). MoveIt's
> `move_group` plans, then sends trajectories to those Gazebo controllers
> through a `FollowJointTrajectory` action. Nothing uses the ROS 1
> `<rosparam>` / `.launch` / `gazebo_ros_control` patterns.

---

## 1. Controllers config (the "joint states" config)

Created `moveit_arm_package/config/controllers.yaml`. This is loaded by the
`gazebo_ros2_control` plugin and defines the controllers Gazebo spawns:

- `joint_state_broadcaster` — publishes `/joint_states` (ROS 2 name for the old
  ROS 1 `joint_state_controller`).
- `arm_controller` — `JointTrajectoryController` for the 6 arm joints. Its name
  and `follow_joint_trajectory` action must match `moveit_controllers.yaml`.
- `gripper_controller` — `JointTrajectoryController` for the 2 finger joints.

`controller_manager.ros__parameters.update_rate: 100`.

## 2. URDF / xacro changes (`moveit_arm_package/urdf/arm.urdf.xacro`)

1. **Hardware plugin** in the `<ros2_control>` block:
   `mock_components/GenericSystem` → `gazebo_ros2_control/GazeboSystem`.
   (mock has no physics; the Gazebo system is the real sim hardware interface.)

2. **Gazebo plugin** block (was already present) points at the controllers file:
   ```xml
   <gazebo>
     <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
       <parameters>$(find moveit_arm_package)/config/controllers.yaml</parameters>
     </plugin>
   </gazebo>
   ```

3. **Anchor to the world** so the robot doesn't fall/collapse in Gazebo. The
   root link is `robot_base`, and the SRDF already declares a fixed
   `world -> robot_base` virtual joint, but Gazebo needs a real link + joint:
   ```xml
   <link name="world"/>
   <joint name="world_joint" type="fixed">
     <parent link="world"/>
     <child link="robot_base"/>
     <origin xyz="0 0 0.05" rpy="0 0 0"/>
   </joint>
   ```

## 3. Build wiring

- `moveit_arm_package/CMakeLists.txt`: added `config` to the installed dirs:
  `install(DIRECTORY urdf launch rviz config DESTINATION share/${PROJECT_NAME})`
- `moveit_arm_package/package.xml`: added runtime deps:
  `gazebo_ros`, `gazebo_ros2_control`, `controller_manager`,
  `joint_state_broadcaster`, `joint_trajectory_controller`,
  `moveit_configs_utils`, `moveit_ros_move_group`, `moveit_ros_visualization`.

## 4. `moveit_config/config/ros2_controllers.yaml`

Left as **generated** (it is only used by MoveIt's RViz-only `demo.launch.py`
with mock hardware — Gazebo does NOT read it; Gazebo reads
`moveit_arm_package/config/controllers.yaml`). Do not put ROS 1-style
`controller_list` / `joint_state_controller` entries here.

## 5. Launch files (all in `moveit_arm_package/launch/`)

- **`gazebo.launch.py`** — starts Gazebo, `robot_state_publisher` (from
  `arm.urdf.xacro`), spawns the robot, then spawns controllers in order
  (`joint_state_broadcaster` → `arm_controller` + `gripper_controller`) using
  `OnProcessExit` chaining so nothing races the controller_manager.
- **`moveit.launch.py`** — `move_group` + RViz, built from
  `MoveItConfigsBuilder("arm_robot", package_name="moveit_config")`, with
  `use_sim_time: True`. No mock ros2_control node.
- **`run_controller.launch.py`** — runs the custom `arm_controller`
  (`MoveGroupInterface`) node with `robot_description`,
  `robot_description_semantic`, `robot_description_kinematics` injected.
- **`bringup.launch.py`** — includes `gazebo.launch.py` and (after a 5 s
  `TimerAction` delay) `moveit.launch.py`.

---

## How to run

```bash
cd ~/MoveItArm
colcon build --packages-select moveit_arm_package moveit_config
source install/setup.bash

# Terminal 1 — sim + MoveIt + RViz
ros2 launch moveit_arm_package bringup.launch.py

# Terminal 2 — command the arm (after RViz/Gazebo are up)
ros2 launch moveit_arm_package run_controller.launch.py
```

Equivalent split form (Terminal 1 = `gazebo.launch.py`, Terminal 2 =
`moveit.launch.py`, Terminal 3 = `run_controller.launch.py`).

---

## Gotchas / rules

- **Never** `ros2 run moveit_arm_package arm_controller` directly — it has no
  `robot_description_semantic` and crashes parsing the SRDF. Always launch via
  `run_controller.launch.py` (or any launch that injects the MoveIt params).
  Anything using `MoveGroupInterface` needs those params.
- **Do not** run `moveit_config demo.launch.py` alongside Gazebo — it starts a
  *mock* `ros2_control` node that collides with Gazebo's controller_manager and
  executes on fake hardware instead of the sim.
- **Two `ros2_control` blocks exist**: `GazeboSystem` in `arm.urdf.xacro` and
  `FakeSystem` in `moveit_config/arm_robot.ros2_control.xacro`. Load only
  `arm.urdf.xacro` for Gazebo. (move_group loading the moveit_config
  description is fine — it ignores the ros2_control/gazebo tags.)
- If `move_group` starts before the controllers exist (slow machine), increase
  the `TimerAction` `period` in `bringup.launch.py`.
- `use_sim_time: True` everywhere so MoveIt and the Gazebo controllers share
  `/clock` (trajectory timing).

## Verify controllers are up (optional)

```bash
sudo apt install ros-humble-ros2controlcli   # one-time
ros2 control list_controllers               # all three should read 'active'
```
```
joint_state_broadcaster  ... active
arm_controller           ... active
gripper_controller       ... active
```
