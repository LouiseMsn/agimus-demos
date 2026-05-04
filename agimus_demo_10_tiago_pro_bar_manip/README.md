AGIMUS demo 04 dual arm manipulation for TIAGo Pro
--------------------------------------------------

The purpose of this demo is to implement the Agimus case study CS 2.1 (Kleeman).

Tiago pro manipulates a reinforcment bar with two hands in order to glue it to
an elevator ceiling.

### Dependencies

This demo requires source built of dependencies found in:
- [control.repos](../control.repos)
- [tiago-pro.repos] (../tiago-pro.repos)

### Simulation

To be completed

### Real robot

To be completed
## Setup tiago pos (gazebo)
## Launch simulation in Pal docker

`ros2 launch agimus_demos_common tiago_pro_simulation.launch.py tuck_arm:=False end_effector_right:=pal-pro-gripper end_effector_left:=pal-pro-gripper`

# For docker control side don't forget to setup cyclone dds
```bash
export CYCLONEDDS_URI=/home/gepetto/ros2_ws/src/agimus-demos/agimus_demos_common/config/tiago_pro/cyclone_config.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_DOMAIN_ID
```

```bash
ros2 topic pub --once /arm_left_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'arm_left_1_joint','arm_left_2_joint','arm_left_3_joint',
    'arm_left_4_joint','arm_left_5_joint','arm_left_6_joint','arm_left_7_joint'
  ],
  points: [{
    positions: [1.5708, -1.95, -0.35, -2.15, -3.0, 0.6, -1.5708],
    time_from_start: {sec: 3}
  }]
}"\
&& ros2 topic pub --once /arm_right_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'arm_right_1_joint','arm_right_2_joint','arm_right_3_joint',
    'arm_right_4_joint','arm_right_5_joint','arm_right_6_joint','arm_right_7_joint'
  ],
  points: [{
    positions: [-3.127, -1.95, 0.35, -2.15, 3.0, 0.6, -1.5708],
    time_from_start: {sec: 3}
  }]
}"\
&& ros2 topic pub --once /torso_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'torso_lift_joint'
  ],
  points: [{
    positions: [0.2],
    time_from_start: {sec: 3}
  }]
}"
```

## Launch HPP orchestrator
```bash
ros2 launch agimus_demo_10_tiago_pro_bar_manip bringup.launch.py use_sim_time:=True



ros2 run agimus_demo_10_tiago_pro_bar_manip orchestrator_node --ros-args -p weights_config:=src/agimus-demos/agimus_demo_10_tiago_pro_bar_manip/config/ocp/ocp_weights.yaml -p joints_config:=src/agimus-demos/agimus_demo_10_tiago_pro_bar_manip/config/robot/tiago_pro_joints.yaml
```
## Request planning
```bash
ros2 action send_goal /orchestrator/plan_bar_handling test_agimus_type/action/PlanBarGrasp "{task: 'pick', gripper: 'tiago_pro/left', handle: 'reinforcement_bar/left'}"
```

```bash
ros2 topic pub /environment_description std_msgs/msg/String "{data: ""}"
```
