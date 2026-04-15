import os
import copy
import time

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

import pinocchio as pin

from test_agimus_type.action import PlanBarGrasp
from agimus_demo_10_tiago_pro_bar_manip.traj.hpp_traj import (
    BaseObject,
    HPPPathGenerator,
)
from agimus_controller.trajectory import TrajectoryPoint
from agimus_demo_10_tiago_pro_bar_manip.rostools import process_xacro


def _patch_srdf(srdf_str: str) -> str:
    """Inject disabled collision pairs into SRDF string."""
    i = srdf_str.find("</robot>")
    assert i != -1, "SRDF string does not contain </robot>"
    pairs = [
        ("base_link", "wheel_front_left_link"),
        ("base_link", "wheel_front_right_link"),
        ("base_link", "wheel_rear_left_link"),
        ("base_link", "wheel_rear_right_link"),
        ("gripper_left_screw_left_link", "gripper_left_fingertip_left_link"),
        ("gripper_right_screw_left_link", "gripper_right_fingertip_left_link"),
    ]
    insert = "".join(
        f'  <disable_collisions link1="{l1}" link2="{l2}" reason="Never"/>\n'
        for l1, l2 in pairs
    )
    return srdf_str[:i] + insert + "</robot>"

class HPPActionServer(Node):
    def __init__(self):
        super().__init__("hpp_action_server")

        # == Internal state ====================================================
        self._joint_state = None
        self._odom = None
        self._bar_pose = None
        self._plate_pose = None

        # == Planning state =================================================
        self._path = None
        self._planning = False
        self._traj_pick = None
        self._traj_place = None
        self._q_after_grasp = None

        # == OCP-related variables ==============================================
        self._ocp_dt = 0.1

        # == Robot state ====================================================
        self._robot_model = None  
        self._robot_data = None 
        self._nq = None 
        self._nv = None
        self._left_tool_frame_id_name = "gripper_left_grasping_frame_joint"
        self._left_tool_frame_id_pin_frame = None
        self._right_tool_frame_id_name = "gripper_right_grasping_frame_joint"
        self._right_tool_frame_id_pin_frame = None

        self._cb_group = ReentrantCallbackGroup()

        # == HPP init (deferred until /robot_description received) ==========
        self._hpp: HPPPathGenerator | None = None
        self.get_logger().info("Waiting for /robot_description …")

        # == ROS =============================================================
        # == Subscribers
        # /robot_description
        qos_r_descr = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # transient_local so we get the latched value
        )
        self.create_subscription(
            String,
            "/robot_description",
            self._cb_robot_description,
            qos_r_descr,
            callback_group=self._cb_group,
        )

        # /joint_states
        qos_js = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._cb_joints,
            qos_js,
            callback_group=self._cb_group,
        )
        self.create_timer(0.5, self._cb_object_pose, callback_group=self._cb_group)

        # == TF2 ============================================================
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self.create_timer(0.5, self._cb_object_pose, callback_group=self._cb_group)

        # == Action server ==================================================
        self._action_server = ActionServer(
            self,
            PlanBarGrasp,
            "/orchestrator/plan_bar_handling",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

    # == /robot_description callback ========================================

    def _cb_robot_description(self, msg: String):
        if self._hpp is not None:
            return  # already initialized
        
        self._buildRobot(urdf_str=msg.data)
        self.get_logger().info("/robot_description received — building HPP...")
        self._init_hpp(urdf_str=msg.data)
    
    def _buildRobot(self, urdf_str):
        self._robot_model = pin.buildModelFromXML(urdf_str)
        self._robot_data = self._robot_model.createData()
        self._nq = self._robot_model.nq
        self._nv = self._robot_model.nv
        self._left_tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._left_tool_frame_id_name
        )
        self._right_tool_frame_id_pin_frame = self._robot_model.getFrameId(
            self._right_tool_frame_id_name
        )
        self.get_logger().info(f"Pinocchio model built: {self._robot_model.nq} dof")

    def _init_hpp(self, urdf_str):

        if self._robot_model is None:
            raise ValueError("Pinocchio robot model is empty!")

        # SRDF string 
        moveit_pkg = get_package_share_directory("tiago_pro_moveit_config")
        srdf_raw = os.path.join(moveit_pkg, "config", "srdf", "tiago_pro.srdf.xacro")

        srdf_string = process_xacro(
            str(srdf_raw),
            "end_effector_left:=pal-pro-gripper",
            "end_effector_right:=pal-pro-gripper",
        )
        srdf_str = _patch_srdf(srdf_string)

        # Package paths for objects
        pkg = get_package_share_directory("agimus_demo_10_tiago_pro_bar_manip")

        self._hpp = HPPPathGenerator(
            urdf_str=urdf_str,
            srdf_str=srdf_str,
            robot_model=self._robot_model,
            handle_config=os.path.join(
                pkg, "config/planning", "handles_configurations.yaml"
            ),
            handle_object=BaseObject(
                root_joint_type="freeflyer",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/reinforcement_bar.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/reinforcement_bar.srdf",
                name="reinforcement_bar",
            ),
            plate_object=BaseObject(
                root_joint_type="freeflyer",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/plate.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/plate.srdf",
                name="plate",
            ),
            table_object=BaseObject(
                root_joint_type="anchor",
                urdf_path="package://agimus_demo_10_tiago_pro_bar_manip/urdf/table.urdf",
                srdf_path="package://agimus_demo_10_tiago_pro_bar_manip/srdf/table.srdf",
                name="table",
            ),
            ocp_dt=0.1,
            robot_name="tiago_pro",
            logger=self.get_logger(),
        )
        self.get_logger().info("HPPPathGenerator ready — action server active.")

    # == ROS callbacks ======================================================

    def _cb_joints(self, msg):
        self._joint_state = msg

    def _cb_object_pose(self):
        self._bar_pose = self._lookup_pose("table_link", "bar_base_link")
        self._plate_pose = self._lookup_pose("table_link", "plate_base_link")
        self._odom = self._lookup_pose("table_link", "base_footprint")
        self._bar_goal_pose = self._lookup_pose("table_link", "bar_goal_pose")

    # == Action callbacks ===================================================

    def _goal_cb(self, goal_request):

        if self._hpp is None:
            self.get_logger().error("REJECTED: HPP hasn't been initialized, check if /robot_description is published")
            return GoalResponse.REJECT
        if self._planning:
            self.get_logger().warn("REJECTED: already planning")
            return GoalResponse.REJECT

        action_type = goal_request.action_type
        gripper = goal_request.gripper
        handle = goal_request.handle

        self.get_logger().info(
            f"Goal — action_type='{action_type}' gripper='{gripper}' handle='{handle}'"
        )

        if not action_type or not gripper or not handle:
            self.get_logger().error("REJECTED: empty field(s) in action request")
            return GoalResponse.REJECT
        
        match action_type:
            case "pick":
                if self._robot_model is None:
                    self.get_logger().error(
                        "REJECTED: Robot description has not been received yet"
                    )
                    return GoalResponse.REJECT

            case "place":
                if self._q_after_grasp is None:
                    self.get_logger().error("REJECTED: no pick planned")
                    return GoalResponse.REJECT

            case _ :
                self.get_logger().error(f"REJECTED: action_type '{action_type}' not in ['pick','place']")
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):

        self.get_logger().info("Cancel requested.")
        self.get_logger().info(f"{goal_handle.action_type}")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle) -> PlanBarGrasp.Result:
        """Plans the trajectory using HPP

        Args:
            goal_handle (rclpy.action.server.ServerGoalHandle): ros2 action message

        Returns:
            test_agimus_type.action._plan_bar_grasp.PlanBarGrasp_Result: return message
        """

        self._planning = True
        result_msg = PlanBarGrasp.Result()
        action_type = goal_handle.request.action_type
        gripper_name = goal_handle.request.gripper
        handle_name = goal_handle.request.handle

        self.get_logger().info(
            f"Executing '{action_type}' — {gripper_name} / {handle_name}"
        )

        if not self._wait_for_state(timeout=5.0):
            result_msg.success = False
            result_msg.message = "Timeout waiting for robot state."
            goal_handle.abort()
            self._planning = False
            return result_msg

        # Planning
        try:
            match action_type:
                case 'pick':
                    success, message, path_id, traj = self._plan_pick(gripper_name, handle_name)
                    # add traj to buffer
                case 'place':
                    success, message, path_id, traj = self._plan_place(gripper_name, handle_name)
                case _:
                    self.get_logger().error(f"_execute_ plan : action_type '{action_type}' not in ['pick','place']")
        except Exception as e:
            import traceback

            self.get_logger().error(f"Planning exception:\n{traceback.format_exc()}")
            success, message, path_id = False, str(e), -1

        # Running

        result_msg.success = success
        result_msg.message = message
        result_msg.path_id = path_id

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._planning = False
        return result_msg

    # == Planning helpers ===================================================

    def _plan_pick(self, gripper, handle):
        q_init = self._build_q_init()
        if q_init is None:
            return False, "Failed to build q_init.", -1

        traj, q_end = self._hpp.plan_pick(
            gripper=gripper, handle=handle, q_init=q_init
        )
        if traj is None:
            return False, "Grasp planning failed.", -1

        self._q_after_grasp = q_end
        path_id = self._hpp._ps.numberPaths() - 1
        self.get_logger().info(f"Grasp planned sucessfully (path_id={path_id})")
        return True, "Grasp planned successfully.", path_id, traj

    def _plan_place(self, gripper, handle):
        q_init_place = list(self._q_after_grasp)  # copy
        r = self._hpp.robot.rankInConfiguration["tiago_pro/root_joint"]
        q_init_place[r] = 3.0

        target_bar_pose = self._bar_goal_pose

        traj, _ = self._hpp.plan_place(
            gripper=gripper,
            handle=handle,
            q_init=q_init_place,
            target_bar_pose=target_bar_pose,
        )
        if traj is None:
            return False, "Place planning failed.", -1

        path_id = self._hpp._ps.numberPaths() - 1
        self.get_logger().info(f"Place planned (path_id={path_id})")
        return True, "Place planned successfully.", path_id ,traj

    # Running helpers:
    def _publish_mpc_input_cb(self) -> None:
        # Skip if buffer is empty
        with self._trajectory_buffer_lock:
            self._buffer_len = len(self._trajectory_buffer)
        if not self._buffer_len:
            return

        if self._buffer_size >= self._params.ocp_buffer_size:
            return

        def _get_traj_point() -> WeightedTrajectoryPoint:
            if len(self._trajectory_buffer) == 1:
                return self._trajectory_buffer[0]
            else:
                return self._trajectory_buffer.popleft()

        n_points = self._params.ocp_buffer_size - self._buffer_size # how to define ocp buffer ?
        with self._trajectory_buffer_lock:
            mpc_input_array = MpcInputArray(
                inputs=[
                    weighted_traj_point_to_mpc_msg(_get_traj_point())
                    for _ in range(n_points)
                ]
            )
        if self._params.visualization.publish_path:
            self._mpc_target_pose.publish(
                PoseStamped(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id="fer_link0",
                    ),
                    pose=mpc_input_array.inputs[-1].ee_inputs[0].pose,
                )
            )

        self._mpc_input_pub.publish(mpc_input_array)
    # == State helpers ======================================================

    def _wait_for_state(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if all(
                m is not None
                for m in [
                    self._joint_state,
                    self._odom,
                    self._bar_pose,
                    self._plate_pose,
                    self._bar_goal_pose,
                ]
            ):
                return True
            time.sleep(0.05)
        return False

    def _lookup_pose(self, parent_frame, child_frame):
        try:
            t = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            quat = np.array([ro.x, ro.y, ro.z, ro.w])
            quat /= np.linalg.norm(quat)
            return [tr.x, tr.y, tr.z, quat[0], quat[1], quat[2], quat[3]]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed for {child_frame}: {e}")
            return None

    def _tf_to_base_odom(self, tf):
        x, y = tf[0], tf[1]
        qx, qy, qz, qw = tf[3], tf[4], tf[5], tf[6]
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        return [x, y, np.cos(np.arctan2(siny, cosy)), np.sin(np.arctan2(siny, cosy))]

    def _build_q_init(self):
        if any(
            m is None
            for m in [self._joint_state, self._odom, self._bar_pose, self._plate_pose]
        ):
            return None
        robot = self._hpp.robot
        q = robot.getCurrentConfig()
        for i, joint_name in enumerate(self._joint_state.name):
            full = f"tiago_pro/{joint_name}"
            if full in robot.jointNames:
                rank = robot.rankInConfiguration[full]
                value = self._joint_state.position[i]
                nq = robot.getJointConfigSize(full)
                if nq == 2:
                    q[rank] = np.cos(value)
                    q[rank + 1] = np.sin(value)
                elif nq == 1:
                    q[rank] = value
        r = robot.rankInConfiguration["tiago_pro/root_joint"]
        q[r : r + 4] = self._tf_to_base_odom(self._odom)
        r_bar = robot.rankInConfiguration["reinforcement_bar/root_joint"]
        q[r_bar : r_bar + 7] = self._bar_pose
        r_plate = robot.rankInConfiguration["plate/root_joint"]
        q[r_plate : r_plate + 7] = self._plate_pose
        return q

    # == Path helpers =========================================================
    def _convert_path(self, raw_path):
        """Converts the hpp path into an agimus-controller compatible one

        Args:
            raw_path (hpp_idl.hpp.core_idl._objref_PathVector): path as an HPP object

        Returns:
            [TrajectoryPoint]: trajectory converted in a list of TrajectoryPoint
        """

        while self._robot_model is None:
            self.get_logger().info("No robot descr yet")
            time.sleep(0.5)

        length = raw_path.length()
        slowdown = 4
        n_traj_points = int(np.ceil(length / self._ocp_dt)) * slowdown
        trajectory = np.array(
            [
                raw_path.call(i * self._ocp_dt / slowdown)[0][: self._nq]
                for i in range(n_traj_points)
            ]
        )
        raw_path.deleteThis()  # ?ss

        converted_trajectory = [
            self._convert_point(i, trajectory) for i in range(n_traj_points)
        ]

        return converted_trajectory

    def _convert_point(self, i: int, trajectory: list) -> TrajectoryPoint:
        """Converts a point from HPP to agimus controller format

        Args:
            i (int): id of the point wanted
            trajectory (list of np.arrays): list of HPP points

        Returns:
            TrajectoryPoint: point in the agimus-controller format
        """
        q = trajectory[i, :]
        pin.framesForwardKinematics(self._robot_model, self._robot_data, q)

        return TrajectoryPoint(
            id=i,
            time_ns=0,
            robot_configuration=q,
            robot_velocity=np.zeros_like(q),
            robot_acceleration=np.zeros(self._nv),
            robot_effort=np.zeros(self._nv),
            forces={},
            end_effector_poses={
                self._left_tool_frame_id_name: copy.copy(
                    self._robot_data.oMf[self._left_tool_frame_id_pin_frame]
                ),
                self._right_tool_frame_id_name: copy.copy(
                    self._robot_data.oMf[self._right_tool_frame_id_pin_frame]
                ),
            },
        )


# == Entrypoint ==============================================================
def main():
    rclpy.init()
    node = HPPActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
