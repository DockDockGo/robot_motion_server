import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from robot_action_interfaces.action import DockUndock
from robot_action_interfaces.srv import GetRobotPose
from ddg_multi_robot_srvs.srv import DdgExecuteWaypoints
from robot_action_interfaces.action import Navigate
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Pose
from nav_msgs.msg import Odometry
from copy import deepcopy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import robot_motion_server.custom_navigator as custom_nav
import robot_motion_server.custom_navigator_multi_robot as custom_nav_multi_robot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import math
import time
from threading import Event
from tf_transformations import euler_from_quaternion
from fiducial_msgs.msg import FiducialMarkerData

# Handling Sync and Async : https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255
#                         : https://gist.github.com/driftregion/14f6da05a71a57ef0804b68e17b06de5


class DockingUndockingActionServer(Node):

    def __init__(self):
        #! TODO: Add namespace
        super().__init__('docking_undocking_action_server')
        self.get_logger().info("Starting Docking Undocking Action Server")

        self.declare_parameter('namespace_param', '')
        robot_namespace = self.get_parameter('namespace_param').get_parameter_value().string_value
        self.get_logger().info(f"namespace is {robot_namespace}")

        self.callback_group = ReentrantCallbackGroup()

        if robot_namespace != '':
            action_server_name = "/" + robot_namespace + "/" + "DockUndock"
            publisher_topic = "/" + robot_namespace + "/cmd_vel"
            subscriber_topic = "/" + robot_namespace + "/map_pose"
        else:
            action_server_name = "DockUndock"
            publisher_topic = "/cmd_vel"
            subscriber_topic = "/map_pose"

        self.get_logger().info(f"cmd vel topic is {publisher_topic}")
        self.robot_pose = None
        self.fiducial_range = None
        self.fiducial_orientation = None
        self.fiducial_lateral_offset = None
        self.fiducial_marker_id = None
        self.goal_pose = None
        self.dock_id = None
        self.goal_lateral_threshold = 0.05 # metres
        self.robot_to_fiducial_goal_distance = 0.75 # metres
        self.undock_duration = 1

        # define biases to be used for docking
        self.x_bias = 0.0
        self.y_bias = 0.0

        # construct the action server
        self._action_server = ActionServer(
            self,
            DockUndock,
            action_server_name,
            self.execute_callback,
            callback_group=self.callback_group)

        self.publisher_ = self.create_publisher(Twist, publisher_topic, 10)

        # Pose Subscriber
        self.pose_subscription = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic=subscriber_topic,  # Topic on which pose is being relayed
            callback=self.robot_pose_callback,
            qos_profile=10,  # Adjust the queue size as needed
        )

        # Fiducial Subscriber
        self.fiducial_subscription = self.create_subscription(
            msg_type=FiducialMarkerData,
            topic="/fiducial_marker_data",
            callback=self.fiducial_orientation_range_callback,
            qos_profile=10,  # Adjust the queue size as needed
        )

    def define_goal_pose(self, dx, dy):
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = self.robot_pose.pose.pose.position.x + dx
        self.goal_pose.pose.position.y = self.robot_pose.pose.pose.position.y + dy
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = self.robot_pose.pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = self.robot_pose.pose.pose.orientation.w

    def robot_pose_callback(self, msg : PoseWithCovarianceStamped):
        self.robot_pose = msg

    def fiducial_orientation_range_callback(self, msg):
        self.fiducial_marker_id = int(msg.marker_frame_id[-1])
        if self.dock_id is None:
            self.fiducial_lateral_offset = msg.lateral_offset + self.y_bias
            self.fiducial_range = msg.range
            self.fiducial_orientation = msg.yaw
        # verification check to ensure we only use pose from correct DockID
        elif (self.dock_id is not None) and (int(self.dock_id) == self.fiducial_marker_id):
            self.get_logger().info("got matched dock ID")
            self.fiducial_lateral_offset = msg.lateral_offset + self.y_bias
            self.fiducial_range = msg.range
            self.fiducial_orientation = msg.yaw
        else:
            self.get_logger().info(f"got wrong fiducial ID {self.fiducial_marker_id}")

    def publish_zero_twist(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def publish_action_server_result(self, flag, goal_handle):
        goal_handle.succeed()
        result = DockUndock.Result()
        result.success = flag
        return result

    def wrap_around_pi(self, x):
        x = abs(x)
        pi = 3.1415926
        if x < pi:
            return x
        return abs((x + 2 * pi) % pi - pi)

    def radians_to_degrees(self, angle):
        return angle * 180.0 / 3.1415926

    def calc_dtheta(self):
        pose1 = self.goal_pose
        pose2 = self.robot_pose

        # Extract the positions from the poses
        if isinstance(pose1, PoseStamped) and isinstance(pose2, PoseWithCovarianceStamped):
            pos1 = pose1.pose
            pos2 = pose2.pose.pose
        elif pose1 is None:
            return None
        else:
            raise ValueError("Input pose1 is not a valid type (Pose or PoseWithCovariance)")

        # Convert orientations to Euler angles (roll, pitch, and yaw)
        euler_angles1 = euler_from_quaternion([pos1.orientation.x, pos1.orientation.y, pos1.orientation.z, pos1.orientation.w])
        euler_angles2 = euler_from_quaternion([pos2.orientation.x, pos2.orientation.y, pos2.orientation.z, pos2.orientation.w])
        # Calculate the orientation difference
        dtheta = self.radians_to_degrees(self.wrap_around_pi(euler_angles2[2] - euler_angles1[2]))
        return dtheta # dtheta in degrees

    def calc_euclidean_dist(self):
        pose1 = self.goal_pose
        pose2 = self.robot_pose

        # Extract the positions from the poses
        if isinstance(pose1, PoseStamped) and isinstance(pose2, PoseWithCovarianceStamped):
            pos1 = pose1.pose
            pos2 = pose2.pose.pose
        elif pose1 is None:
            return None
        else:
            raise ValueError("Input pose1 is not a valid type (Pose or PoseWithCovariance)")
        ex = pos1.position.x - pos2.position.x
        ey = pos1.position.y - pos2.position.y

        return ex, ey


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Docking/Undocking...')
        result = DockUndock.Result()

        # Define and fill the feedback message
        # feedback_msg = DockUndock.Feedback()
        # feedback_msg.pose_feedback = None
        # goal_handle.publish_feedback(feedback_msg)

        # Get current euclidean distance to goal
        if self.robot_pose is None:
            self.get_logger().error("Map Pose Not found")

        self.dock_id = int(abs(goal_handle.request.secs))
        self.get_logger().info(f"Docking Goal is {goal_handle.request.secs}")

        ################## Simple Undocking ##################
        if goal_handle.request.secs > 0.0:
            msg = Twist()
            msg.linear.x = 0.2
            for i in range(0, self.undock_duration):
                self.get_logger().info("Undocking in Progress")
                self.publisher_.publish(msg)
                time.sleep(1)

            self.get_logger().info("FINISHED UNDOCKING")
            result.success = True
            goal_handle.succeed()
            return result

        ################## Fiducial Docking ##################

        # Step1: Align Pose to fiducial
        while(abs(self.fiducial_orientation) > 3): # NOTE: self.fiducial_orientation in degrees
            rotation_direction = math.copysign(1, self.fiducial_orientation)
            self.get_logger().info(f"Docking_1 in Progress with orientation {self.fiducial_orientation}")
            msg = Twist()
            msg.angular.z = 0.1 * -1 * rotation_direction
            self.publisher_.publish(msg)
            time.sleep(0.02)

        self.publish_zero_twist()

        # Step2: Calculate the error in pre_dock and use that to set new goal_pose
        dy = self.fiducial_lateral_offset
        dx = 0.0
        self.define_goal_pose(dx, dy) #! Fix your goal pose, something wrong here

        # Step3: Turn robot 90 degrees depending on which side of the fiducial we are located
        rotation_direction_1 = math.copysign(1, dy)
        rotation_direction_2 = -1 * math.copysign(1, dy)
        dtheta = self.calc_dtheta()
        while(not (dtheta > 89 and dtheta < 93)):
            self.get_logger().info(f"Docking_3 in Progress with dtheta {dtheta}")
            msg = Twist()
            msg.angular.z = 0.1 * rotation_direction_1
            self.publisher_.publish(msg)
            time.sleep(0.02)
            dtheta = self.calc_dtheta()

        self.publish_zero_twist()

        # Step4: Move to goal pose
        error_x, error_y = self.calc_euclidean_dist()
        self.get_logger().info(f"Docking_4 Going to start with dx {error_x} and dy {error_y}")
        while(abs(error_y) > self.goal_lateral_threshold):
            self.get_logger().info(f"Docking_4 in Progress with dx {error_x} and dy {error_y}")
            msg = Twist()
            msg.linear.x = 0.05
            self.publisher_.publish(msg)
            time.sleep(0.02)
            error_x, error_y = self.calc_euclidean_dist()

        self.publish_zero_twist()

        # Step5: Use the rotation_direction of Step3
        dtheta = self.calc_dtheta()
        self.get_logger().info(f"Docking_5 Going to start with dtheta {dtheta}")
        while(dtheta < 2):
            msg = Twist()
            msg.angular.z = 0.1 * rotation_direction_2
            self.publisher_.publish(msg)
            time.sleep(0.1)
            dtheta = self.calc_dtheta()

        self.publish_zero_twist()

        # Step6: Move robot backwards until fiducial range is minimized
        self.get_logger().info(f"Docking_6 Going to start with fiducial range={self.fiducial_range}")
        while(self.fiducial_range > abs(self.robot_to_fiducial_goal_distance)):
            msg = Twist()
            msg.linear.x = -0.2
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # Step7: Verify if goal pose has been reached
        if self.fiducial_lateral_offset < 0.05 and abs(self.fiducial_range - self.robot_to_fiducial_goal_distance) < 0.05:
            self.get_logger().info(f"Successfully doced with fiducial lateral offset at {self.fiducial_lateral_offset} and fiducial range at {self.fiducial_range}")
        else:
            self.get_logger().info(f"Bad Docking with fiducial lateral offset at {self.fiducial_lateral_offset} and fiducial range at {self.fiducial_range}")

        # while(True):
        #     # dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
        #     # robot_orientation_in_world_frame = self.world_frame_orientation()
        #     goal_orientation, robot_orientation = self.get_individual_orientations(self.goal_pose)
        #     self.get_logger().info(f"goal orientation is {goal_orientation[2]} and robot orientation is {robot_orientation[2]}")
        #     # self.get_logger().info(f"orientation in world frame is {robot_orientation_in_world_frame}")
        #     self.get_logger().info(f"axis is {dock_axis}")
        #     self.get_logger().info(f"dx is {dx} and dy is {dy}")
        #     self.get_logger().info(f"rotation direction {rotation_direction}")
        #     time.sleep(10)

        msg = Twist()
        self.publisher_.publish(msg)
        self.get_logger().info("FINISHED DOCKING")
        result = DockUndock.Result()
        result.success = True

        goal_handle.succeed()

        return result


class MotionActionServer(Node):

    def __init__(self):
        namespace = ""
        super().__init__('motion_action_server')
        self.get_logger().info("Starting Motion Action Server")

        self.declare_parameter('namespace_param', '')
        robot_namespace = self.get_parameter('namespace_param').get_parameter_value().string_value
        self.get_logger().info(f"namespace is {robot_namespace}")

        if robot_namespace != '':
            action_server_name = "/" + robot_namespace + "/" + "Navigate"
            self.navigator = custom_nav_multi_robot.CustomNavigator(namespace=robot_namespace)
            subscriber_topic = "/" + robot_namespace + "/map_pose"
            waypoint_follower_service_name = "/" + robot_namespace  + "/" + "ddg_navigate_through_poses"
        else:
            action_server_name = "Navigate"
            self.navigator = custom_nav.CustomNavigator()
            subscriber_topic = "/map_pose"
            waypoint_follower_service_name = "/" + "ddg_navigate_through_poses"

        self.get_logger().info(f"map pose subsriber topic is {subscriber_topic}")
        self.get_logger().info(f"DDG Waypoint follower Client name is {waypoint_follower_service_name}")

        self.callback_group = ReentrantCallbackGroup()
        self.motion_server_goal_handle = None

        self.ddg_waypoint_follower = self.create_client(srv_type=DdgExecuteWaypoints,
                                                        srv_name=waypoint_follower_service_name,
                                                        callback_group=self.callback_group)

        while not self.ddg_waypoint_follower.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("DDG Waypoint Follower Service not available, waiting...")

        self.distance_to_goal = None
        self.orientation_difference = None

        self.get_logger().info("DDG Waypoint Follower Service is now active")
        self.ddg_waypoint_follower_path = DdgExecuteWaypoints.Request()
        self.navigator_final_success = False
        self.robot_pose = None
        self.goal_pose = None

        self.action_complete = Event()
        self.action_complete.clear()

        # Pose Subscribers
        self.pose_subscription = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic=subscriber_topic,  # Topic on which pose is being relayed
            callback=self.robot_pose_callback,
            qos_profile=10,  # Adjust the queue size as needed
        )

        # construct the action server
        self._action_server = ActionServer(
            self,
            Navigate,
            action_server_name,
            self.execute_callback,
            callback_group=self.callback_group)

        # Create a simple timer callback within a different callback group
        self.timer = self.create_timer(0.5, callback_group=self.callback_group, callback=self.simple_timer_callback)


    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def wrap_around_pi(self, x):
        x = abs(x)
        pi = 3.1415926
        if x < pi:
            return x
        return abs((x + 2 * pi) % pi - pi)

    def radians_to_degrees(self, angle):
        return angle * 180.0 / 3.1415926

    def euclidean_distance(self):
        pose1 = self.goal_pose
        pose2 = self.robot_pose
        # Extract the positions from the poses
        if isinstance(pose1, PoseStamped) and isinstance(pose2, PoseWithCovarianceStamped):
            pos1 = pose1.pose
            pos2 = pose2.pose.pose
        elif pose1 is None: # in Navigation State
            return None
        else:
            raise ValueError("Input pose1 is not a valid type (Pose or PoseWithCovariance)")

        # Calculate the Euclidean distance
        dx = pos1.position.x - pos2.position.x
        dy = pos1.position.y - pos2.position.y
        dz = pos1.position.z - pos2.position.z

        # Convert orientations to Euler angles (roll, pitch, and yaw)
        euler_angles1 = euler_from_quaternion([pos1.orientation.x, pos1.orientation.y, pos1.orientation.z, pos1.orientation.w])
        euler_angles2 = euler_from_quaternion([pos2.orientation.x, pos2.orientation.y, pos2.orientation.z, pos2.orientation.w])

        # Calculate the orientation difference
        self.orientation_difference = self.radians_to_degrees(self.wrap_around_pi(
            euler_angles2[2] - euler_angles1[2]
        ))

        distance = math.sqrt(dx**2 + dy**2)
        self.distance_to_goal = distance

    def simple_timer_callback(self):
        # check if robot within goal radius
        self.euclidean_distance()

        # Publish Feedback:
        if self.distance_to_goal is not None:
            feedback_msg = Navigate.Feedback()
            feedback_msg.pose_feedback = self.robot_pose
            self.motion_server_goal_handle.publish_feedback(feedback_msg)

        # distance in metres and orientation in degrees
        # self.get_logger().info(f'{self.distance_to_goal=}')
        # self.get_logger().info(f'{self.orientation_difference=}')
        if self.distance_to_goal is not None and self.distance_to_goal < 0.1 and self.orientation_difference is not None and self.orientation_difference < 5.0:
            # self.get_logger().info("Setting navigation to COMPLETE!")
            self.navigator_final_success = True
            self.action_complete.set()


    def waypoint_follower_callback(self, future):
        if future.done():
            self.get_logger().info(f"goal accepted status {future.result().success}")


    def execute_callback(self, goal_handle):
        """
        Callback of action server with goal_handle.request having input to action server
        """
        self.motion_server_goal_handle = goal_handle
        route = goal_handle.request.goals
        self.goal_pose = route[-1]
        routh_path = Path()
        routh_path.poses = route
        self.get_logger().info(f"Goals to waypoint follower are {str(routh_path)}")

        self.ddg_waypoint_follower_path.waypoints = routh_path
        self.action_complete.clear()
        self.get_logger().info("Going to send goal to waypoint follower")
        waypoint_future = self.ddg_waypoint_follower.call_async(self.ddg_waypoint_follower_path)
        waypoint_future.add_done_callback(self.waypoint_follower_callback)

        self.action_complete.wait()
        motion_server_result = Navigate.Result()
        goal_handle.succeed()

        if self.navigator_final_success:
            self.get_logger().info(f"Motion Server outcome is {waypoint_future.result()}")
            motion_server_result.success = True
        else:
            motion_server_result.success = False
            pass

        self.get_logger().info(f"RETURNING MOTION SERVER RESULT {waypoint_future.result()}")
        return motion_server_result


def MotionServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)

    # start the MotionActionServer
    multi_executor = MultiThreadedExecutor()
    robot_motion_action_server = MotionActionServer()
    rclpy.spin(robot_motion_action_server, executor=multi_executor)
    robot_motion_action_server.destroy_node()
    rclpy.shutdown()


def DockUndockServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)
    # start the MotionActionServer
    multi_executor_2 = MultiThreadedExecutor()
    robot_dockundock_action_server = DockingUndockingActionServer()
    rclpy.spin(robot_dockundock_action_server, executor=multi_executor_2)
    robot_dockundock_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    MotionServer()
    DockUndockServer()