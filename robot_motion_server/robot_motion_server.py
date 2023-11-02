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

# Handling Sync and Async : https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255


class DockingUndockingActionServer(Node):

    def __init__(self):
        #! TODO: Add namespace
        super().__init__('docking_undocking_action_server')
        self.get_logger().info("Starting Docking Undocking Action Server")

        self.declare_parameter('namespace_param', '')
        robot_namespace = self.get_parameter('namespace_param').get_parameter_value().string_value
        self.get_logger().info(f"namespace is {robot_namespace}")

        if robot_namespace != '':
            action_server_name = robot_namespace + "/" + "DockUndock"
            publisher_topic = "/" + robot_namespace + "/cmd_vel"
        else:
            action_server_name = "DockUndock"
            publisher_topic = "/cmd_vel"

        self.get_logger().info(f"cmd vel topic is {publisher_topic}")

        # construct the action server
        self._action_server = ActionServer(
            self,
            DockUndock,
            action_server_name,
            self.execute_callback)
        self.publisher_ = self.create_publisher(Twist, publisher_topic, 10)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Docking/Undocking...')

        msg = Twist()
        msg.linear.x = 0.2

        # Define and fill the feedback message
        # feedback_msg = DockUndock.Feedback()
        # feedback_msg.pose_feedback = None
        # goal_handle.publish_feedback(feedback_msg)

        goal_duration = math.ceil(abs(goal_handle.request.secs))
        self.get_logger().info(f"Docking Goal Duration is {goal_duration}")
        if goal_handle.request.secs < 0:
            msg.linear.x = -1 * msg.linear.x

        for i in range(0, goal_duration):
            self.get_logger().info("Docking/Undocking in Progress")
            self.publisher_.publish(msg)
            time.sleep(1)

        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("FINISHED DOCKING")

        result = DockUndock.Result()
        result.success = True

        goal_handle.succeed()

        return result



class MotionActionServer(Node):

    def __init__(self):
        #! TODO: Add namespace
        namespace = ""
        super().__init__('motion_action_server')
        self.get_logger().info("Starting Motion Action Server")

        self.declare_parameter('namespace_param', '')
        robot_namespace = self.get_parameter('namespace_param').get_parameter_value().string_value
        self.get_logger().info(f"namespace is {robot_namespace}")

        if robot_namespace != '':
            action_server_name = robot_namespace + "/" + "Navigate"
            prefixed_namespace = "/" + robot_namespace #! kept for debugging
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
        # self.get_logger().info(f"Got Robot Pose")

    def euclidean_distance(self):
        pose1 = self.goal_pose
        pose2 = self.robot_pose
        # Extract the positions from the poses
        if isinstance(pose1, PoseStamped) and isinstance(pose2, PoseWithCovarianceStamped):
            pos1 = pose1.pose.position
            pos2 = pose2.pose.pose.position
        elif pose1 is None: # in Navigation State
            return None
        else:
            raise ValueError("Input pose1 is not a valid type (Pose or PoseWithCovariance)")

        # Calculate the Euclidean distance
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        # self.get_logger().info(f"distance to goal is {distance}")

        self.distance_to_goal = distance

    def simple_timer_callback(self):
        # check if robot within goal radius
        self.euclidean_distance()

        # Publish Feedback:
        if self.distance_to_goal is not None:
            feedback_msg = Navigate.Feedback()
            feedback_msg.pose_feedback = self.robot_pose
            self.motion_server_goal_handle.publish_feedback(feedback_msg)

        # self.get_logger().info(f"distance to goal is {self.distance_to_goal}")
        if self.distance_to_goal is not None and self.distance_to_goal < 0.2:
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
    # robot_motion_action_server = MotionActionServer(nav2_client, get_pose_client)
    robot_dockundock_action_server = DockingUndockingActionServer()
    rclpy.spin(robot_dockundock_action_server)
    robot_dockundock_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    MotionServer()
    DockUndockServer()