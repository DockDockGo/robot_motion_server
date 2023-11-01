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
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import time
from threading import Event

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
        self.callback_group = ReentrantCallbackGroup()
        self.action_complete = Event()

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

        self.ddg_waypoint_follower = self.create_client(srv_type=DdgExecuteWaypoints,
                                                        srv_name=waypoint_follower_service_name)

        while not self.ddg_waypoint_follower.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("DDG Waypoint Follower Service not available, waiting...")

        self.timer = self.create_timer(1, self.timer_callback)
        self.distance_to_goal = None

        self.get_logger().info("DDG Waypoint Follower Service is now active")
        self.ddg_waypoint_follower_path = DdgExecuteWaypoints.Request()
        self.navigator_final_result = None
        self.robot_pose = None
        self.goal_pose = None

        # Pose Subscribers
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            subscriber_topic,  # Topic on which pose is being relayed
            self.robot_pose_callback,
            10  # Adjust the queue size as needed
        )

        # construct the action server
        self._action_server = ActionServer(
            self,
            Navigate,
            action_server_name,
            self.execute_callback,
            callback_group=self.callback_group)


    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        # self.get_logger().info(str(type(msg)))

    def send_goal_to_waypoint_follower(self, goal):
        waypoint_future = self.ddg_waypoint_follower.call_async(self.ddg_waypoint_follower_path)
        rclpy.spin_until_future_complete(self, waypoint_future)
        return waypoint_future

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
        self.get_logger().info(f"distance to goal is {distance}")

        return distance

    def timer_callback(self):
        self.distance_to_goal = self.euclidean_distance()
        if self.distance_to_goal is not None and self.distance_to_goal < 0.2:
            self.action_complete.set()


    def execute_callback(self, goal_handle):
        """
        Callback of action server with goal_handle.request having input to action server
        """
        self.action_complete.clear()
        route = goal_handle.request.goals
        self.goal_pose = route[-1]
        feedback_msg = Navigate.Feedback()
        routh_path = Path()
        routh_path.poses = route
        self.get_logger().info(f"Goals to waypoint follower are {str(routh_path)}")

        self.ddg_waypoint_follower_path.waypoints = routh_path
        self.get_logger().info("Going to send goal to waypoint follower")
        waypoint_future = self.send_goal_to_waypoint_follower(routh_path)

        if waypoint_future.done(): #! .done() should be returned only after completion, this is for debugging only
            self.get_logger().info(f"goal accepted \n {waypoint_future.result().success}")

        self.action_complete.wait()

        motion_server_result = Navigate.Result()
        if waypoint_future.result() is not None:
            self.get_logger().info(f"outcome is {waypoint_future.result()}")
            motion_server_result.success = True
        else:
            motion_server_result.success = False

        goal_handle.succeed()
        return motion_server_result

        #NOTE: Set init pose skipped for now because we may use 3D localization
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 3.45
        # initial_pose.pose.position.y = 2.15
        # initial_pose.pose.orientation.z = 1.0
        # initial_pose.pose.orientation.w = 0.0
        # navigator.setInitialPose(initial_pose)

        # Wait for navigation to activate fully
        # self.navigator.waitUntilNav2Active()

        # route_poses = []
        # pose = PoseStamped()
        # pose.header.frame_id = 'map'
        # pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # for pt in route:
        #     pose.pose.position.x = pt.pose.position.x
        #     pose.pose.position.y = pt.pose.position.y
        #     self.get_logger().info(f"goal pos x = {pt.pose.position.x}")
        #     self.get_logger().info(f"goal pos y = {pt.pose.position.y}")
        #     pose.pose.position.z = 0.0
        #     pose.pose.orientation.z = pt.pose.orientation.z
        #     pose.pose.orientation.w = pt.pose.orientation.w
        #     route_poses.append(deepcopy(pose))


        # self.get_logger().info(f"THE NUMBER OF WAYPOINTS GIVEN TO GO THROUGH POSES IS {str(len(route_poses))}")
        # self.navigator.goThroughPoses(route_poses)

        # i = 0
        # while not self.navigator.isTaskComplete():
        #     i = i + 1
        #     feedback = self.navigator.getFeedback()
        #     if feedback and i % 5 == 0:
        #         self.get_logger().debug('Estimated time to complete current route: ' + '{0:.0f}'.format(
        #             Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
        #             + ' seconds.')
        #         # publish feedback
        #         feedback_msg.pose_feedback = self.robot_pose
        #         goal_handle.publish_feedback(feedback_msg)

        #         # Some failure mode, must stop since the robot is clearly stuck
        #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
        #             self.get_logger().warning('Navigation has exceeded timeout of 180s, canceling the request.')
        #             self.navigator.cancelTask()

        # result = self.navigator.getResult()
        # self.get_logger().info(str(result))

        # # ! Some issue with trying to check just result status
        # if str(result) == "TaskResult.SUCCEEDED":
        #     self.get_logger().info('Route complete!')
        #     self.navigator_final_result = "success"
        # elif str(result) == "TaskResult.CANCELED":
        #     self.get_logger().info('route was canceled, exiting.')
        #     self.navigator_final_result = "fail"
        # elif str(result) == "TaskResult.FAILED":
        #     self.get_logger().info('route failed!')
        #     self.navigator_final_result = "fail"


        # motion_server_result = Navigate.Result()
        # if self.navigator_final_result == "success":
        #     motion_server_result.success = True
        # else:
        #     motion_server_result.success = False

        # goal_handle.succeed()

        # return motion_server_result


def MotionServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)

    # start the MotionActionServer
    # robot_motion_action_server = MotionActionServer(nav2_client, get_pose_client)
    executor = MultiThreadedExecutor()
    robot_motion_action_server = MotionActionServer()
    rclpy.spin(robot_motion_action_server, executor)


def DockUndockServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)
    # start the MotionActionServer
    # robot_motion_action_server = MotionActionServer(nav2_client, get_pose_client)
    robot_dockundock_action_server = DockingUndockingActionServer()
    rclpy.spin(robot_dockundock_action_server)


if __name__ == '__main__':
    MotionServer()
    DockUndockServer()