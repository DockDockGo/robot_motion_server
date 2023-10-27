import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from robot_action_interfaces.action import DockUndock
from robot_action_interfaces.srv import GetRobotPose
from robot_action_interfaces.action import Navigate
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from copy import deepcopy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import robot_motion_server.custom_navigator as custom_nav
import math
import time

class DockingUndockingActionServer(Node):

    def __init__(self):
        #! TODO: Add namespace
        super().__init__('docking_undocking_action_server')
        self.get_logger().info("Starting Docking Undocking Action Server")

        # accept client node instances
        self._robot_pose_client = self.create_client(GetRobotPose, 'get_robot_pose')
        self.pose_request = GetRobotPose.Request()
        self.pose_request.frame_id = ''

        # construct the action server
        self._action_server = ActionServer(
            self,
            DockUndock,
            'DockUndock',
            self.execute_callback)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Docking/Undocking...')

        msg = Twist()
        msg.linear.x = -0.2

        # Define and fill the feedback message
        feedback_msg = DockUndock.Feedback()

        for i in range(0, math.ceil(goal_handle.request.secs)):
            self.get_logger().info("Docking/Undocking in Progress")
            self.publisher_.publish(msg)
            time.sleep(1)

        """
        Fix the below get_pose_future client
        """
        # get_pose_future = self._robot_pose_client.call_async(self.pose_request)
        # rclpy.spin_until_future_complete(self, get_pose_future)

        # time.sleep(1)

        # if get_pose_future.result() is not None:
        #     # NOTE: when on server side, it's DockUndock.Feedback().pose_feedback
        #     # NOTE: on client side it's, feedback_msg.feedback.pose_feedback
        #     feedback_msg.pose_feedback = get_pose_future.result().robot_pose
        #     goal_handle.publish_feedback(feedback_msg)

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

        self.navigator = custom_nav.CustomNavigator()
        self.navigator_final_result = None

        # Pose Clients
        self._robot_pose_client = self.create_client(GetRobotPose, 'get_robot_pose')
        self.pose_request = GetRobotPose.Request()

        # construct the action server
        self._action_server = ActionServer(
            self,
            Navigate,
            'Navigate',
            self.execute_callback)


    def execute_callback(self, goal_handle):
        """
        Callback of action server with goal_handle.request having input to action server
        """

        # replace security_route with goal_handle.request.secs
        security_route = [[-0.7, 0.4, 0]]

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
        self.navigator.waitUntilNav2Active()

        # Send your route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))

        self.navigator.goThroughPoses(route_poses)

        # Print ETA for the demonstration
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time to complete current route: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    self.get_logger().info('Navigation has exceeded timeout of 180s, canceling the request.')
                    self.navigator.cancelTask()

        time.sleep(2)
        result = self.navigator.getResult()
        self.get_logger().info(str(result))

        # ! Some issue with trying to check just result status
        if str(result) == "TaskResult.SUCCEEDED":
            self.get_logger().info('Route complete!')
            self.navigator_final_result = "success"
        elif str(result) == "TaskResult.CANCELED":
            self.get_logger().info('route was canceled, exiting.')
            self.navigator_final_result = "fail"
        elif str(result) == "TaskResult.FAILED":
            self.get_logger().info('route failed!')
            self.navigator_final_result = "fail"


        motion_server_result = Navigate.Result()
        if self.navigator_final_result == "success":
            motion_server_result.success = True
        else:
            motion_server_result.success = False

        goal_handle.succeed()

        return motion_server_result


def MotionServer(args=None):
    rclpy.init(args=args)
    print("ARGS IS", args)

    # start the MotionActionServer
    # robot_motion_action_server = MotionActionServer(nav2_client, get_pose_client)
    robot_motion_action_server = MotionActionServer()
    rclpy.spin(robot_motion_action_server)


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