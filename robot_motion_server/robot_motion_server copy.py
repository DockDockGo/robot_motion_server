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

        for i in range(0, int(goal_handle.request.secs)):
            self.publisher_.publish(msg)
            time.sleep(1)

        get_pose_future = self._robot_pose_client.call_async(self.pose_request)
        rclpy.spin_until_future_complete(self, get_pose_future)
        
        time.sleep(1)
        
        if get_pose_future.result() is not None:
            feedback_msg.feedback = get_pose_future.result().robot_pose
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        msg.linear.x = 0.0
        self.publisher_.publish(msg)

        result = DockUndock.Result()
        result.success = True
        return result


class MotionActionServer(Node):

    def __init__(self):
        #! TODO: Add namespace
        namespace = ""
        super().__init__('motion_action_server')
        self.get_logger().info("Starting Motion Action Server")
        
        # Nav2 Clients
        # print( "Action client used = ", + namespace + "/navigate_to_pose")
        # self.nav2_client = ActionClient(self, NavigateToPose, (namespace + "/navigate_to_pose"))
        self.get_logger().info(f"Action client used =  {namespace}/navigate_through_poses")
        self.nav2_client = ActionClient(self, NavigateThroughPoses, (namespace + "/navigate_through_poses"))
        self.nav2_feedback = None
        self.nav2_goal_handle = None
        self.nav2_result_status = None

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
        while not self.nav2_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warning("'NavigateThroughPoses' action server not available, waiting...")
        
        self.get_logger().info('Executing Navigation Goal...')
        # NOTE: If you want to use the action server input command you can do goal_handle.request.secs

        route_waypoints = self.get_poses_at_waypoints(goal_handle.request.secs)

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = route_waypoints
        goal_msg.behavior_tree = ''

        self.get_logger().info(f'Navigating with {len(goal_msg.poses)} goals....')
        send_goal_future = self.nav2_client.send_goal_async(goal_msg, self.nav2_feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.nav2_goal_handle = send_goal_future.result()

        if not self.nav2_goal_handle.accepted:
            self.get_logger().error(f'Goal with {len(goal_msg.poses)} poses was rejected!')
            result = Navigate.Result()
            result.success = False
            return result

        self.nav2_result_future = self.nav2_goal_handle.get_result_async()

        i = 0
        while not self.nav2_result_future:
            i = i + 1
            if self.nav2_feedback and i%5 == 0:
                print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                      Duration.from_msg(self.nav2_feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(self.nav2_feedback.navigation_time) > Duration(seconds=50.0):
                    print('Navigation has exceeded timeout of 180s, canceling the request.')
                    self.cancelTask()

        # define the feedback_msg of the motion_action_server
        # motion_server_feedback = Navigate.Feedback()        

        """
        ############### GET POSE FEEDBACK FROM LOCALIZATION ######################
        get_pose_future = self._robot_pose_client.call_async(self.pose_request)
        rclpy.spin_until_future_complete(self, get_pose_future)
        
        if get_pose_future.result() is not None:
            self.get_logger().info("Before")
            self.get_logger().info(str(type(get_pose_future.result().robot_pose)))
            # self.get_logger().info(str(get_pose_future.result().robot_pose))
            motion_server_feedback.feedback = get_pose_future.result().robot_pose
            self.get_logger().info("After")
            self.get_logger().info(str(type(motion_server_feedback.feedback)))
            goal_handle.publish_feedback(motion_server_feedback)
        ##########################################################################
        """

        motion_server_result = Navigate.Result()

        # May need to do this:
        rclpy.spin_until_future_complete(self, self.nav2_result_future, timeout_sec=0.10)
        if self.nav2_result_future.result():
            self.nav2_result_status = self.nav2_result_future.result().status

        if self.nav2_result_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Route complete!')
            motion_server_result.success = True
            goal_handle.succeed()
        elif self.nav2_result_status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info('route was canceled, exiting.')
            motion_server_result.success = False
            goal_handle.abort() 
        elif self.nav2_result_status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('route failed!')
            motion_server_result.success = False
            goal_handle.abort()
        else:
            motion_server_result.success = False
            goal_handle.abort()
        
        return motion_server_result
    

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.get_logger().warn('Canceling current task.')
        if self.nav2_result_future:
            future = self.nav2_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def nav2_feedback_callback(self, msg):
        self.nav2_feedback = msg.feedback

    
    def get_poses_at_waypoints(self, secs):
        waypoints = [[0.75, -2.82, 0],
                     [4.22, -2.82, 0],
                     [4.04, 2.78, 0],
                     [1.71, 2.71, 0],]
        route_waypoints = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in waypoints:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.z = math.sin(pt[2] / 2.0)
            pose.pose.orientation.w = math.cos(pt[2] / 2.0)
            route_waypoints.append(deepcopy(pose))

        return route_waypoints


# def nav2_client(args=None):
#     rclpy.init(args=args)

#     # start the nav2 client
#     nav2_client = Nav2Client()
#     rclpy.spin(nav2_client)


# def get_pose_client(args=None):
#     rclpy.init(args=args)

#     # start the GetRobotPose client
#     get_pose_client = RobotPoseClient()
#     rclpy.spin(get_pose_client)


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