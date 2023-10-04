import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from robot_action_interfaces.action import DockUndock
from robot_action_interfaces.srv import GetRobotPose
from robot_action_interfaces.action import Navigate
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import time


class Nav2Client(Node):
 
    def __init__(self):
        # Create a client    
        super().__init__('nav2_client_node')
        # msg format and clients to communicate with Nav2
        self.get_logger().info('Starting Nav2 Client Node')
        self.goal_msg = NavigateToPose.Goal()
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_feedback = None # TODO: replace with posewithcovariancestamped
       
        # Check if the a service is available  
        while not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def feedback_callback(self, feedback_msg):
        self.nav2_feedback = feedback_msg
        # self.get_logger().info('Received feedback: %s' % feedback_msg)

    def send_goal_pose(self, x, y, yaw): # yaw in radians
        self.goal_msg.pose.header.frame_id = 'map'
        self.goal_msg.pose.pose.position.x = x
        self.goal_msg.pose.pose.position.y = y

        # Set the orientation (yaw) of the goal pose
        self.goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # goal_response_future = self.nav2_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)
        goal_response_future = self.nav2_client.send_goal_async(self.goal_msg)

        self.get_logger().info('Goal pose sent to Navigation2.')

        return goal_response_future


class RobotPoseClient(Node):

    def __init__(self):
        # Create a client    
        super().__init__('robot_pose_client_node')
        self.get_logger().info('Starting Robot Pose Retriever Client')
        self.robot_pose_client = self.create_client(GetRobotPose, 'get_robot_pose')
        while not self.robot_pose_client.wait_for_service(timeout_sec=2):
            self.get_logger().error("Robot Pose Retriever Service Not Available")
        
        self.pose_request = GetRobotPose.Request()
    
    def get_pose(self):
        # Request the robot's pose from the service client
        self.pose_request.frame_id = ''
        future = self.robot_pose_client.call_async(self.pose_request)
        return future


class DockingUndockingActionServer(Node):
    
    def __init__(self):
        super().__init__('docking_undocking_action_server')
        self.get_logger().info("Starting Docking Undocking Action Server")
        
        # accept client node instances
        self.get_pose_client = RobotPoseClient()

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
        msg.linear.x = 0.5

        # Define and fill the feedback message
        feedback_msg = DockUndock.Feedback()

        for i in range(0, int(goal_handle.request.secs)):
            self.publisher_.publish(msg)
            time.sleep(1)

        get_pose_future = self.get_pose_client.get_pose()
        rclpy.spin_until_future_complete(self.get_pose_client, get_pose_future)
        
        if get_pose_future.done():
            try:
                reponse = get_pose_future.result()
                self.get_logger().info(str(reponse.robot_pose))
            except Exception as e:
                self.get_logger().error('Service call failed %r' % (e,))
            else:
                feedback_msg.feedback = PoseWithCovarianceStamped()
                feedback_msg.feedback = reponse.robot_pose
                goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        msg.linear.x = 0.0
        self.publisher_.publish(msg)

        result = DockUndock.Result()
        result.success = True
        return result


class MotionActionServer(Node):

    def __init__(self):
        super().__init__('motion_action_server')
        self.get_logger().info("Starting Motion Action Server")
        
        # accept client node instances
        self.nav2_client = Nav2Client()
        self.get_pose_client = RobotPoseClient()

        # construct the action server
        self._action_server = ActionServer(
            self,
            Navigate,
            'Navigate',
            self.execute_callback)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    
    """
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        msg = Twist()
        msg.linear.x = 0.5

        # Define and fill the feedback message
        feedback_msg = DockUndock.Feedback()

        for i in range(0, int(goal_handle.request.secs)):
            self.publisher_.publish(msg)
            # feedback_msg.feedback = 0.2
            # self.get_logger().info('temp feedback val, replace with pose: {0}'.format(feedback_msg.feedback))
            # goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        msg.linear.x = 0.0
        self.publisher_.publish(msg)

        result = DockUndock.Result()
        result.success = True
        return result
    """

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Navigation Goal...')

        # Define and fill the feedback message
        feedback_msg = Navigate.Feedback()

        # Do some checks to verify if goal pose is correct later on
        # feedback_msg.feedback = 0.2
        # goal_handle.publish_feedback(feedback_msg)

        nav2_goal_future = self.nav2_client.send_goal_pose(x=2.0, y=2.0, yaw=1.57)
        rclpy.spin_until_future_complete(self.nav2_client, nav2_goal_future)

        time.sleep(0.5)

        get_pose_future = self.get_pose_client.get_pose()
        rclpy.spin_until_future_complete(self.get_pose_client, get_pose_future)
        
        if get_pose_future.done():
            try:
                reponse = get_pose_future.result()
                self.get_logger().info(str(reponse.robot_pose))
            except Exception as e:
                self.get_logger().error('Service call failed %r' % (e,))
            else:
                feedback_msg.feedback = PoseWithCovarianceStamped()
                feedback_msg.feedback = reponse.robot_pose
                goal_handle.publish_feedback(feedback_msg)

        # possibly add more checks here
        if nav2_goal_future.done():
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = Navigate.Result()
        result.success = True
        return result




def nav2_client(args=None):
    rclpy.init(args=args)

    # start the nav2 client
    nav2_client = Nav2Client()
    rclpy.spin(nav2_client)


def get_pose_client(args=None):
    rclpy.init(args=args)

    # start the GetRobotPose client
    get_pose_client = RobotPoseClient()
    rclpy.spin(get_pose_client)


def MotionServer(args=None):
    rclpy.init(args=args)

    # start the MotionActionServer
    # robot_motion_action_server = MotionActionServer(nav2_client, get_pose_client)
    robot_motion_action_server = MotionActionServer()
    rclpy.spin(robot_motion_action_server)


def DockUndockServer(args=None):
    rclpy.init(args=args)

    # start the MotionActionServer
    # robot_motion_action_server = MotionActionServer(nav2_client, get_pose_client)
    robot_dockundock_action_server = DockingUndockingActionServer()
    rclpy.spin(robot_dockundock_action_server)


if __name__ == '__main__':
    MotionServer()