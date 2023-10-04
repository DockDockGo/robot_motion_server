import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from robot_action_interfaces.action import DockUndock
from robot_action_interfaces.srv import GetRobotPose
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import time

#! START HERE IN MORNING
# class Nav2Client(Node):
 
#     def __init__(self):
#         # Create a client    
#         super().__init__('nav2_client_node')
#         self.cli = self.create_client(AddTwoInts, 'add_two_ints')
       
#         # Check if the a service is available  
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = AddTwoInts.Request()

class MotionActionServer(Node):

    def __init__(self):
        super().__init__('motion_action_server')
        self._action_server = ActionServer(
            self,
            DockUndock,
            'DockUndock',
            self.execute_callback)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # msg format and clients to communicate with Nav2
        self.goal_msg = NavigateToPose.Goal()
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Initialize the service client for robot pose feedback
        # (note the name robot_pose_service should be the same defined in the robot_pose_server.py)
        self.robot_pose_client = self.create_client(GetRobotPose, 'get_robot_pose')

        while not self.robot_pose_client.wait_for_service(timeout_sec=2):
            self.get_logger().error("Robot Pose Retriever Service Not Available")
        
        self.pose_request = GetRobotPose.Request()
        self.nav2_feedback = None
    
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

    def feedback_callback(self, feedback_msg):
        self.nav2_feedback = feedback_msg
        # self.get_logger().info('Received feedback: %s' % feedback_msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Define and fill the feedback message
        feedback_msg = DockUndock.Feedback()

        # Do some checks to verify if goal pose is correct later on
        # feedback_msg.feedback = 0.2
        # goal_handle.publish_feedback(feedback_msg)

        nav2_goal_status = self.send_goal_pose(x=2.0, y=2.0, yaw=1.57, goal_handle=goal_handle)

        # # Request the robot's pose from the service client
        self.pose_request.frame_id = ''
        future = self.robot_pose_client.call_async(self.pose_request)
        # rclpy.spin_until_future_complete(self, future)
        rclpy.spin_once()

        if future.done():
            try:
                reponse = future.result()
            except Exception as e:
                self.get_logger().error('Service call failed %r' % (e,))
            else:
                feedback_msg.feedback = reponse.robot_pose
                goal_handle.publish_feedback(feedback_msg)

        # Add a delay before the next request (adjust as needed)
        # time.sleep(1)  # One second delay

        # possibly add more checks here
        if nav2_goal_status == "successful":
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = DockUndock.Result()
        result.success = True
        return result
    

    def send_goal_pose(self, x, y, yaw, goal_handle): # yaw in radians
        self.goal_msg.pose.header.frame_id = 'map'
        self.goal_msg.pose.pose.position.x = x
        self.goal_msg.pose.pose.position.y = y

        # Set the orientation (yaw) of the goal pose
        self.goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav2_client.wait_for_server()
        self.get_logger().info('Goal pose sent to Navigation2.')
        # goal_response_future = self.nav2_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)
        goal_response_future = self.nav2_client.send_goal_async(self.goal_msg)

        # Wait for a goal response
        #! spin_until_future_complete will not work if within a callback 
        # rclpy.spin_until_future_complete(self, goal_response_future)
        rclpy.spin_once(self)

        # Use the goal handle from the DockUndock action
        if goal_response_future.result() is not None:
            self.get_logger().info('Nav2 accepted goal pose')
            return "successful"
        else:
            return "failed"


def main(args=None):
    rclpy.init(args=args)

    robot_motion_action_server = MotionActionServer()

    rclpy.spin(robot_motion_action_server)


if __name__ == '__main__':
    main()
