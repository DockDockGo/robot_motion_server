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
            velocity_subscriber_topic = "/" + robot_namespace + "/odom"
        else:
            action_server_name = "DockUndock"
            publisher_topic = "/cmd_vel"
            subscriber_topic = "/map_pose"
            velocity_subscriber_topic = "/odom"

        self.get_logger().info(f"cmd vel topic is {publisher_topic}")
        self.robot_pose = None
        self.robot_vel = None
        self.goal_pose = None
        self.goal_pose_2 = None
        self.goal_threshold = 0.07
        self.undock_duration = 2

        # construct the action server
        self._action_server = ActionServer(
            self,
            DockUndock,
            action_server_name,
            self.execute_callback,
            callback_group=self.callback_group)

        self.publisher_ = self.create_publisher(Twist, publisher_topic, 10)

        # Pose Subscribers
        self.pose_subscription = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic=subscriber_topic,  # Topic on which pose is being relayed
            callback=self.robot_pose_callback,
            qos_profile=10,  # Adjust the queue size as needed
        )

        # Velocity Subscribers
        self.velocity_subscription = self.create_subscription(
            msg_type=Odometry,
            topic=velocity_subscriber_topic,  # Topic on which pose is being relayed
            callback=self.robot_velocity_callback,
            qos_profile=10,  # Adjust the queue size as needed
        )

        # get euclidean distance first
        self.define_goal_pose()
        self.y_delta = None

    def wrap_around_pi(self, x):
        x = abs(x)
        pi = 3.1415926
        if x < pi:
            return x
        return abs((x + 2 * pi) % pi - pi)

    def radians_to_degrees(self, angle):
        return angle * 180.0 / 3.1415926

    def calc_dx_dy_dtheta(self, pose1):
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
        # dx = goal_pose - robot_pose
        dx = pos1.position.x - pos2.position.x
        dy = pos1.position.y - pos2.position.y
        # Calculate the orientation error (theta_error) using arctangent
        theta_error = math.atan2(dy, dx)
        # Convert orientations to Euler angles (roll, pitch, and yaw)
        euler_angles1 = euler_from_quaternion([pos1.orientation.x, pos1.orientation.y, pos1.orientation.z, pos1.orientation.w])
        euler_angles2 = euler_from_quaternion([pos2.orientation.x, pos2.orientation.y, pos2.orientation.z, pos2.orientation.w])
        # Calculate the orientation difference
        true_theta_error = self.wrap_around_pi(euler_angles2[2] - euler_angles1[2])

        return dx, dy, theta_error, true_theta_error


    def get_individual_orientations(self, pose1):
        pose2 = self.robot_pose
        # Extract the positions from the poses
        if isinstance(pose1, PoseStamped) and isinstance(pose2, PoseWithCovarianceStamped):
            pos1 = pose1.pose
            pos2 = pose2.pose.pose
        elif pose1 is None: # in Navigation State
            return None
        else:
            raise ValueError("Input pose1 is not a valid type (Pose or PoseWithCovariance)")

        # Convert orientations to Euler angles (roll, pitch, and yaw)
        euler_angles1 = euler_from_quaternion([pos1.orientation.x, pos1.orientation.y, pos1.orientation.z, pos1.orientation.w])
        euler_angles2 = euler_from_quaternion([pos2.orientation.x, pos2.orientation.y, pos2.orientation.z, pos2.orientation.w])

        return euler_angles1, euler_angles2


    def world_frame_orientation(self):
        orientation = self.robot_pose.pose.pose.orientation
        euler_angle_rad = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return euler_angle_rad[2]

    def define_goal_pose(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = 6.658
        self.goal_pose.pose.position.y = -0.1
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.675927815
        self.goal_pose.pose.orientation.w = 0.7369678335


    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def robot_velocity_callback(self, msg):
        self.robot_vel = msg.twist.twist

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Docking/Undocking...')

        # Define and fill the feedback message
        # feedback_msg = DockUndock.Feedback()
        # feedback_msg.pose_feedback = None
        # goal_handle.publish_feedback(feedback_msg)

        # Get current euclidean distance to goal
        if self.robot_pose is None:
            self.get_logger().error("Map Pose Not found")

        dock_id = int(abs(goal_handle.request.secs))
        self.get_logger().info(f"Docking Goal is {goal_handle.request.secs}")

        # Simple Undocking
        if goal_handle.request.secs > 0.0:
            msg = Twist()
            msg.linear.x = 0.2
            for i in range(0, self.undock_duration):
                self.get_logger().info("Undocking in Progress")
                self.publisher_.publish(msg)
                time.sleep(1)

            self.get_logger().info("FINISHED UNDOCKING")
            result = DockUndock.Result()
            result.success = True
            goal_handle.succeed()
            return result


        # Find direction of dock (Find along which axis is the dock)
        dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(PoseStamped())
        dock_axis = "y" if (true_dtheta > 0.8) else "x"
        self.get_logger().info(f"dock axis is {dock_axis}")
        # Figure out which direction robot needs to turn for 1st correction (1st while loop)
        robot_orientation_in_world_frame = self.world_frame_orientation()

        dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
        if dock_axis == "x":
            if abs(robot_orientation_in_world_frame) < 0.5:
                rotation_direction = -1 if dy < 0.0 else 1
            else:
                rotation_direction = 1 if dy < 0.0 else -1
        elif dock_axis == "y":
            if (robot_orientation_in_world_frame  < -1) and (robot_orientation_in_world_frame  > -2):
                self.get_logger().info("1st if")
                rotation_direction = -1 if dx < 0.0 else 1
            elif (robot_orientation_in_world_frame  > 1) and (robot_orientation_in_world_frame  < 2):
                self.get_logger().info("2nd if")
                rotation_direction = 1 if dx < 0.0 else -1

        assert rotation_direction

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

        self.get_logger().info(f"true_dtheta is {true_dtheta}")
        while(not (abs(true_dtheta) > 1.56 and abs(true_dtheta) < 1.58)):
            self.get_logger().info("Docking_1 in Progress")
            msg = Twist()
            msg.angular.z = 0.4 * rotation_direction
            self.publisher_.publish(msg)
            dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
            self.get_logger().info(f"dx, dy, dtheta, and true_dtheta is {dx}, {dy}, {dtheta}, {true_dtheta}")
            time.sleep(0.02)

        dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
        stop_condition = lambda dist: abs(dist) > 0.05
        while stop_condition(dy if dock_axis=="x" else dx):
            self.get_logger().info("Docking_2 in Progress")
            msg = Twist()
            msg.linear.x = 0.4 # note this has to be positive due the first loop's reorientation
            self.publisher_.publish(msg)
            dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
            self.get_logger().info(f"dx, dy, dtheta, and true_dtheta is {dx}, {dy}, {dtheta}, {true_dtheta}")
            time.sleep(0.02)

        dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
        self.get_logger().info(f"true_dtheta is {true_dtheta}")
        while(abs(true_dtheta) > 0.05):
            self.get_logger().info("Docking_3 in Progress")
            msg = Twist()
            msg.angular.z = 0.4 * rotation_direction * -1
            self.publisher_.publish(msg)
            dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
            self.get_logger().info(f"dx, dy, dtheta, and true_dtheta is {dx}, {dy}, {dtheta}, {true_dtheta}")
            time.sleep(0.02)

        dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
        stop_condition = lambda dist: dist > 0.03
        while stop_condition(dx if dock_axis=="x" else dy):
            self.get_logger().info("Docking_4 in Progress")
            msg = Twist()
            msg.linear.x = -0.2
            self.publisher_.publish(msg)
            dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
            self.get_logger().info(f"dx, dy, dtheta, and true_dtheta is {dx}, {dy}, {dtheta}, {true_dtheta}")
            time.sleep(0.02)

        msg = Twist()
        self.publisher_.publish(msg)
        self.get_logger().info("FINISHED DOCKING")
        result = DockUndock.Result()
        result.success = True

        goal_handle.succeed()

        return result


        #! Partially works
        # prev_dx, prev_dtheta = 0.0, 0.0
        # dx,dy,dtheta,true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
        # while(abs(dtheta) > 0.06 or abs(dy > 0.02)):
        #     self.get_logger().info("Docking_1 in Progress")
        #     ###### PID LOGIC ######
        #     msg = Twist()
        #     dx, dy, dtheta, true_dtheta = self.calc_dx_dy_dtheta(self.goal_pose)
        #     # Kp_x = 0.3
        #     Kp_theta = 0.4
        #     Kd = 1.0
        #     dt = 0.2
        #     # delta_accel_x = Kp_x*dx + Kd*(dx - prev_dx)
        #     delta_angular = Kp_theta*dtheta + Kd*(dtheta - prev_dtheta)
        #     # vel_x = delta_accel_x * dt + self.robot_vel.linear.x
        #     ang_vel_z = delta_angular * dt + self.robot_vel.angular.z
        #     msg.linear.x = 0.2
        #     msg.angular.z = ang_vel_z if true_dtheta < math.pi/2 else -ang_vel_z
        #     self.publisher_.publish(msg)
        #     # prev_dx = dx
        #     prev_dtheta = dtheta
        #     #######################
        #     self.get_logger().info(f"dx, dy, dtheta, and true_dtheta is {dx}, {dy}, {dtheta}, {true_dtheta}")
        #     time.sleep(0.2)


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

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
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