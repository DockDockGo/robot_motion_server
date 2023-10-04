import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([

        #! FIX LATER, not launching service, need to do manual ros2 run
        # Launch the GetRobotPose Client node with arguments
        # launch_ros.actions.Node(
        #     package='robot_action_interfaces',
        #     executable='robot_pose_server',
        #     name='robot_pose_server',
        #     output='screen',
        # ),

        # Launch the MotionActionServer node with arguments
        launch_ros.actions.Node(
            package='robot_motion_server',
            executable='robot_docking_undocking_server_node', # NOTE: Executable name defined in setup.py
            name='robot_docking_undocking_server_node',
            output='screen',
        ),

        # Launch the MotionActionServer node with arguments
        launch_ros.actions.Node(
            package='robot_motion_server',
            executable='robot_motion_server_node',
            name='robot_motion_server_node',
            output='screen',
        ),
    ])
