import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ld = LaunchDescription()

    #### Define the params ####

    use_namespace = LaunchConfiguration('use_namespace', default=True)
    namespace = LaunchConfiguration('namespace', default="robot1")

    if not use_namespace:
        namespace = ""

    # Launch the MotionActionServer node with arguments
    Dock_Undock = launch_ros.actions.Node(
        package='robot_motion_server',
        executable='robot_docking_undocking_server_node', # NOTE: Executable name defined in setup.py
        name='robot_docking_undocking_server_node',
        output='screen',
        parameters=[{'namespace_param': namespace}]
    )

    # Launch the MotionActionServer node with arguments
    Navigation = launch_ros.actions.Node(
        package='robot_motion_server',
        executable='robot_motion_server_node', # NOTE: Executable name defined in setup.py
        name='robot_motion_server_node',
        output='screen',
        parameters=[{'namespace_param': namespace}]
    )

    ld.add_action(Dock_Undock)
    ld.add_action(Navigation)

    return ld