from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
import launch.conditions as conditions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare("serp_rl")

    world_path = LaunchConfiguration("world_path")
    update_rate = LaunchConfiguration("update_rate")
    step_size = LaunchConfiguration("step_size")
    show_viz = LaunchConfiguration("show_viz")
    viz_pub_rate = LaunchConfiguration("viz_pub_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ld = LaunchDescription(
        [
            # Flatland parameters.
            # You can either change these values directly here or override them in the launch command default values. Example:
            #   ros2 launch serp_rl serp_rl.launch.py update_rate:="20.0"
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution([pkg_share, "world/world.yaml"]),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="1000.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.01"),
            DeclareLaunchArgument(name="show_viz", default_value="true"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            DeclareLaunchArgument(name="use_sim_time", default_value="true"),

            SetEnvironmentVariable(name="ROSCONSOLE_FORMAT", value="[${severity} ${time} ${logger}]: ${message}"),

            # **** Nodes launched by this file ****
            # launch flatland server
            Node(
                name="flatland_server",
                package="flatland_server",
                executable="flatland_server",
                output="screen",
                parameters=[
                    # use the arguments passed into the launchfile for this node
                    {"world_path": world_path},
                    {"update_rate": update_rate},
                    {"step_size": step_size},
                    {"show_viz": show_viz},
                    {"viz_pub_rate": viz_pub_rate},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            # runs the code in the file serp_rl/__init__.py that is used to control the robot
            Node(
                name="serp_rl",
                package="serp_rl",
                executable="serp_rl",
                output="screen",
            ),

            # maps
            Node(
                name="tf",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),

            # visualisation 
            Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_share, "rviz/robot_navigation.rviz"])],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=conditions.IfCondition(show_viz),
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
