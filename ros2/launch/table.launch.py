from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("object_descriptions"), "objects/table.urdf.xacro"]),
            " ",
            "prefix:=table_ ",
            "connected_to:='' ",
            "xyz:='1  0.5 0' ",
            "rpy:='0 0 1.57' ",
            "height:=0.6 ",
            "x_size:=0.5 ",
            "y_size:=1"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("object_descriptions"), "rviz/object.rviz"])],
        output="log",
    )

    nodes = [
        robot_state_pub_node,
        rviz_node
    ]

    return LaunchDescription(nodes)
