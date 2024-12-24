from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ndi_description"),
                    "config",
                    "ndi_system.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # robot_description,
            str(FindPackageShare("ndi_bringup").find("ndi_bringup") / "config/controllers.yaml"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[("~/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ndi_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ndi_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay start of joint_state_broadcaster after controller_manager
    delay_joint_state_broadcaster_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of ndi_broadcaster after controller_manager
    delay_ndi_broadcaster_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ndi_broadcaster_spawner],
        )
    )

    nodes = [
        robot_state_pub_node,
        controller_manager,
        delay_joint_state_broadcaster_after_controller_manager,
        delay_ndi_broadcaster_after_controller_manager,
    ]

    return LaunchDescription(nodes)