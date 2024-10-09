# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, RegisterEventHandler
# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     # Declare arguments
#     declared_arguments = [] 
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "gui",
#             default_value="true",
#             description="Start RViz2 automatically with this launch file.",
#         )
#     )

#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "controllers_file",
#             default_value="controllers.yaml",
#             description="YAML file with controller configurations.",
#         )
#     )

#     # Get URDF via xacro
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [
#                     FindPackageShare("example_7"),
#                     "urdf",
#                     "arm.urdf",
#                 ]
#             ),
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     # Node for publishing robot state
#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )

#     # # Controller manager
#     controller_manager_node = Node(
#     package="controller_manager",
#     executable="ros2_control_node",
#     parameters=[
#         robot_description,
#         PathJoinSubstitution(
#             [FindPackageShare("example_7"), "config", LaunchConfiguration("controllers_file")]
#         ),
#     ],
#     output={"stdout": "screen", "stderr": "screen"},
#     remappings=[('/controller_manager', '/controller_manager')]
# )


#     # Spawners for controllers
#     # joint_state_broadcaster_spawner = Node(
#     #     package="controller_manager",
#     #     executable="spawner",
#     #     arguments=["joint_state_broadcaster"],
#     #     output="screen",
#     # )

#     # position_trajectory_controller_spawner = Node(
#     #     package="controller_manager",
#     #     executable="spawner",
#     #     arguments=["position_trajectory_controller"],
#     #     output="screen",
#     # )

#     # RViz node
#     gui = LaunchConfiguration("gui")
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         condition=IfCondition(gui),
#     )

#     # # Ensure controllers are spawned after controller_manager is up
#     # delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
#     #     OnProcessExit(
#     #         target_action=controller_manager_node,
#     #         on_exit=[joint_state_broadcaster_spawner],
#     #     )
#     # )

#     # delayed_position_trajectory_controller_spawner = RegisterEventHandler(
#     #     OnProcessExit(
#     #         target_action=joint_state_broadcaster_spawner,
#     #         on_exit=[position_trajectory_controller_spawner],
#     #     )
#     # )

#     # List of nodes to launch
#     nodes = [
#         robot_state_pub_node,
#         controller_manager_node,
#         rviz_node,
#         # delayed_joint_state_broadcaster_spawner,
#         # delayed_position_trajectory_controller_spawner,
#     ]

#     return LaunchDescription(declared_arguments + nodes)









from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [] 
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="YAML file with controller configurations.",
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("example_7"),
                    "urdf",
                    "arm.urdf",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Node for publishing robot state
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            "/home/octobotics/ros2_control_ws/src/example_7/description/urdf/arm.urdf",
            "/home/octobotics/ros2_control_ws/src/example_7/bringup/config/controllers.yaml" ,
        ],
        output={"stdout": "screen", "stderr": "screen"},
        remappings=[('/controller_manager', '/controller_manager')]
    )

    # Spawners for controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller"],
        output="screen",
    )

    # RViz node
    gui = LaunchConfiguration("gui")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(gui),
    )

    # Ensure controllers are spawned after controller_manager is up
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_position_trajectory_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_trajectory_controller_spawner],
        )
    )

    # List of nodes to launch
    nodes = [
        robot_state_pub_node,
        controller_manager_node,
        rviz_node,
        delayed_joint_state_broadcaster_spawner,
        delayed_position_trajectory_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)























# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, RegisterEventHandler
# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     # Declare arguments
#     declared_arguments = [] 
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "gui",
#             default_value="true",
#             description="Start RViz2 automatically with this launch file.",
#         )
#     )

#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "controllers_file",
#             default_value="controllers.yaml",
#             description="YAML file with controller configurations.",
#         )
#     )

#     # Get URDF via xacro
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [
#                     FindPackageShare("example_7"),
#                     "urdf",
#                     "arm.urdf",
#                 ]
#             ),
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     # Node for publishing robot state
#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )

#     # Controller manager node
#     controller_manager_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[
#             robot_description,
#             PathJoinSubstitution(
#                 [FindPackageShare("example_7"),
#                 "bringup",
#                 "config",
#                 "controllers.yaml"]
#             )
#         ],
#         output={"stdout": "screen", "stderr": "screen"},
#         remappings=[('/controller_manager', '/controller_manager')]
#     )

#     # Spawners for controllers
#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster"],
#         output="screen",
#     )

#     position_trajectory_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["position_trajectory_controller"],
#         output="screen",
#     )

#     # RViz node
#     gui = LaunchConfiguration("gui")
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         condition=IfCondition(gui),
#     )

#     # Ensure controllers are spawned after controller_manager is up
#     delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
#         OnProcessExit(
#             target_action=controller_manager_node,
#             on_exit=[joint_state_broadcaster_spawner],
#         )
#     )

#     delayed_position_trajectory_controller_spawner = RegisterEventHandler(
#         OnProcessExit(
#             target_action=joint_state_broadcaster_spawner,
#             on_exit=[position_trajectory_controller_spawner],
#         )
#     )

#     # List of nodes to launch
#     nodes = [
#         robot_state_pub_node,
#         controller_manager_node,
#         rviz_node,
#         delayed_joint_state_broadcaster_spawner,
#         delayed_position_trajectory_controller_spawner,
#     ]

#     return LaunchDescription(declared_arguments + nodes)


# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     # Declare arguments
#     declared_arguments = [] 
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "gui",
#             default_value="false",
#             description="Start RViz2 automatically with this launch file.",
#         )
#     )

#     # Get URDF via xacro
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [
#                     FindPackageShare("example_7"),
#                     "urdf",
#                     "arm.urdf",
#                 ]
#             ),
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     # Node for publishing robot state (robot_state_publisher)
#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )

#     # Controller manager to load and manage hardware
#     controller_manager_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[
#             robot_description,  # Robot description (URDF)
#         ],
#         output="both",
#     )

#     # List of nodes to launch
#     nodes = [
#         robot_state_pub_node,
#         controller_manager_node,
#     ]

#     return LaunchDescription(declared_arguments + nodes)
