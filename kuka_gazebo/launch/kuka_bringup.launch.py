# Author: REZ3LIET
# Updated for ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2

import os
import time
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import rclpy.logging


logger = rclpy.logging.get_logger("kuka_bringup.launch")


# =============================================================================
# Utility functions
# =============================================================================

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()

    except EnvironmentError:
        return None


def rewrite_yaml(source_file: str, root_key: str):
    """
    Add the robot namespace as root key to the controller YAML.

    If no namespace is used, return the original YAML directly.
    """

    if not root_key:
        return source_file

    with open(source_file, "r") as file:
        original_data = yaml.safe_load(file)

    updated_yaml = {root_key: original_data}

    destination_path = f"/tmp/kuka_controllers_{time.time()}.yaml"

    with open(destination_path, "w") as file:
        yaml.safe_dump(updated_yaml, file)

    return destination_path


def load_yaml(package_path, file_path):
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)

    except EnvironmentError:
        return None


# =============================================================================
# Robot launch
# =============================================================================

def load_robot(context, *args, **kwargs):

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------

    use_sim_time = LaunchConfiguration(
        "use_sim_time",
        default="True"
    )

    robot_name = LaunchConfiguration(
        "robot_name",
        default="kuka_arm"
    )

    namespace = LaunchConfiguration(
        "namespace",
        default=""
    )

    gripper_name = LaunchConfiguration(
        "gripper_name",
        default="robotiq_2f_140"
    )

    position_x = LaunchConfiguration(
        "position_x",
        default="0.0"
    )

    position_y = LaunchConfiguration(
        "position_y",
        default="0.0"
    )

    orientation_yaw = LaunchConfiguration(
        "orientation_yaw",
        default="0.0"
    )


    # Evaluate values
    robot_name_val = robot_name.perform(context)
    namespace_val = namespace.perform(context)
    gripper_name_val = gripper_name.perform(context)


    logger.info(
        f"Loading Kuka Robot [{gripper_name_val}] "
        f"with name: {robot_name_val}, "
        f"namespace: {namespace_val}"
    )


    # -------------------------------------------------------------------------
    # Select robot configuration according to gripper
    # -------------------------------------------------------------------------

    if gripper_name_val == "robotiq_2f_85":

        controller_yaml = "config/kuka_2f85_controllers.yaml"
        moveit_pkg = "kuka_2f85_moveit"

    elif gripper_name_val == "robotiq_2f_140":

        controller_yaml = "config/kuka_2f140_controllers.yaml"
        moveit_pkg = "kuka_2f140_moveit"

    else:

        controller_yaml = "config/kuka_controllers.yaml"
        moveit_pkg = "kuka_moveit"


    # -------------------------------------------------------------------------
    # Package paths
    # -------------------------------------------------------------------------

    gazebo_pkg_dir = get_package_share_directory(
        "kuka_gazebo"
    )

    description_pkg_dir = get_package_share_directory(
        "kuka_description"
    )

    moveit_pkg_dir = get_package_share_directory(
        moveit_pkg
    )


    # -------------------------------------------------------------------------
    # Controller configuration
    # -------------------------------------------------------------------------

    controller_file = rewrite_yaml(
        source_file=os.path.join(
            gazebo_pkg_dir,
            controller_yaml
        ),
        root_key=namespace_val,
    )


    # -------------------------------------------------------------------------
    # Robot description
    # -------------------------------------------------------------------------

    robot_xacro = Command([
        "xacro ",
        os.path.join(
            description_pkg_dir,
            "urdf/kr70_r2100.urdf.xacro"
        ),
        " robot_name:=",
        robot_name,
        " namespace:=",
        namespace,
        " gripper_name:=",
        gripper_name,
        " controller_file:=",
        controller_file,
    ])

    robot_description = {
        "robot_description": robot_xacro
    }


    # -------------------------------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------------------------------

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        remappings=remappings,
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )


    # -------------------------------------------------------------------------
    # Spawn robot in Gazebo Harmonic
    # -------------------------------------------------------------------------

    robot_description_topic = (
        f"{namespace_val}/robot_description"
        if namespace_val
        else "/robot_description"
    )

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            robot_description_topic,

            "-name",
            robot_name,

            "-robot_namespace",
            namespace,

            "-x",
            position_x,

            "-y",
            position_y,

            "-Y",
            orientation_yaw,
        ],
        output="both",
    )


    # =========================================================================
    # ros2_control controllers
    # =========================================================================

    controller_manager_path = (
        f"{namespace_val}/controller_manager"
        if namespace_val
        else "/controller_manager"
    )


    # -------------------------------------------------------------------------
    # Joint State Broadcaster
    # -------------------------------------------------------------------------

    joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
            "-c",
            controller_manager_path,
        ],
        output="screen",
    )


    # -------------------------------------------------------------------------
    # KUKA arm controller
    # -------------------------------------------------------------------------

    kuka_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "kuka_arm_controller",
            "-c",
            controller_manager_path,
        ],
        output="screen",
    )


    # -------------------------------------------------------------------------
    # Robotiq controller
    # -------------------------------------------------------------------------

    if gripper_name_val == "robotiq_2f_85":

        robotiq_controller = ExecuteProcess(
            cmd=[
                "ros2",
                "control",
                "load_controller",
                "--set-state",
                "active",
                "robotiq_2f85_controller",
                "-c",
                controller_manager_path,
            ],
            output="screen",
        )

    elif gripper_name_val == "robotiq_2f_140":

        robotiq_controller = ExecuteProcess(
            cmd=[
                "ros2",
                "control",
                "load_controller",
                "--set-state",
                "active",
                "robotiq_2f140_controller",
                "-c",
                controller_manager_path,
            ],
            output="screen",
        )

    else:

        robotiq_controller = ExecuteProcess(
            cmd=[
                "echo",
                "No gripper loaded",
            ],
            output="screen",
        )


    # -------------------------------------------------------------------------
    # Load controllers after robot spawn
    # -------------------------------------------------------------------------

    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                joint_state_controller,
                kuka_controller,
                robotiq_controller,
            ],
        )
    )


    # =========================================================================
    # MoveIt 2 configuration
    # =========================================================================

    moveit_config = (
        MoveItConfigsBuilder(
            "kr70_r2100",
            package_name=moveit_pkg,
        )

        .robot_description(
            file_path="config/kr70_r2100.urdf.xacro",
            mappings={
                "robot_name": robot_name_val,
                "namespace": namespace_val,
                "gripper_name": gripper_name_val,
                "controller_file": controller_file,
            },
        )

        .planning_pipelines(
            pipelines=[
                "ompl",
                "pilz_industrial_motion_planner",
            ],
            default_planning_pipeline=(
                "pilz_industrial_motion_planner"
            ),
        )

        .joint_limits(
            file_path="config/joint_limits.yaml"
        )

        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )

        .robot_description_semantic(
            file_path="config/kr70_r2100.srdf"
        )

        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )

        .pilz_cartesian_limits(
            file_path="config/pilz_cartesian_limits.yaml"
        )

        .to_moveit_configs()
    )


    # -------------------------------------------------------------------------
    # Planning Scene configuration
    # -------------------------------------------------------------------------

    planning_scene_parameters = {

        "publish_planning_scene": True,

        "publish_geometry_updates": True,

        "publish_state_updates": True,

        "publish_transforms_updates": True,

        "publish_robot_description": True,

        "publish_robot_description_semantic": True,
    }


    # =========================================================================
    # OMPL pipeline - MoveIt Jazzy format
    # =========================================================================

    #
    # IMPORTANT:
    #
    # MoveIt Jazzy expects:
    #
    # planning_plugins -> string array
    # request_adapters -> string array
    # response_adapters -> string array
    #
    # NOT the old Humble style strings.
    #

    ompl_planning_pipeline_config = {

        "ompl": {

            "planning_plugins": [
                "ompl_interface/OMPLPlanner",
            ],

            "request_adapters": [

                "default_planning_request_adapters/"
                "ResolveConstraintFrames",

                "default_planning_request_adapters/"
                "ValidateWorkspaceBounds",

                "default_planning_request_adapters/"
                "CheckStartStateBounds",

                "default_planning_request_adapters/"
                "CheckStartStateCollision",
            ],

            "response_adapters": [

                "default_planning_response_adapters/"
                "AddTimeOptimalParameterization",

                "default_planning_response_adapters/"
                "ValidateSolution",

                "default_planning_response_adapters/"
                "DisplayMotionPath",
            ],

            "start_state_max_bounds_error": 0.1,
        }
    }


    # -------------------------------------------------------------------------
    # Load existing OMPL planner definitions
    # -------------------------------------------------------------------------

    ompl_planning_yaml = load_yaml(
        moveit_pkg_dir,
        "config/ompl_planning.yaml",
    )

    if ompl_planning_yaml:

        ompl_planning_pipeline_config[
            "ompl"
        ].update(
            ompl_planning_yaml
        )


    # =========================================================================
    # Move Group
    # =========================================================================

    #
    # Pilz does NOT need to be manually injected here.
    #
    # MoveItConfigsBuilder loads the official
    # pilz_industrial_motion_planner_planning.yaml
    # configuration supplied by MoveIt if the robot package
    # does not contain its own one.
    #

    move_group_node = Node(

        package="moveit_ros_move_group",

        executable="move_group",

        namespace=namespace_val,

        output="screen",

        parameters=[

            # Robot + SRDF + controllers + joint limits
            # + pipelines
            moveit_config.to_dict(),

            # Planning scene publishing
            planning_scene_parameters,

            # Correct Jazzy OMPL configuration
            ompl_planning_pipeline_config,

            # Gazebo clock
            {
                "use_sim_time": use_sim_time
            },
        ],
    )


    # -------------------------------------------------------------------------
    # Start Move Group after controllers
    # -------------------------------------------------------------------------

    load_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robotiq_controller,
            on_exit=[
                move_group_node
            ],
        )
    )


    # =========================================================================
    # Control scripts / MoveGroupInterface
    # =========================================================================

    # -------------------------------------------------------------------------
    # SRDF
    # -------------------------------------------------------------------------

    robot_description_semantic_config = load_file(
        moveit_pkg,
        "config/kr70_r2100.srdf",
    )

    robot_description_semantic = {
        "robot_description_semantic":
            robot_description_semantic_config
    }


    # -------------------------------------------------------------------------
    # Kinematics
    # -------------------------------------------------------------------------

    kinematics_yaml = load_yaml(
        moveit_pkg_dir,
        "config/kinematics.yaml",
    )

    robot_description_kinematics = {
        "robot_description_kinematics":
            kinematics_yaml
    }


    # -------------------------------------------------------------------------
    # Move interface node
    # -------------------------------------------------------------------------

    move_interface = Node(

        package="control_scripts",

        executable="move_robot",

        namespace=namespace_val,

        output="screen",

        parameters=[

            robot_description,

            robot_description_semantic,

            robot_description_kinematics,

            {
                "use_sim_time": use_sim_time
            },

            {
                "ENV_PARAM": "gazebo"
            },
        ],
    )


    # =========================================================================
    # Return launch entities
    # =========================================================================

    return [

        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="True",
            description="Use Gazebo simulation clock",
            choices=[
                "True",
                "False",
            ],
        ),

        DeclareLaunchArgument(
            name="robot_name",
            default_value="kuka_arm",
            description="Name of the robot",
        ),

        DeclareLaunchArgument(
            name="namespace",
            default_value="",
            description="Robot namespace",
        ),

        DeclareLaunchArgument(
            name="gripper_name",
            default_value="robotiq_2f_140",
            description="Name of gripper to use",
            choices=[
                "",
                "robotiq_2f_85",
                "robotiq_2f_140",
            ],
        ),

        DeclareLaunchArgument(
            name="position_x",
            default_value="0.0",
            description="Robot X spawn position",
        ),

        DeclareLaunchArgument(
            name="position_y",
            default_value="0.0",
            description="Robot Y spawn position",
        ),

        DeclareLaunchArgument(
            name="orientation_yaw",
            default_value="0.0",
            description="Robot spawn yaw angle",
        ),

        robot_state_publisher,

        spawn_robot_node,

        load_controllers,

        load_move_group,

        move_interface,
    ]


# =============================================================================
# Main LaunchDescription
# =============================================================================

def generate_launch_description():

    #
    # I keep the launch argument name "ign_gz" temporarily
    # for backwards compatibility with the original project.
    #
    # Internally this now launches ros_gz_sim / Gazebo Harmonic.
    #

    ign_gz = LaunchConfiguration(
        "ign_gz",
        default="True",
    )


    # -------------------------------------------------------------------------
    # Gazebo Harmonic world
    # -------------------------------------------------------------------------

    world = os.path.join(
        get_package_share_directory(
            "kuka_gazebo"
        ),
        "world/empty_world.sdf",
    )


    gz_sim_node = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    "ros_gz_sim"
                ),
                "launch",
                "gz_sim.launch.py",
            )
        ),

        launch_arguments={
            "gz_args": [
                world,
                " -r -v1",
            ]
        }.items(),

        condition=IfCondition(
            ign_gz
        ),
    )


    # -------------------------------------------------------------------------
    # Gazebo Harmonic -> ROS 2 clock bridge
    # -------------------------------------------------------------------------

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
        condition=IfCondition(ign_gz),
    )


    # -------------------------------------------------------------------------
    # Launch
    # -------------------------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            name="ign_gz",
            default_value="True",
            description=(
                "Start Gazebo Harmonic simulation"
            ),
            choices=[
                "True",
                "False",
            ],
        ),

        gz_sim_node,

        clock_bridge,

        OpaqueFunction(
            function=load_robot
        ),
    ])