import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def load_robot_names_from_yaml(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    data = data.get("setups", {})
    data_rr = data.get("real_robots", {})
    first_end_robot_names = data_rr.get("first_end_robot_names", [])
    last_end_robot_names = data_rr.get("last_end_robot_names", [])
    if len(first_end_robot_names) != len(last_end_robot_names):
        raise ValueError("First and last end robot numbers should match")
    return {
        "first_end_robot_names": first_end_robot_names,
        "last_end_robot_names": last_end_robot_names,
    }


def generate_launch_description():
    ld = LaunchDescription()

    robot_names_file_path = "resources/config/robot_names.yaml"
    robot_names = load_robot_names_from_yaml(robot_names_file_path)
    import json

    all_robot_names = (
        robot_names["first_end_robot_names"] + robot_names["last_end_robot_names"]
    )
    first_end_robot_names_as_string = json.dumps(robot_names["first_end_robot_names"])
    last_end_robot_names_as_string = json.dumps(robot_names["last_end_robot_names"])
    combined_robot_names_as_string = json.dumps(all_robot_names)

    robot_names_launch_arg = list(
        {
            "first_end_robot_names": first_end_robot_names_as_string,
            "last_end_robot_names": last_end_robot_names_as_string,
        }.items()
    )
    combined_robot_names_launch_arg = list(
        {"robot_names": combined_robot_names_as_string}.items()
    )
    print(robot_names_launch_arg)
    print(combined_robot_names_launch_arg)
    robot_config_file_launch_arg = list(
        {"robot_config_file": "resources/config/robot_configs.yaml"}.items()
    )
    config_file_launch_arg = list(
        {"config_file": "resources/config/rr_config.yaml"}.items()
    )
    rviz_file_launch_arg = list({"rviz_file": "resources/rviz/rr_plan.rviz"}.items())

    optitrack_launch_dir = PathJoinSubstitution(
        [FindPackageShare("mocap4r2_optitrack_driver"), "launch"]
    )
    mocap_tf_launch_dir = PathJoinSubstitution(
        [FindPackageShare("mocap4r2_tf2"), "launch"]
    )

    controller_launch_dir = PathJoinSubstitution(
        [FindPackageShare("feedback_linearization_controller"), "launch"]
    )

    motor_controller_launch_dir = PathJoinSubstitution(
        [FindPackageShare("motor_controller"), "launch"]
    )

    cable_plan_executor_launch_dir = PathJoinSubstitution(
        [FindPackageShare("cable_plan_executor"), "launch"]
    )

    cable_action_planner_launch_dir = PathJoinSubstitution(
        [FindPackageShare("cable_action_planner"), "launch"]
    )

    path_planner_launch_dir = PathJoinSubstitution(
        [FindPackageShare("path_planner"), "launch"]
    )

    robot_visualizer_launch_dir = PathJoinSubstitution(
        [FindPackageShare("robot_visualizer"), "launch"]
    )

    for robot_name in all_robot_names:
        ld.add_action(
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [motor_controller_launch_dir, "motor_controller_launch.py"]
                        ),
                        launch_arguments=config_file_launch_arg,
                    ),
                ]
            )
        )
        ld.add_action(
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [
                                controller_launch_dir,
                                "feedback_linearization_controller.launch.py",
                            ]
                        ),
                        launch_arguments=robot_config_file_launch_arg
                        + config_file_launch_arg,
                    ),
                ]
            )
        )
        ld.add_action(
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [robot_visualizer_launch_dir, "robot_visualizer.launch.py"]
                        ),
                        launch_arguments=robot_config_file_launch_arg
                        + config_file_launch_arg,
                    ),
                ]
            )
        )

    # For all robots
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([optitrack_launch_dir, "optitrack2.launch.py"])
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([mocap_tf_launch_dir, "mocap4r2_tf2.launch.py"])
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [cable_plan_executor_launch_dir, "cable_plan_executor.launch.py"]
            ),
            launch_arguments=config_file_launch_arg + robot_names_launch_arg,
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [cable_action_planner_launch_dir, "cable_action_planner.launch.py"]
            ),
            launch_arguments=config_file_launch_arg + robot_names_launch_arg,
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([path_planner_launch_dir, "path_planner.launch.py"]),
            launch_arguments=config_file_launch_arg + combined_robot_names_launch_arg,
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            "resources/launch/vis.launch.py",
            launch_arguments=config_file_launch_arg + rviz_file_launch_arg,
        )
    )
    return ld
