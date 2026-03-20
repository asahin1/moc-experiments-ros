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
    data_sim = data.get("simulation", {})
    rear_end_robot_names = data_sim.get("rear_end_robot_names", [])
    front_end_robot_names = data_sim.get("front_end_robot_names", [])
    if len(rear_end_robot_names) != len(front_end_robot_names):
        raise ValueError(
            f"There should be as many rear and robots as there are front end robots. Rear end: {len(rear_end_robot_names)}, Front end: {len(front_end_robot_names)}"
        )
    return {
        "rear_end_robot_names": rear_end_robot_names,
        "front_end_robot_names": front_end_robot_names,
    }


def generate_launch_description():
    ld = LaunchDescription()

    # Parse robot names
    robot_names_file_path = "resources/config/robot_names.yaml"
    robot_names = load_robot_names_from_yaml(robot_names_file_path)
    import json

    all_robot_names = (
        robot_names["rear_end_robot_names"] + robot_names["front_end_robot_names"]
    )
    rear_end_robot_names_as_string = json.dumps(robot_names["rear_end_robot_names"])
    front_end_robot_names_as_string = json.dumps(robot_names["front_end_robot_names"])
    combined_robot_names_as_string = json.dumps(all_robot_names)
    robot_names_launch_arg = list(
        {
            "rear_end_robot_names": rear_end_robot_names_as_string,
            "front_end_robot_names": front_end_robot_names_as_string,
        }.items()
    )
    combined_robot_names_launch_arg = list(
        {"robot_names": combined_robot_names_as_string}.items()
    )

    # For individual robot configs
    robot_config_file_launch_arg = list(
        {"robot_config_file": "resources/config/robot_configs.yaml"}.items()
    )

    # For general simulation configs
    config_file_launch_arg = list(
        {"config_file": "resources/config/sim_config.yaml"}.items()
    )

    # Simulation config
    rviz_file_launch_arg = list({"rviz_file": "resources/rviz/sim_plan.rviz"}.items())

    # Launch directories
    model_launch_dir = PathJoinSubstitution(
        [FindPackageShare("car_model_py"), "launch"]
    )
    mocap_launch_dir = PathJoinSubstitution(
        [FindPackageShare("mocap_sim_tf2_cpp"), "launch"]
    )
    controller_launch_dir = PathJoinSubstitution(
        [FindPackageShare("feedback_linearization_controller"), "launch"]
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

    # Per robot nodes
    for robot_name in all_robot_names:
        ld.add_action(
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    IncludeLaunchDescription(
                        PathJoinSubstitution([model_launch_dir, "car_model.launch.py"]),
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
                            [
                                controller_launch_dir,
                                "feedback_linearization_controller.launch.py",
                            ]
                        ),
                        launch_arguments=robot_config_file_launch_arg
                        + config_file_launch_arg
                        + combined_robot_names_launch_arg,
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

    # Common nodes
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution([mocap_launch_dir, "mocap_sim_tf2.launch.py"]),
            launch_arguments=config_file_launch_arg + combined_robot_names_launch_arg,
        ),
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
