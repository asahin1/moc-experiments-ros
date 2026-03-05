# moc-experiments-ros

A ROS2 workspace for performing MOCP outputs in simulation and on real robots.

## Docker-based workflow

### To build the docker image

```bash
./bin/image_build.sh
```

### To run the container

```bash
./bin/run_container.sh
```

### To start a new shell inside the container

```bash
./bin/shell_container.sh
```

## Building
Run inside the container:

```bash
cd moc_ws
colcon build
```
## Running

Source the ros2 workspace:
```bash
source moc_ws/install/setup.bash
```

### Interactive navigation

Run the following launch files to create an interactive environment with simulated/real robots that can be assigned targets via clicking on rviz2:

#### Simulation

```bash
ros2 launch resources/launch/sim_click.launch.py
```
#### Real-Robot


```bash
ros2 launch resources/launch/rr_click.launch.py
```

### Pre-planned navigation by MOCP
Run the following launch file to make simulated/real robots execute planned trajectories:

#### Simulation
```bash
ros2 launch resources/launch/sim_plan.launch.py
```

#### Real-Robot

```bash
ros2 launch resources/launch/rr_plan.launch.py
```
#### Common

Since `cable_plan_executor` is a lifecycle node, run the following in a separate terminal for both simulation and real-robot cases to start plan execution:

```bash
ros2 lifecycle set /cable_plan_executor configure
ros2 lifecycle set /cable_plan_executor activate
```

