# moc-experiments-ros2

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

## Experiment setups

### Simulation

Run the following launch file to create an interactive environment with simulated robots that can be assigned targets via clicking on rviz2:

```bash
ros2 launch resources/launch/sim_click.launch.py
```

Run the following launch file to make robots execute planned trajectories in simulation:

```bash
ros2 launch resources/launch/sim_plan.launch.py
```

### Real-Robot

Run the following launch file to create an interactive environment that visualizes the real robots which can also be assigned targets via clicking on rviz2:

```bash
ros2 launch resources/launch/rr_click.launch.py
```

Run the following launch file to make real robots execute planned trajectories:

```bash
ros2 launch resources/launch/rr_plan.launch.py
```
