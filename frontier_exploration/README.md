# frontier_exploration

Package ```frontier_exploration``` is used for testing the algorithms in the simulation with mobile robots Pioneer P3-DX. 

## Table Of Contents

- [Installation](#Installation)
- [Basic usage](#BasicUsage)
  * [Running the simulation](#Running)

## <a name="Installation"></a> Installation
Detailed installation instructions can be found in [InstallationInstructions.md](https://github.com/larics/decentralised_multi_robot_exploration/blob/master/InstallationInstructions.md)

## <a name="BasicUsage"></a> Basic Usage
This section describes how to run autonomous decentralized exploration for two mobile robots Pioneer P3-DX (robot_0 and robot_1).

### <a name="Running"></a> Running the simulation
Run each command in a special terminal.

```roscore```

Before you run cartographer.launch, export ROBOT_NAME and then run launch file in the same terminal:

```export ROBOT_NAME=pioneer```

```roslaunch frontier_exploration cartographer.launch```

Cartographer.launch includes cartographer_occupancy_grid_node to crate a map and  2D simulator [Stage](http://wiki.ros.org/stage).

Run trajectories for the both robots:

```ROBOT_NAME=robot_0 roslaunch frontier_exploration add_trajectory.launch ```

```ROBOT_NAME=robot_1 roslaunch frontier_exploration add_trajectory.launch ```


Now, you can run [move_base node](http://wiki.ros.org/move_base):

```roslaunch frontier_exploration multi_robot_navigation.launch```

After running the commands above, the mobile robots are ready for navigation. You can move them by sending goals to the  [navigation stack](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack) with rviz.

You should also be able to crate a map of the environment with two mobile robots. 

To achive autonomous decentralized exploration, run filter.launch and multi_robot_assigner.launch:

```roslaunch frontier_exploration filter.launch```

```roslaunch frontier_exploration multi_robot_assigner.launch```

Make sure that you set the correct number of mobile robots (n_robots) in filter.launch and robot_assigner.launch.

