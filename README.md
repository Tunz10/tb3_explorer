# tb3_explorer

Autonomous **frontier-based exploration** for TurtleBot3 on ROS 2.
The robot drives into unknown space by itself and builds a map of the environment with SLAM Toolbox, using Nav2 to reach each goal.

![ROS 2](https://img.shields.io/badge/ROS_2-22314E?style=flat&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)

## How it works

1. **Read the map.** The node subscribes to the `/map` occupancy grid published by SLAM Toolbox.
2. **Find frontiers.** A frontier cell is a free cell next to at least one unknown cell, i.e. the edge between explored and unexplored space.
3. **Cluster them.** Neighbouring frontier cells are grouped with BFS; clusters smaller than `min_frontier_cluster` are ignored as noise.
4. **Pick a goal.** The centroid of the largest cluster is converted from grid to world coordinates.
5. **Navigate.** The goal is sent to Nav2's `navigate_to_pose` action. When it finishes, or gets stuck past `stuck_timeout_sec`, a new frontier is chosen.
6. **Stop.** When no frontiers are left, the map is complete.

## Requirements

- ROS 2 (developed on Humble)
- `turtlebot3_gazebo`, `slam_toolbox`, `nav2_bringup`

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Tunz10/tb3_explorer.git
cd ~/ros2_ws
colcon build --packages-select tb3_explorer
source install/setup.bash
```

## Run

Start the Gazebo world, SLAM Toolbox, Nav2 and the explorer together:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_explorer autonomous_mapping.launch.py
```

Or run only the explorer node next to your own SLAM + Nav2 setup:

```bash
ros2 run tb3_explorer explorer
```

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `map_topic` | `/map` | Occupancy grid topic |
| `goal_frame` | `map` | Frame used for navigation goals |
| `min_frontier_cluster` | `30` | Minimum cluster size (cells) to be a valid target |
| `goal_cooldown_sec` | `2.0` | Minimum time between two goals |
| `stuck_timeout_sec` | `40.0` | Cancel a goal and pick a new one after this long |
| `max_goals` | `999999` | Stop after sending this many goals |

## Roadmap

- Score frontiers by distance as well as size, so the robot doesn't cross the map for a slightly bigger target
- Blacklist frontiers that repeatedly fail
- Save the finished map automatically

## License

MIT
