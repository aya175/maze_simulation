# Maze Solver

ROS 2 package that drives a TurtleBot3 through a Gazebo maze using custom actions, odometry, and the maze wall service.

It defines `MoveRobotX` and `MoveRobotYaw`, runs the matching action servers, and executes `solve_maze()` from `maze_client.py`.

## Requirements

- ROS 2 (Humble or later recommended)
- A sourced workspace that already includes the maze simulation (`maze_control`)
- TurtleBot3 Gazebo maze launch available as:

  ```bash
  ros2 launch maze_control maze_simulation_tb3.launch.py
  ```

## Layout

```
maze_solver/
  action/                 Custom action definitions
  launch/                 Launch files
  maze_solver_py/         Python nodes
  CMakeLists.txt
  package.xml
```

| File | Role |
| --- | --- |
| `action/MoveRobotX.action` | Drive a distance in metres (negative = reverse) |
| `action/MoveRobotYaw.action` | Rotate a relative yaw in radians |
| `maze_solver_py/action_servers.py` | Action servers: publish `/cmd_vel`, track `/odom` |
| `maze_solver_py/maze_client.py` | Wall control + `solve_maze()` sequence |
| `maze_solver_py/odom_watcher.py` | Odometry pose and stale-sensor checks |

## Build

From your colcon workspace (this repo as `src/maze_solver` or the folder that contains `maze_solver/`):

```bash
cd ~/your_ws
colcon build --packages-select maze_solver
source install/setup.bash
```

On Windows (PowerShell), source the matching `setup.ps1` after build.

## Run

Start the maze simulation first, then this package.

**Terminal 1 — Gazebo maze**

```bash
ros2 launch maze_control maze_simulation_tb3.launch.py
```

**Terminal 2 — action servers + solver**

```bash
ros2 launch maze_solver maze_solve.launch.py
```

That launch starts the action servers immediately and, after a short delay, the maze client (default `run_client:=true`).

Servers only (then run the client yourself):

```bash
ros2 launch maze_solver maze_solver_servers.launch.py
ros2 run maze_solver maze_client.py
```

Useful launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `cmd_vel_topic` | `/cmd_vel` | Velocity command topic |
| `odom_topic` | `/odom` | Odometry topic |
| `linear_speed` | `0.18` | Forward speed (m/s) |
| `angular_speed` | `0.7` | Yaw speed (rad/s) |
| `run_client` | `true` | Start `maze_client` from `maze_solve.launch.py` |

Example:

```bash
ros2 launch maze_solver maze_solve.launch.py run_client:=false linear_speed:=0.15
```

## What `solve_maze()` does

1. Wait for `move_robot_x`, `move_robot_yaw`, and `/toggle_walls_1_2`.
2. Turn \(+\pi/2\) (left).
3. Raise wall 1 fully, hold ~6 s, drive **1.05 m**, then lower both walls.
4. Raise wall 2 fully, hold ~6 s, drive **1.10 m**, then lower both walls.
5. Turn \(-\pi/2\) (right).
6. Drive **4.55 m** to the exit.

Walls are commanded to joint position **2.0** (up) and **0.0** (down). The client also publishes hold commands on `/wall_1/cmd_pos` and `/wall_2/cmd_pos` so one wall stays up while the other stays down.

## Custom actions

**`MoveRobotX`** (`move_robot_x`)

- Goal: `distance` (m)
- Feedback: `distance_travelled`, `distance_remaining`
- Result: `success`, `message`, `final_distance_travelled`

**`MoveRobotYaw`** (`move_robot_yaw`)

- Goal: `target_yaw` (rad, relative; positive = CCW)
- Feedback: `yaw_turned`, `yaw_remaining`
- Result: `success`, `message`, `final_yaw_turned`

Servers abort if `/odom` never appears, goes silent, or the motion times out. Speed tapers near the target so the robot does not overshoot.

## License

MIT — see `maze_solver/package.xml`.
