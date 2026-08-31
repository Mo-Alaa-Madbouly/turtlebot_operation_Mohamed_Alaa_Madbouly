# turtlebot_operation_Mohamed_Alaa_Madbouly
ROS2 Arabic - Assignment 3 Part 1

Obstacle avoidance and manual override system for a TurtleBot in ROS2 Jazzy.
The robot autonomously avoids obstacles (forward → turn → reverse) using LiDAR data,
and an operator can override its movement at runtime via a service call.

## Packages

- **obstacle_direction_interfaces** — defines the `SetDirection.srv` service used for manual override.
- **obstacle_direction_controller** — contains `direction_autopilot_node.py`, which subscribes to `/scan`,
  publishes to `/cmd_vel`, and hosts the `/set_direction` service.

## 1. Setup Instructions

Clone this repo into the `src` folder of a colcon workspace:

\`\`\`bash
mkdir -p turtlebot_operation_ws/src
cd turtlebot_operation_ws/src
git clone <your-repo-url>
cd ..
colcon build
source install/setup.bash
\`\`\`

## 2. ROS2 Commands Used

| Command | What it does |
|---|---|
| `colcon build` | Builds all packages in the workspace (compiles the interface `.srv` and installs the Python node). |
| `source install/setup.bash` | Loads the built packages into the current shell so `ros2 run` can find them. |
| `ros2 run obstacle_direction_controller direction_autopilot_node` | Starts the autopilot node: subscribes to `/scan`, publishes `/cmd_vel`, hosts `/set_direction`. |
| `ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'left'}"` | Manually overrides the robot's movement for a few seconds, then autonomous control resumes. |
| `ros2 topic echo /cmd_vel` | Shows the live velocity commands being published, for debugging. |
| `ros2 node list` | Confirms `direction_autopilot_node` is running. |

## 3. How to Test the Nodes

1. Launch a TurtleBot simulation (e.g. Gazebo) so `/scan` and `/cmd_vel` exist.
2. In a sourced terminal, run:
   \`\`\`bash
   ros2 run obstacle_direction_controller direction_autopilot_node
   \`\`\`
3. In a second sourced terminal, send an override:
   \`\`\`bash
   ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'forward'}"
   \`\`\`
4. Try an invalid direction to confirm validation works:
   \`\`\`bash
   ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'up'}"
   \`\`\`

## 4. Expected Output

- Node startup log: `direction_autopilot_node started. State: FORWARD`
- While driving toward an obstacle: a `OBSTACLE: Front X.XX m <= 0.50 m` warning, then `State: FORWARD -> REVERSE`.
- After reversing: `State: REVERSE -> TURN (left)` or `(right)`, whichever side has more clearance.
- Once the front is clear again: `State: TURN -> FORWARD`.
- On a valid service call: response `success: True` and a `MANUAL OVERRIDE -> <direction>` log line.
- On an invalid service call: response `success: False` with a message listing the accepted directions.

