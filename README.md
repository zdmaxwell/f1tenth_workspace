F1TENTH Autonomous Driving Bring-Up
===================================

This workspace contains two ROS 2 packages:

- `f1tenth_control`: provides the `slash_mpc` model predictive controller that consumes `/odom` and publishes `/ackermann_cmd`.
- `f1tenth_drive`: provides the `slash_twist_to_ackermann` teleop helper plus the `twist_to_ackermann.launch.py` convenience launcher.

Use the guide below whenever you need to bring the stack up.

## 1. Build (first time or after code changes)

```bash
cd ~/f1tenth_ws
source /opt/ros/humble/setup.zsh      # or setup.bash if you use bash
colcon build --event-handlers console_direct+
source install/setup.zsh
```

> Tip: after the first build you only need to source `/opt/ros/.../setup.(z)sh` and `install/setup.(z)sh` in each new terminal before launching nodes.

## 2. Launch the Slash MPC node

1. Open terminal #1 and source the same two setup files shown above.
2. Ensure the centerline file referenced in `src/f1tenth_control/src/slash_mpc.cpp` exists (`~/f1tenth_ws/bag_files/teleop/extracted_data/centerline_drive_data_0502_1050.csv` by default).
3. Start the controller:

   ```bash
   ros2 run f1tenth_control slash_mpc
   ```

   - Subscribes to `/odom` (e.g., from Isaac Sim or state estimation).
   - Publishes `ackermann_msgs/AckermannDriveStamped` on `/ackermann_cmd`.
   - Logs will confirm the centerline file load and node startup.

Keep this terminal running so the MPC continues to send commands.

## 3. Launch Twist→Ackermann translator (for teleop)

1. Open terminal #2, source the same setup files.
2. Launch the node via the provided launch file so you can override parameters at runtime:

   ```bash
   ros2 launch f1tenth_drive twist_to_ackermann.launch.py \
     twist_subscribe_topic:=/cmd_vel \
     ackermann_publish_topic:=/ackermann_cmd \
     track_width:=0.325 \
     max_speed:=3.0 \
     max_steering_angle:=0.523 \
     publish_period_ms:=20
   ```

   - The launch file simply declares these arguments and then starts the `slash_twist_to_ackermann` executable from `src/f1tenth_drive/src/slash_twist_to_ackermann.cpp`.
   - Default values already match the code (20 ms publish period, `/cmd_vel` input, `/ackermann_cmd` output). Override them on the command line if you change the hardware or topic names.
   - The node listens for `geometry_msgs/msg/Twist` (teleop, autonomy planner, etc.) and publishes `ackermann_msgs/msg/AckermannDriveStamped` that Isaac Sim or the real car can use.

If you only want MPC control you can skip this section; do not run two nodes that both drive `/ackermann_cmd` simultaneously.

## 4. Drive and verify

- Send `Twist` commands however you like (e.g., `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}'` for quick testing).
- Monitor traffic from either node:

  ```bash
  ros2 topic list
  ros2 topic echo /ackermann_cmd
  ros2 node list
  ```

- If `/ackermann_cmd` is quiet, check that both terminals are sourced correctly and that your `/odom` or `/cmd_vel` sources are active.

That’s it—the README now serves as a bring-up checklist so you can reproduce the same workflow later without digging back through the code.

## 5. Visualize the MPC waypoints in RViz2

The `slash_mpc` node publishes two marker topics (`/mpc/centerline_points` and `/mpc/polyfit`) so you can confirm the controller is fitting the track correctly.

1. Ensure RViz2 is installed (it is part of the standard ROS 2 Desktop install; otherwise run `sudo apt install ros-humble-rviz2` or replace `humble` with your distro name).
2. In a new terminal source both `/opt/ros/<distro>/setup.(z)sh` and `~/f1tenth_ws/install/setup.(z)sh`.
3. Start RViz: `rviz2`.
4. Set the fixed frame to `base_link` (the markers are published in the vehicle frame).
5. Add two “Marker” displays:
   - Topic `/mpc/centerline_points` — shows the ego-frame centerline samples (green points).
   - Topic `/mpc/polyfit` — shows the 3rd-order polynomial the MPC tracks (red line strip).
6. Run Isaac Sim/the MPC node and confirm the green points align with the actual track and the red fit hugs those points. If nothing appears, verify the topics exist (`ros2 topic list`) and the node is receiving `/odom`.

Using RViz while you tune the controller helps catch issues like wrong waypoint transforms or polynomial fits before they show up in Isaac Sim as erratic steering.
