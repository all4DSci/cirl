PI Controller with CBF (fake-robot test)
=====================================

This small test setup lets you run `pi_controller_cbf.py` together with `fake_robot.py`.
`fake_robot.py` subscribes to `/cmd_vel_unstamped` and publishes `/odom` by integrating
the commanded velocities. `pi_controller_cbf.py` reads parameters for the goal and
for a circular obstacle and will apply a simple CBF-based safety filter.

Quick start (in two terminals):

Terminal 1 - run the fake robot:

```bash
ros2 run ros2 run-or-launch ???
# or simply:
python3 fake_robot.py
```

Terminal 2 - run the controller (example parameters):

```bash
# You can pass params using ROS2 CLI --ros-args -p
python3 pi_controller_cbf.py --ros-args -p goal_x:=1.0 -p goal_y:=-0.5 -p obs_x:=0.5 -p obs_y:=0.0 -p obs_R:=0.2
```

Notes:
- The `fake_robot.py` is a minimal integrator, not a physics-accurate simulator. Use it only
  to visualize controller behavior and tune parameters.
- If you use a ROS2 launch system, convert these to nodes in a launch description.
