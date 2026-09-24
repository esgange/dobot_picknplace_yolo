# dobot_rviz

`dobot_rviz` is the read-only RViz viewer for the actual Dobot CR10 and every
TF currently published on the local ROS 2 graph. It never starts Dobot
bringup, never publishes joint states, never substitutes zero positions, and
never provides manual joint sliders.

## Data path

```text
dobot_bringup_v4 /joint_states
             -> robot_state_publisher + fixed CR10 URDF
             -> /tf and /tf_static
             -> RViz RobotModel and TF displays
```

`robot_state_publisher` subscribes directly to the canonical `/joint_states`
topic. The viewer's monitor requires exactly one publisher on that topic: the
root-namespace node named by `DOBOT_ROBOT_NODE_NAME` in the canonical root
`.env`. The message must contain `joint1` through `joint6` in that exact order,
six finite positions, and a non-zero timestamp.

Startup hard-fails if a valid actual stream is not available within five
seconds. After startup, loss of the canonical publisher or a stream age greater
than one second terminates the viewer. Failures and lifecycle events are written
to `logs/dobot_rviz/events.jsonl` with UTC timestamps and the project-wide
1,000-event cap.

## Run

The safety-checked Dobot bringup now starts this viewer automatically:

```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

Closing RViz or a viewer failure stops the viewer and its robot TF publisher,
while the Dobot driver continues. Ctrl-C in the bringup terminal stops both;
driver exit also stops the owned viewer. No viewer process is restarted
automatically, and closing RViz does not command the robot to Stop.

To reopen the viewer after it has exited, run this in a second sourced terminal
while bringup is still publishing. Run only one viewer at a time to avoid
duplicate robot TF publishers:

```bash
cd ~/PicknPlace
source scripts/source_ros_workspace.bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

The launch has no arguments and uses the installed CR10 URDF and RViz
configuration only. The RViz fixed frame is `base_link`; the RobotModel display
shows the CR10 and the TF display shows the robot frames plus any additional TF
published by other local ROS nodes.

Useful read-only checks:

```bash
ros2 topic echo /joint_states --once
ros2 topic info /joint_states --verbose
ros2 run tf2_ros tf2_echo base_link Link6
```
