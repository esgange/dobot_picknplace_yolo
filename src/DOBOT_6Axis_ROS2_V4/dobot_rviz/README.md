# dobot_rviz

`dobot_rviz` provides the fixed CR10 model, headless robot-TF launch and optional
read-only RViz viewer. Canonical Dobot bringup owns the TF session; the separate
viewer launch starts only RViz. Neither launch starts the hardware driver,
publishes joint states, substitutes zero positions or provides joint sliders.

## Data path

```text
dobot_bringup_v4 launch
  -> driver -> /joint_states
  -> owned robot_state.launch.py
       -> actual-joint monitor
       -> robot_state_publisher + fixed CR10 URDF -> /tf and /tf_static
                                                    -> headless perception
                                                    -> optional RViz viewer
```

`robot_state_publisher` subscribes directly to the canonical `/joint_states`
topic. The headless monitor requires exactly one publisher on that topic: the
root-namespace node named by `DOBOT_ROBOT_NODE_NAME` in the canonical root
`.env`. The message must contain `joint1` through `joint6` in that exact order,
six finite positions, and a non-zero timestamp.

Startup hard-fails if a valid actual stream is not available within five
seconds. After startup, loss of the canonical publisher or a stream age greater
than one second terminates the TF session while the driver stays available.
Failures and lifecycle events are written
to `logs/dobot_rviz/events.jsonl` with UTC timestamps and the project-wide
1,000-event cap.

## Run

The safety-checked Dobot bringup starts the driver, joint monitor and robot TF,
without RViz or any display requirement:

```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

Ctrl-C in the bringup terminal or driver exit stops its owned TF session.
Monitor or TF-publisher exit stops both TF-session nodes without stopping the
driver or restarting anything. This failure is reported in the bringup terminal.

Open the viewer separately in a sourced terminal while bringup is publishing:

```bash
cd ~/PicknPlace
source scripts/source_ros_workspace.bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```

The viewer launch has no arguments and uses the installed RViz configuration.
It owns only the RViz process: closing it, a display failure or reopening it
never affects robot TF, joint monitoring or the driver, and never duplicates a
TF publisher. It sends no robot commands. Reopening the viewer cannot recover a
failed TF session. `robot_state.launch.py` is owned by bringup; do not launch a
second copy alongside it. Stop the old bringup/viewer before switching from the
former combined viewer launch, then rebuild both packages and relaunch manually.

The RViz fixed frame is `base_link`; the RobotModel display
shows the CR10 and the TF display shows the robot frames plus any additional TF
published by other local ROS nodes.

The optional **Robot camera - 90 x 25 x 30 mm body** Marker display subscribes
to `/item_teach/robot_camera_body`. The separate read-only
`item_perception_yolo/robot_camera_box` node, started by Item Teach launch,
draws a translucent magenta box from Item Teach's validated calibration selection
with its center (+11, 0, −12.79) mm from the RGB optical origin and XYZ sizes
90/25/30 mm in RGB optical axes. It follows live Link6 TF using the same saved
mounting pose, nominal RGB-to-link bridge and offset body as camera
clearance planning. It publishes no TF or hardware commands. Selection changes,
invalid calibration or missing Item Teach updates hide the box. Closing Item
Teach stops its display process; stopping that node expires its marker after
three seconds. Controller launch does not start it. Reload this
configuration or add that Marker topic to an already-open RViz window.

The bundled configuration also subscribes to Item Teach's optional 1 Hz
`/item_teach/voxel_cloud` (colored 10 mm boxes) and `/item_teach/valid_items`
(validated pose guides without text/number overlays). These displays stay empty until
Item Teach has calibrated inputs; the viewer does not start a detector, load
models or save images. Candidate frames use `item_teach_live_candidate_N`.
TF frames expire after 2.5 seconds without updates. This is an intentional
project integration patch to the vendored RViz configuration only.
The cloud's **Decay Time = 0** keeps the latest cloud until replacement.
Matching **Reliable**, **Transient Local**, **Keep Last**, **Depth = 1** QoS
supplies the cached cloud even when RViz opens after its publication, while Item
Teach runs. Item Teach turns it grey after five seconds without new validated
voxel data, or immediately on invalidation or orderly exit. A fresh snapshot
replaces it and restores its colors; repeated/frozen frames cannot reset its age.
Reload the canonical RViz configuration to apply this QoS to an existing viewer.
Use **Size (m) = 0.010** when updating an already-open RViz display manually.

A separate enabled **Tray Teach - 10 mm colored voxels** display subscribes to
`/tray_teach/voxel_cloud` with the same 10 mm boxes, zero decay and retained-cloud
QoS. Tray Teach produces it from calibrated synchronized RGB/depth/TF even with
YOLO off, retains its source timestamp, and greys it after five seconds without
a fresh replacement or immediately on invalidation/exit. It needs no taught
tray plane. Only the selected tray has a TF (`tray_teach_selected_tray`), and
there are no numeric tray overlays. This is a project viewer integration patch;
the viewer starts no teaching nodes and sends no hardware commands.

A separate enabled **Tray Detect - 10 mm colored voxels** display subscribes to
`/tray_detect/voxel_cloud` with identical size, retention and QoS. Headless Tray
Detect refreshes the calibrated scene at up to 1 Hz without background YOLO or
tray-pose TF; pose inference remains request-driven. Reload this installed
configuration via **File → Open Config** in an already-running RViz to add the
display. Cloud status is available on `/tray_detect/rviz_diagnostics`.

The enabled **XY and blue UP guides** displays retain true red X/green Y and show
the blue reference-plane normal facing positive `base_link` Z. In addition to
live `/item_teach/valid_items`, subscribe to `/item_teach/selected_pose_guides`
for clicked/simulated items and `/tray_teach/pose_guides` for the selected tray.
All guides are 200 mm long with 20 mm shafts and 40 mm arrow heads (diameter and
length). Their length and shaft width match RViz's robot-joint TF axes at the
canonical **Marker Scale = 1**. Changing the TF display's scale manually affects
only raw TF axes; it does not resize separately published guide markers.
These arrows do not change any actual TF or controller pose. The canonical TF
display now sets **Show Axes = false** so its original downward axes do not
overlap the upward guides. This also hides robot-frame axes; the RobotModel and
TF data remain available. Enable **TF → Show Axes** explicitly to inspect the
real coordinate frames. No numeric labels are added. Guides
clear with invalidation and expire after 2.5 seconds without publication.
For an already-open viewer, uncheck **TF → Show Axes**, or use **File → Open
Config** to reload the installed canonical configuration; no node restart is needed.

Useful read-only checks:

```bash
ros2 topic echo /joint_states --once
ros2 topic info /joint_states --verbose
ros2 run tf2_ros tf2_echo base_link Link6
```
