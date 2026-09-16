# Robot Controller v2

`robot_controller` is the sole production application-level authority for the
physical CR10. It provides two deterministic operations: Home and Pick Item.
It does not launch Dobot bringup, cameras, Item Detect, or RViz.

Launching the package never enables, recovers, homes, or moves the robot. An
operator or supervisor must load configuration and make an explicit Startup
service call before a hardware action can be accepted.

## Processes

- `robot_controller` is headless hardware authority. It alone creates Dobot
  motion, Pause/Continue/Stop, robot-setting, and gripper-output clients.
- `robot_controller_preview` calculates and broadcasts TF-only Home/Pick plans.
  It has no Dobot command client and cannot actuate the robot.
- `robot_controller_gui` is a client of the controller and preview APIs. It has
  no Dobot command client.

The normal launch starts all three:

```bash
ros2 launch robot_controller robot_controller.launch.py
```

Headless mode starts only the hardware controller and loads exactly one Item
Teach YAML/paired `.pt` plus one Bin Teach YAML from the flat root
`runtime_teach/` directory. It still remains `INACTIVE` until Startup:

```bash
ros2 launch robot_controller robot_controller.launch.py headless:=true
```

Canonical Dobot bringup and Item Detect are separate processes. Do not run the
maintenance `motion_debug` or `gripper_control` application alongside
production Startup; their presence is rejected as competing command ownership.

## Typed API

Actions:

- `/robot_controller/go_home` — goal contains `configuration_id`.
- `/robot_controller/pick_item` — goal contains `configuration_id` and the
  one-shot `save_debug_images` flag. Candidate count cannot be supplied by the
  caller; it comes from Item Teach `retry.pose_candidates`.

Services:

- `/robot_controller/configure` loads explicit Item/Bin Teach in GUI mode.
- `/robot_controller/startup` performs the deterministic cold Startup sequence.
- `/robot_controller/recover` stops/clears/enables/restores settings without
  moving Home.
- `/robot_controller/pause` pauses and confirms the current Dobot queue without
  canceling the active Home/Pick action.
- `/robot_controller/continue` resumes only a confirmed `PAUSED` queue.
- `/robot_controller/stop` pre-empts and confirms Stop while preserving outputs.
  It is always direct and never requires a preceding Pause.
- `/robot_controller/set_global_speed` accepts an integer 1–100 only while
  stationary in `READY` or `HOLDING`.
- `/robot_controller/preview` belongs to the TF-only preview node.

`/robot_controller/status` uses
`robot_controller_interfaces/msg/ControllerStatus`, reliable/transient-local
QoS. It reports state, phase, waypoint, candidate index, configuration ID,
holding context, Startup completion, global factor, and feedback freshness.
The removed Trigger/JSON/Live/Enable/validation/pose-proxy/debug-image endpoints
have no compatibility wrappers.

Example CLI flow for a non-headless controller:

```bash
ros2 service call /robot_controller/configure \
  robot_controller_interfaces/srv/Configure \
  "{item_teach_file: '/absolute/item.yaml', bin_teach_file: '/absolute/bin.yaml'}"
ros2 service call /robot_controller/startup \
  robot_controller_interfaces/srv/Command '{}'
ros2 topic echo --once /robot_controller/status \
  robot_controller_interfaces/msg/ControllerStatus
ros2 action send_goal /robot_controller/go_home \
  robot_controller_interfaces/action/GoHome \
  "{configuration_id: '<exact status configuration_id>'}" --feedback
ros2 action send_goal /robot_controller/pick_item \
  robot_controller_interfaces/action/PickItem \
  "{configuration_id: '<exact status configuration_id>', save_debug_images: false}" \
  --feedback
ros2 service call /robot_controller/pause \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/continue \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/stop \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/recover \
  robot_controller_interfaces/srv/Command '{}'
ros2 service call /robot_controller/set_global_speed \
  robot_controller_interfaces/srv/SetGlobalSpeed '{percent: 50}'
```

## State and safety contract

The states are `UNCONFIGURED`, `INACTIVE`, `STARTING`, `READY`, `HOMING`,
`PICKING`, `HOLDING`, `PAUSED`, `STOPPING`, `RECOVERY_REQUIRED`, `RECOVERING`,
`HELD_UNKNOWN`, and `FAULT`. One immutable configuration snapshot and one
operation generation exist at a time. Action configuration IDs prevent a stale
GUI or supervisor from executing a replaced profile.

Startup validates sole canonical services and publishers, then performs:

1. best-effort `StopMoveJog`;
2. strict `Stop` with stationary, empty queue confirmation (a latched pause flag
   does not invalidate a confirmed Stop);
3. cold DI1 check (active DI1 preserves I/O and enters `HELD_UNKNOWN`);
4. Disable and conditional ClearError;
5. Enable and confirm fresh enabled mode. A latched `isPauseCmdFlag` does not
   block READY when mode 5 is enabled and the queue is empty/not running;
6. SpeedFactor 100, User 0, Tool 0, Tool 1 TCP zero, and CP 100;
7. DO1/DO2/DO13/DO14 reset only when no item is held;
8. 200 ms of coherent `READY` feedback.

Recover repeats Stop/error-clear/enable/settings/readiness but never moves Home.
It retains the last confirmed global factor. Trusted holding recovery preserves
suction/finger outputs and verifies DI1. Cold-start DI1 remains `HELD_UNKNOWN`;
after the operator physically resolves it, another explicit Stop observes DI1
clear and changes the state to `RECOVERY_REQUIRED`, where Recover is allowed.

Native action cancellation and Stop invalidate the active command generation,
use the independent Stop client, wait for acknowledgement and 300 ms stationary
empty-queue feedback, and preserve every gripper output. They never Home,
release, disable, or resume a discarded queue. The result is
`RECOVERY_REQUIRED`. A late motion acknowledgement causes another Stop;
unconfirmed stopping is `FAULT`. Held-item DI1 and expected-output integrity are
checked throughout Stop and Recover. Unexpected running/nonempty queue feedback
while otherwise idle is immediately routed through the same Stop confirmation
path rather than merely changing the state label.

Pause is different from Stop. From `READY`, `HOLDING`, `HOMING`, or `PICKING`,
it waits for the canonical Pause response plus pause-flagged, stationary
feedback, enters `PAUSED`, and retains the queued path and active action context.
No later host-side waypoint or I/O command is dispatched while paused. Continue
is accepted only from that confirmed state; it waits for the canonical response
and three fresh cleared-pause samples before restoring the suspended state.
Feedback, held suction, and expected outputs remain supervised. An ambiguous
Pause/Continue is contained by direct Stop. Intentional pause duration is not
charged to sensor, no-progress, arrival, or hard-motion deadlines.

`isPauseCmdFlag` is contextual telemetry rather than a global command gate.
Live evidence showed `EnableRobot()` latching it to one with an idle empty queue,
and a raw `Continue()` returned `-1`. Only a controller-issued, acknowledged
Pause with retained operation context enters `PAUSED`; an idle latch never does.

The GUI SpeedFactor slider tracks the handle position and sends it once on
release. Keyboard and groove changes are debounced for 350 ms. Controller status
cannot overwrite an active or pending edit, and unchanged selections do not send
another SpeedFactor request.

The GUI presents these services as two dynamic controls. `START` calls Startup
from `INACTIVE` and becomes `CONTINUE` in `PAUSED`. The amber `PAUSE` control
immediately becomes red `STOP` on its first click. A rapid second click records
a Stop request locally, waits for the Pause response, and then calls direct
Stop—never overlapping the two vendor requests. External nodes do not use this
two-click policy and may call `/robot_controller/stop` immediately in any state.

Feedback is condition-driven from the approximately 100 Hz FeedInfo stream.
Policies are: five seconds for service discovery/response, one-second feedback
age, two-second expected mode changes, three consistent pause/error samples,
three-second no-progress watchdog, and 300-second physical-motion cap. Home
arrival is within one degree on every taught joint; Cartesian arrival is within
5 mm and one degree, plus enabled, queue-idle and stationary confirmation.

The bridge publishes `RobotStatus.is_enable` as `robot_mode == 5`; it is an
idle-mode alias on a separate, slower publisher rather than an independent
enable latch. Fresh RobotStatus remains mandatory for connection/liveness, and
idle READY/final-arrival checks require its Boolean to converge. Active motion
uses authoritative FeedInfo `EnableStatus`, mode, error/collision, user/tool and
queue fields so an asynchronous status sample cannot cancel an accepted move.

Every actual canonical Dobot call has paired audit output in the ROS console and
`logs/robot_controller/events.jsonl`. Each `SEND` and terminal accepted,
rejected, timed-out, canceled, errored or late-response record contains a
process-local request ID, exact endpoint/request fields, ROS response `res`,
available `robot_return`, and elapsed milliseconds. This covers Startup/Recover
settings, GetPose, DO, all three motion services, Pause, Continue, and the
independent Stop channel. A successful response is still only command acceptance;
fresh robot feedback remains required for completion.

## Home and Pick

Home is permitted from `READY` and trusted `HOLDING`. Fresh GetPose selects the
branch: below taught Home Z, `RelMovLUser` first rises at current XY/attitude;
at/above it, that segment is skipped. Exact taught Home joints are then sent via
joint-mode `MovL`. Holding Home preserves and monitors suction.

Pick is permitted only from `READY` with DI1 clear:

1. run the same Home function;
2. request one fresh profile/model/camera/platform/bin-hash-matched batch from
   `/item_detect/get_item_poses`;
3. transform platform-relative targets into base coordinates;
4. attempt up to Item Teach `pose_candidates` in detector rank order;
5. after every miss, retract and return Home before advancing;
6. after success, retract and return Home holding with suction on.

The schema-6 geometry is unchanged: pick Z is item Z plus `standoff_height`,
pre-pick adds `prepick_height`, and clearance adds `retract_height`. Home/travel
uses taught travel rates, final descent uses approach rates, and pick-to-prepick
uses retract rates. Enabled fingers open at 50% of the clearance move. Suction
turns on at 0% of final descent. With `grip_onpick`, fingers close after DI1;
otherwise they close at the end of retract-to-prepick, still only after DI1.

No-I/O targets use `MovL`. `MovLIO` is used only for a real non-empty timed DO
tuple. Conditional Home rise uses `RelMovLUser`. The controller never calls
`InverseKin`; `Continue` is reserved solely for explicit resume from `PAUSED`.
Service acknowledgement is acceptance only; actual
feedback confirms every result. Only coherent missed suction advances to the
next candidate. All command, feedback, state, cancellation, and result events
are written to ignored `logs/robot_controller/events.jsonl`, capped at 1,000.

Software tests use synthetic services/feedback and must never commission
physical motion. Real commissioning requires separate explicit authorization,
clear workspace, functional physical emergency stop, verified wiring, and an
attentive operator.

`robot_controller` is the sole production/runtime Dobot command issuer.
`motion_debug` may issue direct commands only as a mutually exclusive maintenance
application; production Startup rejects competing maintenance command owners.
