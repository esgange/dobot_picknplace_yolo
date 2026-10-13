# Remote Setup

`remote_setup` is the station's CATARM communication settings editor and read-only
connection tester. It has a Qt UI and a one-shot headless mode. Opening the UI
never connects; Save and Test are explicit. It does not enable remote control,
consume commands, publish heartbeats, create/bind queues, deploy teaching files,
or contact ROS robot services. Offline production does not depend on it.

After building and sourcing the workspace:

```bash
source scripts/source_ros_workspace.bash
ros2 launch remote_setup remote_setup.launch.py
# One test of the saved settings, then exit:
ros2 run remote_setup remote_setup --headless
# Equivalent launch form:
ros2 launch remote_setup remote_setup.launch.py headless:=true
```

The HMI has **Tools / Debug → Remote Setup → Open**, available in Manual and Auto,
including during a run. It owns only the window it opens and closes its own test
child on exit. Externally opened windows remain external. It is never auto-started.
Use `ros2 run ... --headless` when a script needs the test exit status: 0 means both
checks passed, 1 failed, 2 invalid configuration. A ROS launch wrapper's exit code
is not a substitute for the node's result.

## Settings

All settings live only in ignored root `.env`. The twelve recognized `REMOTE_*`
keys are optional to existing offline station readers. The example keeps them
commented so older running readers still see their original required key set. Existing installations do
not need migration. Remote Setup requires a complete valid configuration to test;
blank or missing required fields are errors, with no inferred server or credentials.
First-run fields show suggestions only as placeholders. Save validates before an
atomic replacement, preserves all unrelated lines, and sets file mode 0600 because
credentials are included. It refuses malformed/unknown/duplicate project keys and
detects intervening edits before replacement. Reload Saved explicitly discards edits.
The actual workstation `.env` is never changed by building/installing this package.

| Root `.env` key | Meaning |
| --- | --- |
| `REMOTE_SERVER_HOST` | Broker IP address or DNS hostname; no scheme |
| `REMOTE_AMQP_PORT` | Explicit AMQP port, usually 5672 or TLS 5671 |
| `REMOTE_AMQP_TLS` | `true` or `false`; TLS verifies host and system trust roots |
| `REMOTE_AMQP_VHOST` | Exact RabbitMQ virtual host, e.g. `/` |
| `REMOTE_AMQP_USERNAME` | Broker username |
| `REMOTE_AMQP_PASSWORD` | Broker password; masked in UI |
| `REMOTE_CONVEYOR_API_URL` | HTTP(S) conveyor-server base URL, including its port; no credentials/query/fragment |
| `REMOTE_BELT_ID` | Server belt ID, 1–64 letters/digits/underscores/hyphens |
| `REMOTE_ARM_NUMBER` | Station's CATARM arm number, 1–8 |
| `REMOTE_CONNECT_TIMEOUT_SEC` | Each check's deadline, 1–60 seconds |
| `REMOTE_INTERNAL_API_KEY` | Optional future artifact API credential; masked, stored but not sent/tested |
| `REMOTE_MESSAGE_SIGNING_KEY` | Optional future raw UTF-8 HMAC key; masked, stored but not sent/tested |

Values use literal strict `KEY=value` syntax, with no surrounding whitespace or
shell evaluation. An `=` inside a password is supported. Secrets are not passed
in process arguments, exported to the environment, or included in test diagnostics.
Save does not imply connection success. Unsaved edits must be saved or reloaded
before testing. Each test uses one frozen snapshot of the saved settings.

## What a test proves

1. A single AMQP login opens the configured vhost and passively checks the existing
   `catarm.events` exchange, then closes. It does not create an exchange, verify
   its type, inspect/consume station queues, publish, bind, or reconnect.
2. A GET to `<REMOTE_CONVEYOR_API_URL>/api/health` must return HTTP 200 with JSON
   `status=ok`, `service=conveyor-server`, and the configured `belt_id`. Response
   size is capped at 64 KiB. Redirects and environment proxies are disabled.

The CATARM reference is its `conveyor_server/services/rabbitmq.py` exchange and
`api/routes/health.py` endpoint (studied at commit `05983826`). Future routing is
`belt.<belt_id>.arm.<arm_number>`; displaying that prefix reserves no identity.
Signing keys, internal API permissions, arm uniqueness, teaching-schema compatibility,
command authorization and robot readiness are **not verified** by these checks.
The [aio-pika API](https://docs.aio-pika.com/apidoc.html) defines the passive exchange
check. No application-level command protocol is implemented in this version.

Both probes run in a disposable child. The UI stays responsive, Cancel/Close kills
only that child, and UI/headless enforce an overall `2 × timeout + 8 seconds` bound,
including process startup/cleanup. There are no retries or persistent connections.
Sanitized outcomes go to stdout/UI and capped 1,000-entry
`logs/remote_setup/events.jsonl`; passwords and server response bodies are omitted.

## Dependencies and validation

Uses ROS Humble `rclpy`, PyQt5, standard-library HTTP, and the pinned AMQP client in
`requirements.txt`. Install using the ROS `/usr/bin/python3` interpreter. For an
offline station, prepare and retain its matching wheel set on an online machine:

```bash
/usr/bin/python3 -m pip download -r src/remote_setup/requirements.txt -d /path/to/wheels
/usr/bin/python3 -m pip install --no-index --find-links /path/to/wheels -r src/remote_setup/requirements.txt
```

Missing AMQP dependencies produce an explicit failed AMQP check; the UI still
opens and the independent HTTP check still runs. Never download/install at launch.
Tests use temporary settings, fake broker/HTTP transports and harmless private child
processes. They do not contact a real server or robot. Build changes to
`remote_setup`, `orbbec_camera_launcher`, `item_perception_yolo`, `robot_controller`
and the updated bringup launch before using saved remote settings. When the station
is running, use isolated build/install trees; deploy canonically only after its
operator has stopped affected processes.
