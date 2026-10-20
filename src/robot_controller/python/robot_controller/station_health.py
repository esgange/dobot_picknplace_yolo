"""Read-only station health presentation; discovery alone never means healthy."""

from dataclasses import dataclass


@dataclass(frozen=True)
class HealthRow:
    key: str
    label: str
    state: str
    detail: str


def health_rows(status, nodes, camera, tf_health, *, initialized=True, manual=False,
                loading='OFF', dependency_failure='', preview_available=False,
                driver_name='dobot_bringup_ros2'):
    names = [name for name, _namespace in nodes]
    fresh = status is not None and status.feedback_fresh

    def observed(key, label, node, ready=False, detail='', namespace='/'):
        if not initialized:
            return HealthRow(key, label, 'NOT STARTED', 'Press Initialize Station')
        count = names.count(node)
        state = ('CONFLICT' if count > 1 else 'HEALTHY' if count == 1 and (node, namespace) in nodes and ready else
                 'UNVERIFIED' if count == 1 else 'MISSING')
        return HealthRow(key, label, state, detail or
                         'Node discovery alone does not establish live readiness')

    rows = [
        observed('driver', 'Robot driver', driver_name, fresh,
                 dependency_failure or 'Health requires fresh canonical controller feedback'),
        observed('feedback', 'Robot feedback', 'dobot_actual_joint_state_monitor', fresh,
                 'Joint / robot / FeedInfo coherence is checked by the controller'),
        observed('tf', 'Robot TF', 'dobot_robot_state_publisher', tf_health[0],
                 tf_health[1]),
        observed('controller', 'Controller', 'robot_controller', status is not None,
                 'Stopped for maintenance' if manual else
                 'Controller status must be newer than one second'),
        observed('preview', 'Motion preview', 'robot_controller_preview',
                 preview_available,
                 'Canonical preview service available; target eligibility is checked per request'),
    ]
    if manual and initialized:
        rows[3] = HealthRow('controller', 'Controller', 'MAINTENANCE', rows[3].detail)
        rows[4] = HealthRow('preview', 'Motion preview', 'MAINTENANCE', 'Production core stopped')
    if status is not None and not fresh and initialized:
        rows[1] = HealthRow('feedback', 'Robot feedback', 'STALE', status.motion_block_reason)
    camera_reason = camera.unavailable() if camera is not None else 'No camera health received'
    camera_ready = bool(camera is not None and camera.seen and not camera_reason)
    rows.append(observed('supervisor', 'Camera supervisor', 'supervisor', camera_ready,
                         camera_reason or 'Fresh advancing diagnostics for both configured serials',
                         namespace='/camera_watchdog'))
    if camera is not None:
        for name, state, detail in camera.health_rows():
            rows.append(HealthRow(name, name.split('/')[-1],
                                  state if initialized else 'NOT STARTED',
                                  detail if initialized else 'Press Initialize Station'))
    for kind in ('item', 'tray'):
        providers = [name for name in names if name in (kind + '_detect', kind + '_teach')]
        ready = status is not None and getattr(status, kind + '_detector_ready', False)
        state = ('NOT STARTED' if not initialized else 'CONFLICT' if len(providers) > 1 else
                 'SERVICE READY' if len(providers) == 1 and ready else
                 'STARTING' if loading in ('DEPLOYING', 'STARTING', 'PREPARING') else
                 'UNVERIFIED' if providers else 'NOT LOADED')
        rows.append(HealthRow(kind, kind.title() + ' perception', state,
                              'Provider and private worker start with Remote Auto or Load Auto; every request '
                              'revalidates live images, depth, transforms and recipe sources'))
    return rows


def readiness_errors(rows, *, perception=True):
    return [f'{row.label}: {row.state.lower()}'
            for row in rows if (perception or row.key not in ('item', 'tray'))
            and row.state not in ('HEALTHY', 'SERVICE READY')]
