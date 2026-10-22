"""One explicit local-mode selection owns one bounded Configure request."""

import time

from .events import PackageEventLogger


class LocalPreparation:
    def __init__(self, root):
        self.events = PackageEventLogger(root, 'robot_controller_local_preparation')
        self.state = 'OFF'
        self.message = ''
        self.paths = ()
        self.deadline = 0.
        self.configuration_id = ''
        self.awaiting_reply = False
        self.stop_needed = False

    @property
    def busy(self):
        return self.state in ('WAITING', 'CONFIGURING', 'CONFIRMING')

    def transition(self, state, message):
        self.state, self.message = state, message
        self.events.record('ERROR' if state == 'FAILED' else 'INFO',
                           'local_preparation_' + state.lower(), message)

    def request(self, paths):
        if self.busy or self.awaiting_reply or self.stop_needed:
            raise ValueError('Wait for the previous local preparation to finish')
        self.paths = tuple(paths)
        self.configuration_id = ''
        if len(self.paths) != 3 or not all(self.paths):
            self.transition('FAILED', 'Select Item Teach, Bin Teach and Tray Teach, then '
                            'click Local / Maintenance to load and prepare the robot')
            return
        self.deadline = time.monotonic() + 180.
        self.transition('WAITING', 'Waiting for station health before loading selected teach files')

    def fail(self, message):
        self.stop_needed |= self.state in ('CONFIGURING', 'CONFIRMING')
        self.transition('FAILED', message)

    def cancel(self):
        if self.busy:
            self.stop_needed |= self.state in ('CONFIGURING', 'CONFIRMING')
            self.transition('CANCELED', 'Local preparation canceled; select Local / Maintenance '
                            'again when ready')

    def poll(self, errors, status, send):
        if not self.busy:
            return
        if time.monotonic() >= self.deadline:
            self.fail('Local preparation timed out: ' + ('; '.join(errors) or self.message))
            return
        if self.state == 'WAITING':
            if errors:
                self.message = 'Waiting for local preparation: ' + '; '.join(errors)
                return
            # Claim before dispatch; neither a refresh nor a late response may resend.
            self.transition('CONFIGURING', 'Loading selected teach files and preparing/enabling robot')
            self.deadline = time.monotonic() + 180.
            self.awaiting_reply = True
            try:
                if not send(self.paths):
                    self.awaiting_reply = False
                    self.fail('Configure service unavailable; select Local / Maintenance to retry')
            except Exception as exc:
                self.awaiting_reply = False
                self.fail(str(exc))
        elif self.state == 'CONFIRMING' and not errors and status is not None:
            if (status.state == 'READY' and status.configuration_id == self.configuration_id
                    and status.configured and status.startup_complete and status.robot_enabled):
                self.transition('READY', 'Local / Maintenance ready; teaching and robot controls available')

    def response(self, result=None, *, error=''):
        if not self.awaiting_reply:
            return
        self.awaiting_reply = False
        accepted = result is not None and result.success
        if self.state != 'CONFIGURING':
            # Cancellation/timeout cannot adopt an accepted late preparation.
            self.stop_needed |= accepted
            return
        if error or not accepted or not result.configuration_id:
            self.fail(error or ('No Configure response' if result is None else result.message)
                      or 'Configure returned no configuration ID')
            return
        self.configuration_id = result.configuration_id
        self.deadline = time.monotonic() + 5.
        self.transition('CONFIRMING', 'Confirming fresh READY feedback for the selected configuration')
