"""Explicit remote settings editor; Qt owns only its read-only test child."""
import json
from PyQt5 import QtCore, QtWidgets

from . import config, events, runner

LABELS = (
    ('SERVER_HOST', 'RabbitMQ server IP / hostname'), ('AMQP_PORT', 'AMQP port'),
    ('AMQP_TLS', 'AMQP TLS (true / false)'), ('AMQP_VHOST', 'Virtual host'),
    ('AMQP_USERNAME', 'Username'), ('AMQP_PASSWORD', 'Password'),
    ('CONVEYOR_API_URL', 'Conveyor API base URL'), ('BELT_ID', 'Belt ID'),
    ('ARM_NUMBER', 'Arm number (1–8)'), ('CONNECT_TIMEOUT_SEC', 'Timeout per check (seconds)'),
    ('INTERNAL_API_KEY', 'Internal API key (optional; not tested)'),
    ('MESSAGE_SIGNING_KEY', 'Message signing key (optional; not tested)'),
)


class Window(QtWidgets.QWidget):
    def __init__(self, root=None):
        super().__init__()
        self.root = config.project_root() if root is None else root
        self.setWindowTitle('Remote Setup — settings and connection test')
        self.resize(740, 720)
        self.process = None
        layout = QtWidgets.QVBoxLayout(self)
        intro = QtWidgets.QLabel(
            'Configure this station for the CATARM server. Save Settings, then Test Saved Settings.\n'
            'Opening this window does not connect. Robot commands are disabled.')
        intro.setWordWrap(True)
        layout.addWidget(intro)
        form = QtWidgets.QFormLayout()
        self.fields = {}
        for suffix, label in LABELS:
            key = 'REMOTE_' + suffix
            edit = QtWidgets.QLineEdit()
            edit.setMaxLength(4096)
            if key in config.SECRETS:
                edit.setEchoMode(QtWidgets.QLineEdit.Password)
            self.fields[key] = edit
            form.addRow(label, edit)
        self.fields['REMOTE_AMQP_PORT'].setPlaceholderText('5672 (plain) or 5671 (TLS)')
        self.fields['REMOTE_AMQP_VHOST'].setPlaceholderText('/')
        self.fields['REMOTE_AMQP_TLS'].setPlaceholderText('true or false')
        self.fields['REMOTE_CONVEYOR_API_URL'].setPlaceholderText('http://server:port')
        self.fields['REMOTE_CONNECT_TIMEOUT_SEC'].setPlaceholderText('5')
        layout.addLayout(form)
        self.routing = QtWidgets.QLabel('Exchange: catarm.events • no queues are created or consumed')
        self.routing.setWordWrap(True)
        layout.addWidget(self.routing)
        buttons = QtWidgets.QHBoxLayout()
        self.reload_button = QtWidgets.QPushButton('Reload Saved')
        self.save_button = QtWidgets.QPushButton('Save Settings')
        self.test_button = QtWidgets.QPushButton('Test Saved Settings')
        self.cancel_button = QtWidgets.QPushButton('Cancel Test')
        self.cancel_button.setEnabled(False)
        for button in (self.reload_button, self.save_button, self.test_button, self.cancel_button):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.output = QtWidgets.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(60)
        layout.addWidget(self.output)
        self.timer = QtCore.QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(lambda: self.cancel('Connection test timed out and was stopped.'))
        self.reload_button.clicked.connect(self.reload)
        self.save_button.clicked.connect(self.save)
        self.test_button.clicked.connect(self.test)
        self.cancel_button.clicked.connect(lambda: self.cancel('Connection test canceled.'))
        self.reload()

    def reload(self):
        try:
            values = config.read(self.root)
            for key, edit in self.fields.items():
                edit.setText(values[key])
            self.output.setPlainText('Saved settings loaded. Enter all required fields before testing.')
        except (OSError, RuntimeError, ValueError):
            self.output.setPlainText('Cannot read project .env; check its syntax and required station keys.')

    def save(self):
        try:
            settings = config.save({key: edit.text() for key, edit in self.fields.items()}, self.root)
            self.routing.setText('Exchange: catarm.events • routing prefix: ' + settings.route
                                 + '\nNo queues are created or consumed.')
            self.output.setPlainText('Settings saved. Connection test has not run.')
            events.record(self.root, 'settings_saved', 'Remote settings saved explicitly.')
        except ValueError as exc:
            self.output.setPlainText(str(exc))  # Validation messages contain key names only.
        except (OSError, RuntimeError):
            self.output.setPlainText('Could not save settings; check .env syntax and file permissions.')

    def test(self):
        if self.process is not None:
            return
        try:
            settings = config.load(self.root)
            if settings.values != {key: edit.text() for key, edit in self.fields.items()}:
                raise ValueError('Unsaved edits: Save Settings or Reload Saved before testing.')
        except ValueError as exc:
            self.output.setPlainText(str(exc))
            return
        except (OSError, RuntimeError):
            self.output.setPlainText('Cannot load saved remote settings.')
            return
        self.output.setPlainText('Testing saved broker login and conveyor API…')
        process = QtCore.QProcess(self)
        self.process = process
        process.setStandardErrorFile(QtCore.QProcess.nullDevice())
        process.started.connect(lambda: (process.write(json.dumps(settings.values).encode()),
                                         process.closeWriteChannel()))
        process.finished.connect(self.finished)
        process.errorOccurred.connect(self.failed)
        self.set_busy(True)
        self.timer.start(int(settings.deadline * 1000))
        process.start(runner.COMMAND[0], runner.COMMAND[1:])

    def set_busy(self, busy):
        for widget in (*self.fields.values(), self.save_button, self.reload_button, self.test_button):
            widget.setEnabled(not busy)
        self.cancel_button.setEnabled(busy)

    def finished(self, code, _status):
        if self.process is None:
            return
        self.timer.stop()
        process, self.process = self.process, None
        ok, message = runner.result_text(bytes(process.readAllStandardOutput()).decode('utf-8'), code)
        process.deleteLater()
        self.set_busy(False)
        self.output.setPlainText(message)
        events.record(self.root, 'test_passed' if ok else 'test_failed', message)

    def failed(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self.cancel('Could not start the connection test process.')

    def cancel(self, message):
        self.timer.stop()
        if self.process is not None:
            process, self.process = self.process, None
            process.kill()
            process.waitForFinished(1000)
            process.deleteLater()
        self.set_busy(False)
        self.output.setPlainText(message)

    def closeEvent(self, event):
        self.cancel('Connection test closed.')
        event.accept()
