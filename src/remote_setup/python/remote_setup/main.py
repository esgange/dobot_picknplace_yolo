"""UI or one-shot headless testing; no ROS robot clients, services or consumers."""
import argparse
import os
import signal
import sys

from . import config, events, runner


def main(args=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='Test saved settings once and exit')
    arguments = sys.argv if args is None else ['remote_setup', *args]
    options = parser.parse_args(remove_ros_args(arguments)[1:])
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        print('Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 is required.', file=sys.stderr)
        return 2
    rclpy.init(args=arguments[1:])
    node = rclpy.create_node('remote_setup', start_parameter_services=False)
    try:
        if options.headless:
            try:
                settings = config.load()
            except ValueError as exc:
                print(str(exc), file=sys.stderr)
                events.record(config.project_root(), 'settings_invalid', str(exc))
                return 2
            except (OSError, RuntimeError):
                print('Cannot read valid project .env settings.', file=sys.stderr)
                return 2
            ok, message = runner.run(settings)
            print(message)
            events.record(config.project_root(), 'test_passed' if ok else 'test_failed', message)
            return 0 if ok else 1
        from PyQt5 import QtCore, QtWidgets
        from .gui import Window
        app = QtWidgets.QApplication([arguments[0]])
        window = Window()
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: app.quit() if not rclpy.ok() else None)
        timer.start(100)
        signal.signal(signal.SIGINT, lambda *_: window.close())
        signal.signal(signal.SIGTERM, lambda *_: window.close())
        app.aboutToQuit.connect(lambda: window.cancel('Closing Remote Setup.'))
        window.show()
        return app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
