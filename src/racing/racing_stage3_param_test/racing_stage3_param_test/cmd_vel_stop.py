import signal
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions


def zero_twist():
    return Twist()


def publish_stop(publisher, repeat=25, interval_sec=0.02):
    msg = zero_twist()
    for _ in range(repeat):
        try:
            publisher.publish(msg)
        except Exception:
            break
        time.sleep(interval_sec)


def emergency_cli_stop_async(topics=None):
    """后台发零速，避免在信号处理里阻塞导致进程无法退出。"""
    if topics is None:
        topics = ['/cmd_vel', '/stage2_cmd_vel']

    payload = (
        '{linear: {x: 0.0, y: 0.0, z: 0.0}, '
        'angular: {x: 0.0, y: 0.0, z: 0.0}}'
    )

    def _run():
        for _ in range(5):
            for topic in topics:
                try:
                    subprocess.Popen(
                        [
                            'ros2', 'topic', 'pub', '--once', topic,
                            'geometry_msgs/msg/Twist', payload,
                        ],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                except (OSError, FileNotFoundError):
                    pass
            time.sleep(0.04)

    threading.Thread(target=_run, daemon=True).start()


def init_without_ros_signal_handler(args=None):
    import rclpy

    return rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )


def spin_until_stop(node, stop_event, timeout_sec=0.05):
    import rclpy

    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=timeout_sec)


def install_stop_event(stop_event, stop_callback, cli_topics=None):
    def _request_stop():
        if stop_event.is_set():
            return
        stop_event.set()
        try:
            stop_callback()
        except Exception:
            pass
        emergency_cli_stop_async(cli_topics)

    def _handler(signum, frame):
        del signum, frame
        _request_stop()

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    return _request_stop
