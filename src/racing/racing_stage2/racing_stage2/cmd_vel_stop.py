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
    """Publish one bounded CLI stop per topic as a last-resort fallback."""
    if topics is None:
        topics = ['/cmd_vel']

    payload = (
        '{linear: {x: 0.0, y: 0.0, z: 0.0}, '
        'angular: {x: 0.0, y: 0.0, z: 0.0}}'
    )

    def _run():
        for topic in dict.fromkeys(topics):
            process = None
            try:
                process = subprocess.Popen(
                    [
                        'ros2', 'topic', 'pub', '--once', topic,
                        'geometry_msgs/msg/Twist', payload,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                process.communicate(timeout=0.75)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.communicate(timeout=0.25)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
            except (OSError, FileNotFoundError):
                pass

    # The in-process publisher has already sent the stop command.  This CLI
    # fallback must not keep SIGINT shutdown waiting for ROS discovery.
    threading.Thread(target=_run, name='Stage2CliStop', daemon=True).start()


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
