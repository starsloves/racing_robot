"""Production lifecycle supervisor for the three racing stages.

The base stack is owned by the top-level launch.  This node owns only the
stage launch processes and the bounded, continuous handoff protocol.
"""

import json
import ctypes
import os
import signal
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from racing_common.racing_logger import terminal_write
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu, LaserScan
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


_LIBC = ctypes.CDLL(None)


class CompetitionSupervisor(Node):
    COMMON_UNIQUE_NODES = (
        'origincar_base', 'imu_filter_madgwick', 'ekf_filter_node',
        'lslidar_driver_node', 'aurora930_node', 'map_server',
        'lifecycle_manager_map_overlay', 'start_corner_pose_localizer',
        'telemetry_web_monitor', 'competition_supervisor', 'base_to_link',
        'base_to_gyro', 'link_to_laser', 'robot_state_publisher',
        'joint_state_publisher', 'voice_broadcast_node',
    )
    STAGE_NODES = {
        'stage1': ('competition_controller', 'qr_scanner'),
        'stage2': ('stage2_inertial_navigator',),
        'stage3': ('stage3_return_navigator',),
    }

    def __init__(self):
        super().__init__('competition_supervisor')
        self.declare_parameter('base_ready_stable_sec', 1.5)
        self.declare_parameter('base_message_max_age_sec', 2.0)
        self.declare_parameter('base_localizer_topic', 'start_corner_pose_diagnostic')
        self.declare_parameter('base_loss_grace_sec', 8.0)
        self.declare_parameter('base_health_probe_period_sec', 1.0)
        self.declare_parameter('monitor_period_sec', 0.10)
        self.declare_parameter('stage_start_timeout_sec', 20.0)
        self.declare_parameter('handoff_timeout_sec', 12.0)
        self.declare_parameter('shutdown_grace_sec', 3.0)
        self.declare_parameter('restart_limit', 2)
        self.declare_parameter('enable_stage2_vision_ai', True)
        self.declare_parameter('base_tf_timeout_sec', 0.10)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(String, 'competition_supervisor_state', latched)
        self._phase_pub = self.create_publisher(Int32, 'competition_phase', latched)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, 'competition_qr_task', self._qr_cb, latched)
        self.create_subscription(String, 'stage1_state', self._stage1_cb, latched)
        self.create_subscription(String, 'stage2_state', self._stage2_cb, latched)
        self.create_subscription(String, 'stage3_state', self._stage3_cb, latched)
        self.create_subscription(String, 'stage3_prewarm', self._stage3_prewarm_cb, latched)
        # Sensor callbacks must not wait behind process discovery, TF checks,
        # or a child launch.  The old single-threaded executor made a short
        # supervisor callback gap look like a dead IMU and killed startup.
        self._sensor_group = ReentrantCallbackGroup()
        self._base_localizer_topic = str(self.get_parameter('base_localizer_topic').value)
        self.create_subscription(
            Imu, '/imu/data', lambda _: self._mark('imu'), qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            Odometry, '/odom_combined', lambda _: self._mark('odom'), qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            LaserScan, '/scan', lambda _: self._mark('scan'), qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            CameraInfo, '/aurora/rgb/camera_info', lambda _: self._mark('camera'),
            qos_profile_sensor_data, callback_group=self._sensor_group,
        )
        self.create_subscription(
            OccupancyGrid, '/map', lambda _: self._mark('map'), latched,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            String, self._base_localizer_topic, self._localizer_cb, latched,
            callback_group=self._sensor_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.session_id = (
            os.environ.get('COMPETITION_SESSION_ID', '').strip()
            or datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        )
        self.base_state = 'starting'
        self.active_stage = ''
        self.prewarming_stage = ''
        self.lifecycle_state = 'base_starting'
        self.reason = 'waiting for common base layer'
        self._seen = {name: None for name in ('imu', 'odom', 'scan', 'camera', 'map')}
        self._localizer_valid = False
        self._localizer_seen = None
        self._map_ready_reported = False
        self._base_ready_since = None
        self._processes = {'stage1': None, 'stage2': None, 'stage3': None, 'vision_ai': None}
        self._stage_states = {'stage1': '', 'stage2': '', 'stage3': ''}
        self._launch_times = {}
        self._restart_count = {'stage1': 0, 'stage2': 0, 'stage3': 0}
        self._release_requested = set()
        self._activation_requested = set()
        # Keep rclpy.node.Node._clients intact; Node.create_client() appends
        # every created client to that internal list.
        self._service_clients = {}
        self._handoff_deadline = None
        self._finalizing = False
        self._child_cleanup_done = False
        self._child_cleanup_lock = threading.Lock()
        self._async_cleanup_threads = set()
        self._last_monitor_finished = time.monotonic()
        self._last_wait_log = 0.0
        self._last_tf_failure_log = {}
        self._health_probe_at = 0.0
        self._health_probe_cache = {
            'cmd_subscribers': 0,
            'odom_publishers': [],
            'transforms': {
                'map_to_odom_combined': False,
                'odom_combined_to_base_footprint': False,
                'map_to_base_footprint': False,
            },
        }
        self._base_loss_since = None
        self._competition_phase = None
        # Top-level launch can shut down the ROS context before executor spin
        # unwinds.  Register cleanup at the context boundary so independently
        # spawned stage process groups cannot survive a Ctrl+C/restart.
        rclpy.get_default_context().on_shutdown(self._cleanup_child_processes)
        self._set_competition_phase(0)
        self._publish_state()
        # Handoff activation is event-driven; this timer is only the service
        # discovery/retry safety net.  Sensor freshness is updated directly
        # by the reentrant subscriptions, so the monitor need not run at a
        # high rate.
        self._timer = self.create_timer(
            max(0.05, float(self.get_parameter('monitor_period_sec').value)),
            self._monitor,
        )
        self._heartbeat = self.create_timer(1.0, self._publish_state)

    def _mark(self, name):
        self._seen[name] = time.monotonic()
        if name == 'map' and not self._map_ready_reported:
            self._map_ready_reported = True
            terminal_write('[STARTUP] 地图 lifecycle active; /map received')
            self.get_logger().info('map lifecycle active; /map received')

    def _localizer_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if str(payload.get('state', '')).strip().lower() != 'valid':
            return
        self._localizer_valid = True
        self._localizer_seen = time.monotonic()

    def _publish_state(self):
        payload = {
            'session_id': self.session_id,
            'base_state': self.base_state,
            'active_stage': self.active_stage,
            'prewarming_stage': self.prewarming_stage,
            'lifecycle_state': self.lifecycle_state,
            'reason': self.reason,
        }
        self._state_pub.publish(String(data=json.dumps(payload, ensure_ascii=True, sort_keys=True)))

    def _set_competition_phase(self, phase):
        phase = int(phase)
        if phase == self._competition_phase:
            return
        self._competition_phase = phase
        self._phase_pub.publish(Int32(data=phase))
        self.get_logger().info(f'[LIFECYCLE] publish competition_phase={phase}')

    def _task(self, message):
        terminal_write(f'[TASK] {message}')
        self.get_logger().info(message)

    def _error(self, message):
        terminal_write(f'[ERROR] {message}')
        self.get_logger().error(message)

    def _base_health(self):
        now = time.monotonic()
        max_age = float(self.get_parameter('base_message_max_age_sec').value)
        stale_messages = [
            name for name, stamp in self._seen.items()
            # /map is a reliable, transient-local static map.  A healthy
            # map server normally sends it once to this subscriber, rather
            # than periodically like the live sensor streams.
            if stamp is None or (name != 'map' and now - stamp > max_age)
        ]
        # DDS graph queries and TF waits are comparatively expensive.  Running
        # them on every 100 ms monitor tick can starve this process's sensor
        # callbacks during Nav2 startup, making healthy streams look stale.
        # Freshness stays live; only the slow structural probes are cached.
        probe_period = max(
            0.1, float(self.get_parameter('base_health_probe_period_sec').value))
        if now - self._health_probe_at >= probe_period:
            self._health_probe_at = now
            cmd_subscribers = self.count_subscribers('/cmd_vel')
            try:
                odom_publishers = [
                    {
                        'node_name': str(info.node_name),
                        'node_namespace': str(info.node_namespace),
                        'gid': str(info.endpoint_gid),
                    }
                    for info in self.get_publishers_info_by_topic(
                        '/odom_combined', no_mangle=False)
                ]
            except Exception as exc:
                odom_publishers = [{'error': f'{type(exc).__name__}: {exc}'}]
            transforms = {}
            tf_timeout = max(0.0, float(self.get_parameter('base_tf_timeout_sec').value))
            for label, parent, child in (
                ('map_to_odom_combined', 'map', 'odom_combined'),
                ('odom_combined_to_base_footprint', 'odom_combined', 'base_footprint'),
                ('map_to_base_footprint', 'map', 'base_footprint'),
            ):
                try:
                    # Only the composed edge may need to wait for the two dynamic
                    # samples to share a timestamp.  Keep the diagnostic edge
                    # checks non-blocking so the supervisor never starves sensor
                    # callbacks while reporting the missing link.
                    timeout = tf_timeout if label == 'map_to_base_footprint' else 0.0
                    self._tf_buffer.lookup_transform(
                        parent, child, rclpy.time.Time(),
                        timeout=Duration(seconds=timeout),
                    )
                    transforms[label] = True
                except TransformException as exc:
                    transforms[label] = False
                    last_log = self._last_tf_failure_log.get(label, 0.0)
                    if now - last_log >= 5.0:
                        self._last_tf_failure_log[label] = now
                        self.get_logger().warning(
                            f'TF check failed {label} ({parent}->{child}): {exc}')
            self._health_probe_cache = {
                'cmd_subscribers': cmd_subscribers,
                'odom_publishers': odom_publishers,
                'transforms': transforms,
            }
        else:
            cmd_subscribers = self._health_probe_cache['cmd_subscribers']
            odom_publishers = self._health_probe_cache['odom_publishers']
            transforms = self._health_probe_cache['transforms']
        return {
            'stale_messages': stale_messages,
            'localizer_valid': self._localizer_valid,
            'cmd_subscribers': cmd_subscribers,
            'odom_publishers': odom_publishers,
            'transforms': transforms,
        }

    def _base_ready(self):
        now = time.monotonic()
        health = self._base_health()
        ready = (
            not health['stale_messages']
            and health['localizer_valid']
            and health['cmd_subscribers'] > 0
            and len(health['odom_publishers']) == 1
            and 'error' not in health['odom_publishers'][0]
            and health['transforms']['map_to_base_footprint']
        )
        if not ready:
            self._base_ready_since = None
            return False
        if self._base_ready_since is None:
            self._base_ready_since = now
        return now - self._base_ready_since >= float(self.get_parameter('base_ready_stable_sec').value)

    def _base_wait_reason(self):
        health = self._base_health()
        missing = list(health['stale_messages'])
        if health['cmd_subscribers'] == 0:
            missing.append('cmd_vel subscriber')
        if len(health['odom_publishers']) != 1:
            missing.append(
                'odom_combined publishers=' + str(health['odom_publishers']))
        if not health['localizer_valid']:
            missing.append('START_CORNER_LOCALIZER valid')
        for label, available in health['transforms'].items():
            if not available:
                missing.append(f'TF {label}')
        return ', '.join(missing) if missing else 'stable window'

    def _base_loss_reason(self):
        """Describe the exact failed base prerequisite for a safe shutdown."""
        health = self._base_health()
        now = time.monotonic()
        details = []
        if health['stale_messages']:
            ages = ','.join(
                f'{name}={now - self._seen[name]:.2f}s'
                for name in health['stale_messages']
                if self._seen[name] is not None
            )
            details.append('stale=' + ','.join(health['stale_messages']))
            if ages:
                details.append('ages=' + ages)
        if health['cmd_subscribers'] == 0:
            details.append('cmd_vel_subscribers=0')
        if len(health['odom_publishers']) != 1:
            details.append(
                'odom_combined_publishers=' + str(health['odom_publishers']))
        if not health['localizer_valid']:
            details.append('start_corner_localizer_invalid')
        missing_tfs = [
            label for label, available in health['transforms'].items()
            if not available
        ]
        if missing_tfs:
            details.append('missing_tf=' + ','.join(missing_tfs))
        return '; '.join(details) if details else 'base readiness stability window reset'

    def _residual_stage_nodes(self):
        names = {name for name, _namespace in self.get_node_names_and_namespaces()}
        return sorted(name for nodes in self.STAGE_NODES.values() for name in nodes if name in names)

    def _duplicate_common_nodes(self):
        counts = {}
        for name, _namespace in self.get_node_names_and_namespaces():
            counts[name] = counts.get(name, 0) + 1
        return sorted(
            name for name in self.COMMON_UNIQUE_NODES if counts.get(name, 0) > 1
        )

    @staticmethod
    def _command(package, launch_file, arguments):
        return ['ros2', 'launch', package, launch_file] + [f'{key}:={value}' for key, value in arguments.items()]

    @staticmethod
    def _set_child_parent_death_signal(parent_pid):
        """Make a detached child process terminate when Supervisor dies.

        Stage launchers intentionally use their own process groups so they can
        be stopped independently.  ``PR_SET_PDEATHSIG`` preserves that
        isolation while preventing an unexpected Supervisor exit from leaving
        a stage node behind in the next competition session.
        """
        try:
            # Linux PR_SET_PDEATHSIG = 1.
            if _LIBC.prctl(1, signal.SIGTERM) != 0:
                return
            # Cover the small fork/pre-exec race where the parent died before
            # prctl() was installed.
            if os.getppid() != parent_pid:
                os.kill(os.getpid(), signal.SIGTERM)
        except (AttributeError, OSError):
            # The explicit process-group cleanup remains the fallback on
            # platforms without prctl.
            return

    @staticmethod
    def _process_start_ticks(pid):
        """Read the Linux start tick field used to guard against PID reuse."""
        try:
            with open(f'/proc/{pid}/stat', 'r', encoding='utf-8') as stream:
                fields = stream.read().rsplit(')', 1)[1].split()
            return fields[19]
        except (IndexError, OSError):
            return ''

    def _child_environment(self):
        env = os.environ.copy()
        env['COMPETITION_SESSION_ID'] = self.session_id
        env['COMPETITION_SUPERVISOR_PID'] = str(os.getpid())
        env['COMPETITION_SUPERVISOR_START_TICKS'] = self._process_start_ticks(os.getpid())
        env['RMW_FASTRTPS_USE_SHM'] = '0'
        env['RMW_FASTRTPS_TRANSPORT'] = 'UDPv4'
        return env

    def _spawn(self, key, package, launch_file, arguments):
        env = self._child_environment()
        parent_pid = os.getpid()
        process = subprocess.Popen(
            self._command(package, launch_file, arguments),
            start_new_session=True,
            env=env,
            preexec_fn=lambda: self._set_child_parent_death_signal(parent_pid),
        )
        self._processes[key] = process
        self._launch_times[key] = time.monotonic()
        self._task(f'{key.upper()} process started pid={process.pid} (standby)')
        return process

    def _launch_stage1(self):
        if self._alive('stage1'):
            return
        self._spawn('stage1', 'racing_stage1', 'competition_stage1.launch.py', {
            'include_bringup': 'false', 'include_lidar': 'false',
            'include_camera': 'false', 'include_depth': 'false',
            'standalone_map_overlay': 'false',
            'standby': 'true',
        })

    def _launch_stage2(self):
        if self._alive('stage2'):
            return
        self._spawn('stage2', 'racing_stage2', 'competition_stage2.launch.py', {
            'include_bringup': 'false', 'include_lidar': 'false',
            'include_camera': 'false', 'include_depth': 'false',
            'enable_test_publisher': 'false',
            'include_obstacle_markers': 'false', 'cmd_topic': '/cmd_vel',
            'standby': 'true',
        })

    def _launch_stage3(self):
        if self._alive('stage3'):
            return
        self._spawn('stage3', 'racing_stage3', 'competition_stage3.launch.py', {
            'include_bringup': 'false', 'include_lidar': 'false',
            'include_camera': 'false', 'include_depth': 'false',
            'enable_test_publisher': 'false', 'cmd_topic': '/cmd_vel',
            'standby': 'true',
        })

    def _launch_vision_ai(self):
        if not bool(self.get_parameter('enable_stage2_vision_ai').value) or self._alive('vision_ai'):
            return
        config = os.path.join(get_package_share_directory('racing_vision_ai'), 'config', 'vision_ai_config.yaml')
        env = self._child_environment()
        parent_pid = os.getpid()
        self._processes['vision_ai'] = subprocess.Popen([
            'ros2', 'run', 'racing_vision_ai', 'vision_ai_node', '--ros-args',
            '-r', '__node:=stage2_vision_ai', '-p', f'config_path:={config}',
            '-p', 'trigger_topic:=stage2_ai_capture', '-p', 'image_topic:=/aurora/rgb/image_raw',
            '-p', 'result_topic:=ai_description', '-p', 'status_topic:=stage2_ai_status',
            '-p', 'mission_state_topic:=stage3_state',
        ], start_new_session=True, env=env,
            preexec_fn=lambda: self._set_child_parent_death_signal(parent_pid))
        self._task(f'S2 vision AI prewarming pid={self._processes["vision_ai"].pid}')

    def _alive(self, key):
        process = self._processes.get(key)
        return process is not None and process.poll() is None

    def _call_stage(self, stage, action):
        try:
            marker = (stage, action)
            if marker in self._activation_requested or marker in self._release_requested:
                return
            client = self._service_clients.get(marker)
            if client is None:
                client = self.create_client(Trigger, f'/competition/{stage}/{action}')
                self._service_clients[marker] = client
            if not client.wait_for_service(timeout_sec=0.0):
                return
            future = client.call_async(Trigger.Request())
            (self._activation_requested if action == 'activate' else self._release_requested).add(marker)
        except Exception as exc:
            self._fail(f'{stage} {action} service setup failed: {type(exc).__name__}: {exc}')
            return

        def done(result):
            try:
                response = result.result()
            except Exception as exc:
                self._fail(f'{stage} {action} service failed: {exc}')
                return
            if not response.success:
                self._fail(f'{stage} {action} rejected: {response.message}')
                return
            self._task(f'{stage.upper()} {action}: {response.message}')
        future.add_done_callback(done)

    def _activate(self, stage):
        self._call_stage(stage, 'activate')

    def _release(self, stage):
        self._call_stage(stage, 'release')
        if stage == 'stage1':
            self._call_stage_qr_release()
        elif stage == 'stage2':
            # The S2-only AI is a Supervisor-owned companion process. It no
            # longer has a valid mission after S3 accepts the handoff, so
            # release it with S2 without publishing any motion command.
            # Do not wait for a potentially busy model request from inside a
            # ROS subscription callback.  The Supervisor shares a single
            # executor with the common-base freshness subscriptions; a
            # synchronous wait here can make all live sensors appear stale.
            process = self._processes.get('vision_ai')
            if process is None or process.poll() is not None:
                self._processes['vision_ai'] = None
            else:
                self._terminate_process_async(process, 'S2 vision AI')

    def _terminate_process_async(self, process, label):
        if process is None or process.poll() is not None:
            return

        def worker():
            started = time.monotonic()
            try:
                self._terminate_process(process)
            finally:
                elapsed = time.monotonic() - started
                self._task(f'{label} cleanup finished in {elapsed:.2f}s')
                if self._processes.get('vision_ai') is process:
                    self._processes['vision_ai'] = None
                self._async_cleanup_threads.discard(threading.current_thread())

        thread = threading.Thread(
            target=worker,
            name='competition-child-cleanup',
            daemon=True,
        )
        self._async_cleanup_threads.add(thread)
        thread.start()
        self._task(f'{label} cleanup scheduled asynchronously')

    def _call_stage_qr_release(self):
        marker = ('stage1_qr', 'release')
        if marker in self._release_requested:
            return
        client = self._service_clients.get(marker)
        if client is None:
            client = self.create_client(Trigger, '/competition/stage1/qr_release')
            self._service_clients[marker] = client
        if not client.wait_for_service(timeout_sec=0.0):
            return
        future = client.call_async(Trigger.Request())
        self._release_requested.add(marker)

        def done(result):
            try:
                response = result.result()
            except Exception as exc:
                self._fail(f'S1 QR release service failed: {exc}')
                return
            if not response.success:
                self._fail(f'S1 QR release rejected: {response.message}')
                return
            self._task(f'S1 QR release: {response.message}')
        future.add_done_callback(done)

    def _call_stage_qr_activate(self):
        """Retry QR activation until its service has entered the ROS graph."""
        marker = ('stage1_qr', 'activate')
        if marker in self._activation_requested:
            return
        client = self._service_clients.get(marker)
        if client is None:
            client = self.create_client(Trigger, '/competition/stage1/qr_activate')
            self._service_clients[marker] = client
        if not client.wait_for_service(timeout_sec=0.0):
            return
        future = client.call_async(Trigger.Request())
        self._activation_requested.add(marker)

        def done(result):
            try:
                response = result.result()
            except Exception as exc:
                self._activation_requested.discard(marker)
                self._task(f'S1 QR activate retry after service error: {exc}')
                return
            if not response.success:
                self._activation_requested.discard(marker)
                self._task(f'S1 QR activate retry: {response.message}')
                return
            self._task(f'S1 QR activate: {response.message}')
        future.add_done_callback(done)

    def _qr_cb(self, msg):
        if self.active_stage != 'stage1' or not msg.data.strip():
            return
        self._launch_stage2()
        self._launch_vision_ai()
        self.prewarming_stage = 'stage2'
        self.lifecycle_state = 'stage1_running'
        self.reason = f'QR received ({msg.data.strip()}); S2 prewarming while S1 continues'
        self._task('S2 prewarming after QR; S1 keeps motion ownership')
        self._publish_state()

    def _stage1_cb(self, msg):
        state = msg.data.strip()
        previous = self._stage_states['stage1']
        self._stage_states['stage1'] = state
        if state != previous:
            self._task(f'S1 state={state}')
        if state == 'running' and self.active_stage == 'stage1':
            self._set_competition_phase(1)
            self.lifecycle_state = 'stage1_running'
            self.reason = 'S1 activated; motion ownership granted'
            self._publish_state()
        elif state == 'handoff_ready' and self.active_stage == 'stage1':
            self._launch_stage2()
            self.prewarming_stage = 'stage2'
            self.lifecycle_state = 'stage1_handoff_wait'
            self.reason = 'S1 entry pose published; waiting for S2 continuous command'
            self._handoff_deadline = time.monotonic() + float(self.get_parameter('handoff_timeout_sec').value)
            if self._stage_states['stage2'] == 'ready':
                self._task('S2 already ready; scheduling continuous handoff activation')
                self._activate('stage2')
            else:
                self._task('S1 handoff ready; waiting for prewarmed S2 ready state')
            self._publish_state()

    def _stage2_cb(self, msg):
        state = msg.data.strip()
        self.get_logger().info(
            f'[LIFECYCLE] receive stage2_state={state} '
            f't={self.get_clock().now().nanoseconds / 1e9:.3f}'
        )
        self._stage_states['stage2'] = state
        if state == 'ready' and self.active_stage == 'stage1' and self.lifecycle_state == 'stage1_handoff_wait':
            self._task('S2 ready; scheduling continuous handoff activation')
            self._activate('stage2')
        elif state == 'handoff_command_ready' and self.active_stage == 'stage1':
            self._set_competition_phase(2)
            self.active_stage = 'stage2'
            self.prewarming_stage = ''
            self.lifecycle_state = 'stage2_running'
            self.reason = 'S2 emitted first continuous handoff command'
            # S3 imports its navigation stack in a child launch process. Start
            # that cold path as soon as S2 owns motion, rather than waiting
            # until the final handoff line where only a few seconds remain.
            self._launch_stage3()
            self.prewarming_stage = 'stage3'
            self._task('S3 prewarming after S2 handoff; S2 keeps motion ownership')
            # The S1->S2 ownership transfer is complete as soon as S2 has
            # published its first command. Do not keep the old handoff
            # deadline alive while S1 finishes its release shutdown.
            self._handoff_deadline = None
            self._release('stage1')
            self._task('S2 command ready; S1 released itself')
            self._publish_state()
        elif state == 'complete' and self.active_stage == 'stage2':
            self._launch_stage3()
            self.prewarming_stage = 'stage3'
            self.lifecycle_state = 'stage2_handoff_wait'
            self.reason = 'S2 complete anchor received; waiting for S3 command'
            self._handoff_deadline = time.monotonic() + float(self.get_parameter('handoff_timeout_sec').value)
            self._task('S2 complete; waiting for prewarmed S3 ready state')
            self._publish_state()

    def _stage3_prewarm_cb(self, msg):
        if self.active_stage == 'stage2' and msg.data.strip():
            self._launch_stage3()
            self.prewarming_stage = 'stage3'
            self.reason = 'S3 prewarm event received while S2 remains active'
            self._task('S3 prewarming while S2 keeps motion ownership')
            self._publish_state()

    def _stage3_cb(self, msg):
        state = msg.data.strip()
        self._stage_states['stage3'] = state
        if state == 'ready' and self.active_stage == 'stage2' and self.lifecycle_state == 'stage2_handoff_wait':
            self._task('S3 ready; scheduling continuous handoff activation')
            self._activate('stage3')
        elif state == 'handoff_command_ready' and self.active_stage == 'stage2':
            self._set_competition_phase(3)
            self.active_stage = 'stage3'
            self.prewarming_stage = ''
            self.lifecycle_state = 'stage3_running'
            self.reason = 'S3 emitted first continuous handoff command'
            # S3 now owns motion; S2 release latency is not a failed
            # handoff and must not consume the preceding deadline.
            self._handoff_deadline = None
            self._release('stage2')
            self._task('S3 command ready; S2 released itself')
            self._publish_state()
        elif state == 'complete' and self.active_stage == 'stage3':
            self._set_competition_phase(0)
            # ``complete`` is the terminal handoff event: Stage3 has already
            # issued zero speed, released its visual resources, and started
            # its own shutdown.  Do not hold the competition open for a
            # fixed grace interval or for the child launch wrapper to vanish.
            # The top-level launch owns the remaining base-layer teardown.
            self._finish()
            return

    def _monitor(self):
        if self._finalizing:
            return
        monitor_started = time.monotonic()
        callback_gap = monitor_started - self._last_monitor_finished
        self._last_monitor_finished = monitor_started
        if callback_gap > 1.0:
            self.get_logger().warning(
                f'Supervisor monitor callback gap={callback_gap:.2f}s; '
                'checking whether executor starvation affected freshness'
            )
        if ('stage1', 'release') in self._release_requested and ('stage1_qr', 'release') not in self._release_requested:
            self._call_stage_qr_release()
        if self.lifecycle_state == 'base_starting':
            duplicates = self._duplicate_common_nodes()
            if duplicates:
                self._fail('duplicate common nodes from previous launch: ' + ', '.join(duplicates))
                return
            if self._base_ready():
                residual = self._residual_stage_nodes()
                if residual:
                    self._fail(f'previous stage nodes still present: {", ".join(residual)}')
                    return
                self.base_state = 'ready'
                self.active_stage = 'stage1'
                self.lifecycle_state = 'stage1_starting'
                self.reason = 'common base layer stable; launching S1 and prewarming S2'
                self._task('基础节点 ready; S1 standby starting')
                self._launch_stage1()
                # S2 imports cv2/numpy/cv_bridge and its vision stack. Start
                # that cold path while S1 is driving so the handoff window
                # contains only task/anchor delivery and service activation.
                self._launch_stage2()
                self._launch_vision_ai()
                self.prewarming_stage = 'stage2'
                self._task('S2 standby prewarm started before QR; no motion authority')
                self._publish_state()
            elif time.monotonic() - self._last_wait_log > 5.0:
                self._task('waiting for base: ' + self._base_wait_reason())
                self._last_wait_log = time.monotonic()
            return

        # Service discovery can lag behind a latched ready state. Keep
        # retrying activation from the supervisor timer instead of relying on
        # the single callback edge that published ``ready``.
        if self.active_stage == 'stage1':
            if self._stage_states['stage1'] == 'ready':
                self._activate('stage1')
            elif self._stage_states['stage1'] in ('running', 'search_qr'):
                self._call_stage_qr_activate()
        if self.active_stage == 'stage2' and self.lifecycle_state == 'stage2_handoff_wait':
            if self._stage_states['stage3'] == 'ready':
                self._activate('stage3')
        if self.active_stage == 'stage1' and self.lifecycle_state == 'stage1_handoff_wait':
            if self._stage_states['stage2'] == 'ready':
                self._activate('stage2')

        if self.lifecycle_state not in ('finished', 'failed'):
            if self._base_ready():
                self._base_loss_since = None
            else:
                # Before S1 owns motion, tolerate one short sensor/TF gap.
                # A transient executor or serial hiccup must not tear down a
                # fresh launch; once motion is active, fail closed immediately.
                if self.lifecycle_state == 'stage1_starting':
                    if self._base_loss_since is None:
                        self._base_loss_since = time.monotonic()
                    grace = float(self.get_parameter('base_loss_grace_sec').value)
                    if time.monotonic() - self._base_loss_since < grace:
                        return
                self._fail('common base prerequisite lost: ' + self._base_loss_reason())
                return

        if self._handoff_deadline is not None and time.monotonic() > self._handoff_deadline:
            self._fail(f'{self.lifecycle_state} timed out before new stage command')
            return

        for stage in ('stage1', 'stage2', 'stage3'):
            process = self._processes[stage]
            if process is None or process.poll() is None:
                continue
            self._processes[stage] = None
            if stage in ('stage1', 'stage2') and (stage, 'release') in self._release_requested:
                self._task(f'{stage.upper()} self-exit confirmed')
                # Clear only the deadline that belonged to this stage's own
                # handoff. A late S1/S2 exit must not disable a newer S2/S3
                # handoff or the final S3 self-exit timeout.
                if (
                    (stage == 'stage1' and self.lifecycle_state == 'stage2_running')
                    or (stage == 'stage2' and self.lifecycle_state == 'stage3_running')
                ):
                    self._handoff_deadline = None
                continue
            pre_motion = self.active_stage != stage
            if pre_motion and self._restart_count[stage] < int(self.get_parameter('restart_limit').value):
                self._restart_count[stage] += 1
                self._task(f'{stage.upper()} exited before motion; retry {self._restart_count[stage]}')
                getattr(self, f'_launch_{stage}')()
                continue
            self._fail(f'{stage} process exited unexpectedly (code={process.returncode})')
            return


    def _publish_zero(self):
        for _ in range(10):
            self._cmd_pub.publish(Twist())

    def _terminate_process(self, process, shutdown_grace_sec=None):
        if process is None or process.poll() is not None:
            return
        if shutdown_grace_sec is None:
            try:
                shutdown_grace_sec = float(self.get_parameter('shutdown_grace_sec').value)
            except Exception:
                shutdown_grace_sec = 3.0
        for sig, timeout in ((signal.SIGINT, shutdown_grace_sec), (signal.SIGTERM, 2.0), (signal.SIGKILL, 0.5)):
            try:
                os.killpg(process.pid, sig)
                process.wait(timeout=timeout)
                return
            except (subprocess.TimeoutExpired, ProcessLookupError):
                continue

    def _cleanup_child_processes(self):
        with self._child_cleanup_lock:
            if self._child_cleanup_done:
                return
            self._child_cleanup_done = True
            processes = tuple(self._processes.values())
            try:
                shutdown_grace_sec = float(self.get_parameter('shutdown_grace_sec').value)
            except Exception:
                shutdown_grace_sec = 3.0

        # Context shutdown callbacks run synchronously.  Waiting here for a
        # child launch wrapper would keep rclpy.spin() alive and prevent the
        # top-level launch from seeing Supervisor exit.  The worker sends the
        # normal signal sequence independently; the parent-death signal on
        # each wrapper remains the final fallback when this process exits.
        def worker():
            for process in processes:
                self._terminate_process(process, shutdown_grace_sec)

        thread = threading.Thread(
            target=worker,
            name='competition-child-group-cleanup',
            daemon=True,
        )
        thread.start()

    def _fail(self, reason):
        if self._finalizing:
            return
        self._finalizing = True
        self.base_state = 'failed'
        self._set_competition_phase(0)
        self.lifecycle_state = 'failed'
        self.reason = reason
        self._publish_zero()
        self._publish_state()
        self._error(reason)
        self._cleanup_child_processes()
        self._timer.cancel()
        # Do not call rclpy.shutdown() here.  On Humble it can block inside a
        # callback and prevent this process exit from reaching launch.
        os._exit(1)

    def _finish(self):
        self._finalizing = True
        self._set_competition_phase(0)
        self.active_stage = ''
        self.prewarming_stage = ''
        self.lifecycle_state = 'finished'
        self.reason = 'S3 reported complete; closing common base layer'
        self._publish_zero()
        self._publish_state()
        self._task('competition complete; Supervisor exits and top-level launch closes base layer')
        self._timer.cancel()
        # Do not call rclpy.shutdown() from this callback.  It can block
        # before os._exit() runs, which leaves the top-level launch alive.
        # The Supervisor Node's on_exit handler is the single owner of the
        # resulting top-level Shutdown event.
        os._exit(0)

    def destroy_node(self):
        # Ctrl+C and failure use the independent zero-speed publisher before
        # the launch system tears down base hardware.
        if rclpy.ok() and self.lifecycle_state != 'finished':
            self._publish_zero()
        self._cleanup_child_processes()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
