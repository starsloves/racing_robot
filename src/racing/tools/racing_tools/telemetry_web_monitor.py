"""Browser telemetry monitor for the racing robot.

The monitor intentionally uses plain HTTP + Server-Sent Events instead of
requiring rosbridge or a JavaScript ROS client.  It is read-only: it never
publishes motion commands.
"""

import json
import errno
import math
import os
import queue
import re
import socket
import subprocess
import threading
import time
import traceback
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


class _MonitorState:
    def __init__(self):
        self.lock = threading.RLock()
        self.map_meta = None
        self.pose = None
        self.scan = []
        self.scan_msg = None
        self.cmd = {'linear_x': 0.0, 'angular_z': 0.0}
        self.stage_states = {}
        self.phase = 0
        self.qr_task = ''
        self.target = None
        self.route = []
        self.mission_route = []
        self.history = deque(maxlen=1600)
        self.events = deque(maxlen=40)
        self.last_update = 0.0
        self.clients = set()


class _RequestHandler(BaseHTTPRequestHandler):
    server_version = 'RacingTelemetry/1.0'
    protocol_version = 'HTTP/1.1'

    @property
    def monitor(self):
        return self.server.monitor

    def _headers(self, content_type, cache='no-store'):
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', cache)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Connection', 'keep-alive')

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            try:
                with open(self.monitor.html_path, 'rb') as stream:
                    body = stream.read()
            except OSError:
                self.send_error(404, 'monitor.html not installed')
                return
            self.send_response(200)
            self._headers('text/html; charset=utf-8', cache='no-cache')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == '/map.png':
            try:
                with open(self.monitor.map_image_path, 'rb') as stream:
                    body = stream.read()
            except OSError:
                self.send_error(404, 'map image unavailable')
                return
            self.send_response(200)
            self._headers('image/png', cache='public, max-age=60')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path in ('/snapshot.png', '/latest.png'):
            try:
                with open(self.monitor.snapshot_image_path, 'rb') as stream:
                    body = stream.read()
            except OSError:
                self.send_error(404, 'debug snapshot unavailable')
                return
            self.send_response(200)
            self._headers('image/png', cache='no-cache')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == '/health':
            body = json.dumps(self.monitor.health_snapshot(), ensure_ascii=True).encode('utf-8')
            self.send_response(200)
            self._headers('application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == '/state':
            body = json.dumps(self.monitor.snapshot(), ensure_ascii=True).encode('utf-8')
            self.send_response(200)
            self._headers('application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == '/events':
            self.send_response(200)
            self._headers('text/event-stream; charset=utf-8')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            client_queue = queue.Queue(maxsize=3)
            self.monitor.add_client(client_queue)
            try:
                initial = json.dumps(self.monitor.snapshot(), ensure_ascii=True)
                self.wfile.write(f'data: {initial}\n\n'.encode('utf-8'))
                self.wfile.flush()
                while True:
                    try:
                        payload = client_queue.get(timeout=15.0)
                        self.wfile.write(f'data: {payload}\n\n'.encode('utf-8'))
                    except queue.Empty:
                        self.wfile.write(b': heartbeat\n\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                self.monitor.remove_client(client_queue)
            return

        self.send_error(404, 'not found')

    def log_message(self, _format, *_args):
        return


class TelemetryWebMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_web_monitor')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8081)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('route_topic', 'stage1_route')
        self.declare_parameter('mission_route_topic', 'stage1_mission_route')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('stage1_log_path', '/home/sunrise/dev_ws/log/competition_stage1/latest.log')
        self.declare_parameter('snapshot_dir', '/home/sunrise/dev_ws/log/telemetry_web_monitor')
        self.declare_parameter('snapshot_period_sec', 0.50)
        self.declare_parameter('history_min_step_m', 0.06)

        self.state = _MonitorState()
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.route_topic = str(self.get_parameter('route_topic').value)
        self.mission_route_topic = str(self.get_parameter('mission_route_topic').value)
        self._map_grid = None
        package_dir = get_package_share_directory('racing_tools')
        bringup_dir = get_package_share_directory('origincar_bringup')
        self.html_path = os.path.join(package_dir, 'web', 'monitor.html')
        self.map_image_path = os.path.join(bringup_dir, 'map', 'map_restricted.png')
        self.stage1_log_path = str(self.get_parameter('stage1_log_path').value)
        self.snapshot_dir = str(self.get_parameter('snapshot_dir').value)
        self.snapshot_period = max(0.2, float(self.get_parameter('snapshot_period_sec').value))
        self.history_min_step = max(0.01, float(self.get_parameter('history_min_step_m').value))
        os.makedirs(self.snapshot_dir, exist_ok=True)
        self.snapshot_image_path = os.path.join(self.snapshot_dir, 'latest.png')
        self.snapshot_state_path = os.path.join(self.snapshot_dir, 'latest_state.json')
        self._last_snapshot_at = 0.0
        self._last_log_size = 0
        self._last_log_tail = ''
        self._last_callback_error_at = 0.0
        self._last_history_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_imu_at = None
        self.last_scan_at = None
        self.last_odom_at = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value), self._map_cb, latched)
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value), self._scan_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self._odom_cb, 10)
        self.create_subscription(Imu, str(self.get_parameter('imu_topic').value), self._imu_cb, 10)
        self.create_subscription(Path, self.route_topic, self._route_cb, latched)
        self.create_subscription(Path, self.mission_route_topic, self._mission_route_cb, latched)
        self.create_subscription(Twist, str(self.get_parameter('cmd_topic').value), self._cmd_cb, 10)
        self.create_subscription(String, 'stage1_state', lambda m: self._stage_cb('stage1', m), latched)
        self.create_subscription(String, 'stage2_state', lambda m: self._stage_cb('stage2', m), latched)
        self.create_subscription(String, 'stage3_state', lambda m: self._stage_cb('stage3', m), latched)
        self.create_subscription(Int32, 'competition_phase', self._phase_cb, latched)
        self.create_subscription(String, 'competition_qr_task', self._qr_cb, latched)
        self.create_timer(0.10, self._publish_state)

        requested_host = str(self.get_parameter('host').value)
        requested_port = int(self.get_parameter('port').value)
        self.bound_port = requested_port
        try:
            self.http = ThreadingHTTPServer(
                (requested_host, requested_port), _RequestHandler,
            )
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
            # A previous launch can leave the requested listener alive for a
            # short time while launch is already starting the next session.
            # Keep telemetry available instead of killing the whole monitor;
            # advertise the actual fallback URL below.
            self.http = None
            for candidate in range(requested_port + 1, requested_port + 21):
                try:
                    self.http = ThreadingHTTPServer(
                        (requested_host, candidate), _RequestHandler,
                    )
                    self.bound_port = candidate
                    break
                except OSError as candidate_exc:
                    if candidate_exc.errno != errno.EADDRINUSE:
                        raise
            if self.http is None:
                raise RuntimeError(
                    f'web monitor ports {requested_port}-{requested_port + 20} are unavailable'
                ) from exc
            self.get_logger().warning(
                f'web monitor requested port {requested_port} is busy; '
                f'using fallback port {self.bound_port}'
            )
        self.http.monitor = self
        self.http.daemon_threads = True
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f'web monitor listening on {requested_host}:{self.bound_port}; '
            f'snapshots={self.snapshot_image_path}'
        )
        for url in self._browser_urls():
            self._terminal_write(f'[WEB] 浏览器实时监视器: {url}/')

    @staticmethod
    def _terminal_write(message):
        """Bypass launch child-output redirection for the browser URL."""
        terminal_path = os.environ.get('RACING_OPERATOR_TTY', '/dev/tty')
        try:
            with open(terminal_path, 'w', encoding='utf-8', buffering=1) as terminal:
                terminal.write(str(message).rstrip() + '\n')
        except (OSError, IOError):
            print(message, flush=True)

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _map_cb(self, msg):
        with self.state.lock:
            self.state.map_meta = {
                'width': int(msg.info.width), 'height': int(msg.info.height),
                'resolution': float(msg.info.resolution),
                'origin_x': float(msg.info.origin.position.x),
                'origin_y': float(msg.info.origin.position.y),
            }
            self._map_grid = (
                int(msg.info.width), int(msg.info.height), float(msg.info.resolution),
                float(msg.info.origin.position.x), float(msg.info.origin.position.y),
                tuple(int(value) for value in msg.data),
            )

    def _is_static_world(self, x, y):
        grid = self._map_grid
        if grid is None:
            return False
        width, height, resolution, origin_x, origin_y, data = grid
        gx = int(math.floor((x - origin_x) / max(resolution, 1e-6)))
        gy = int(math.floor((y - origin_y) / max(resolution, 1e-6)))
        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return True
        value = data[gy * width + gx]
        return value < 0 or value >= 50

    def _odom_cb(self, _msg):
        self.last_odom_at = self._now()

    def _imu_cb(self, msg):
        del msg
        self.last_imu_at = self._now()

    @staticmethod
    def _path_points(msg):
        return [
            {'x': float(pose.pose.position.x), 'y': float(pose.pose.position.y)}
            for pose in msg.poses
        ]

    def _route_cb(self, msg):
        with self.state.lock:
            self.state.route = self._path_points(msg)

    def _mission_route_cb(self, msg):
        with self.state.lock:
            self.state.mission_route = self._path_points(msg)

    def _scan_cb(self, msg):
        self.last_scan_at = self._now()
        with self.state.lock:
            self.state.scan_msg = msg

    def _cmd_cb(self, msg):
        with self.state.lock:
            self.state.cmd = {
                'linear_x': float(msg.linear.x),
                'angular_z': float(msg.angular.z),
            }

    def _stage_cb(self, name, msg):
        with self.state.lock:
            self.state.stage_states[name] = msg.data.strip()

    def _phase_cb(self, msg):
        with self.state.lock:
            self.state.phase = int(msg.data)

    def _qr_cb(self, msg):
        with self.state.lock:
            self.state.qr_task = msg.data.strip()

    def _browser_urls(self):
        port = int(getattr(self, 'bound_port', self.get_parameter('port').value))
        addresses = ['127.0.0.1']
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.connect(('8.8.8.8', 80))
            address = probe.getsockname()[0]
            probe.close()
            if address not in addresses and not address.startswith('127.'):
                addresses.insert(0, address)
        except OSError:
            pass
        try:
            addresses_info = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
            for _, _, _, _, sockaddr in addresses_info:
                address = sockaddr[0]
                if address not in addresses and not address.startswith('127.'):
                    addresses.insert(0, address)
        except OSError:
            pass
        try:
            output = subprocess.check_output(
                ['ip', '-4', '-o', 'addr', 'show'],
                text=True, stderr=subprocess.DEVNULL, timeout=0.5,
            )
            for line in output.splitlines():
                match = re.search(r'\binet\s+([0-9.]+)/', line)
                if match:
                    address = match.group(1)
                    if address not in addresses and not address.startswith('127.'):
                        addresses.insert(0, address)
        except (OSError, subprocess.SubprocessError):
            pass
        return [f'http://{address}:{port}' for address in addresses]

    def _refresh_log_events(self):
        """Tail the S1 file log so failures remain visible without changing S1."""
        try:
            with open(self.stage1_log_path, 'r', encoding='utf-8', errors='replace') as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - 65536), os.SEEK_SET)
                text = stream.read()
        except OSError:
            return
        if size < self._last_log_size:
            with self.state.lock:
                self.state.events.clear()
                self.state.target = None
        if size == self._last_log_size and text == self._last_log_tail:
            return
        self._last_log_size = size
        self._last_log_tail = text
        parsed = []
        for line in text.splitlines():
            match = re.match(r'^\[([A-Z0-9_]+)\]\s+(.*)$', line.strip())
            if not match:
                continue
            tag, message = match.groups()
            parsed.append({'tag': tag, 'message': message, 'at': time.time()})
            target = re.search(r'target_map=\(([-+0-9.]+),([-+0-9.]+)\)\s+name=([^ ]+)', message)
            if target:
                with self.state.lock:
                    self.state.target = {
                        'x': float(target.group(1)), 'y': float(target.group(2)),
                        'name': target.group(3),
                    }
        with self.state.lock:
            existing = {(item['tag'], item['message']) for item in self.state.events}
            for item in parsed:
                key = (item['tag'], item['message'])
                if key not in existing:
                    self.state.events.append(item)
                    existing.add(key)

    def health_snapshot(self):
        with self.state.lock:
            return {
                'ok': True,
                'urls': self._browser_urls(),
                'snapshot': self.snapshot_image_path,
                'health': {
                    'tf': self.state.pose is not None,
                    'imu': self.last_imu_at is not None,
                    'scan': self.last_scan_at is not None,
                    'odom': self.last_odom_at is not None,
                },
            }

    def _lookup_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.03)
            )
            t = tf.transform.translation
            return float(t.x), float(t.y), self._yaw(tf.transform.rotation)
        except TransformException:
            return None

    def _scan_points_base(self):
        """Convert LaserScan returns into base_footprint coordinates."""
        with self.state.lock:
            scan_msg = self.state.scan_msg
        if scan_msg is None:
            return None
        source_frame = (scan_msg.header.frame_id or 'laser').lstrip('/')
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, Time(), timeout=Duration(seconds=0.03)
            )
        except TransformException:
            return None
        rotation = transform.transform.rotation
        transform_yaw = self._yaw(rotation)
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        points = []
        stride = max(1, len(scan_msg.ranges) // 360)
        for index in range(0, len(scan_msg.ranges), stride):
            distance = float(scan_msg.ranges[index])
            if (not math.isfinite(distance) or
                    distance < max(0.03, float(scan_msg.range_min)) or
                    distance > 5.0):
                continue
            angle = float(scan_msg.angle_min) + index * float(scan_msg.angle_increment)
            laser_x, laser_y = distance * math.cos(angle), distance * math.sin(angle)
            base_x = math.cos(transform_yaw) * laser_x - math.sin(transform_yaw) * laser_y + tx
            base_y = math.sin(transform_yaw) * laser_x + math.cos(transform_yaw) * laser_y + ty
            points.append((base_x, base_y, distance))
        return points

    def snapshot(self):
        self._refresh_log_events()
        pose = self._lookup_pose()
        with self.state.lock:
            if pose is not None:
                self.state.pose = pose
            yaw = None if self.state.pose is None else self.state.pose[2]
            scan = []
            nearest_scan = float('inf')
            nearest_forward = float('inf')
            nearest_dynamic = float('inf')
            nearest_forward_dynamic = float('inf')
            static_points = 0
            base_scan = self._scan_points_base()
            if base_scan is not None and self.state.pose is not None and yaw is not None:
                px, py = self.state.pose[:2]
                for base_x, base_y, distance in base_scan:
                    range_m = math.hypot(base_x, base_y)
                    nearest_scan = min(nearest_scan, range_m)
                    if base_x / max(math.hypot(base_x, base_y), 1e-6) >= math.cos(math.radians(38.0)):
                        nearest_forward = min(nearest_forward, range_m)
                    map_x = px + math.cos(yaw) * base_x - math.sin(yaw) * base_y
                    map_y = py + math.sin(yaw) * base_x + math.cos(yaw) * base_y
                    if self._is_static_world(map_x, map_y):
                        static_points += 1
                    else:
                        nearest_dynamic = min(nearest_dynamic, range_m)
                        if base_x / max(range_m, 1e-6) >= math.cos(math.radians(38.0)):
                            nearest_forward_dynamic = min(nearest_forward_dynamic, range_m)
                    lx, ly = base_x, base_y
                    scan.append({
                        'x': px + math.cos(yaw) * lx - math.sin(yaw) * ly,
                        'y': py + math.sin(yaw) * lx + math.cos(yaw) * ly,
                    })
                if (self._last_history_pose is None or
                        math.hypot(self.state.pose[0] - self._last_history_pose[0],
                                   self.state.pose[1] - self._last_history_pose[1])
                        >= self.history_min_step):
                    self.state.history.append((self.state.pose[0], self.state.pose[1]))
                    self._last_history_pose = self.state.pose[:2]
            scan_stats = {
                'points': len(scan),
                'nearest_m': None if nearest_scan == float('inf') else nearest_scan,
                'nearest_forward_m': None if nearest_forward == float('inf') else nearest_forward,
                'nearest_dynamic_m': None if nearest_dynamic == float('inf') else nearest_dynamic,
                'nearest_forward_dynamic_m': None if nearest_forward_dynamic == float('inf') else nearest_forward_dynamic,
                'static_points': static_points,
            }
            snapshot = {
                'time': self._now(), 'pose': None if self.state.pose is None else {
                    'x': self.state.pose[0], 'y': self.state.pose[1],
                    'yaw': yaw,
                },
                'scan': scan, 'scan_stats': scan_stats, 'map': self.state.map_meta,
                'phase': self.state.phase, 'stage_states': dict(self.state.stage_states),
                'qr_task': self.state.qr_task,
                'cmd': dict(self.state.cmd), 'target': self.state.target,
                'route': list(self.state.route),
                'mission_route': list(self.state.mission_route),
                'history': list(self.state.history), 'events': list(self.state.events),
                'health': {
                    'tf': pose is not None,
                    'imu': self.last_imu_at is not None,
                    'scan': self.last_scan_at is not None,
                    'odom': self.last_odom_at is not None,
                },
            }
        self._write_snapshot(snapshot)
        return snapshot

    @staticmethod
    def _map_pixel(meta, x, y, width, height):
        return (int(round((x - meta['origin_x']) / meta['resolution'])),
                int(round(height - (y - meta['origin_y']) / meta['resolution'])))

    def _write_snapshot(self, snapshot):
        now = time.monotonic()
        if now - self._last_snapshot_at < self.snapshot_period:
            return
        self._last_snapshot_at = now
        meta = snapshot.get('map')
        if not meta:
            return
        try:
            image = cv2.imread(self.map_image_path, cv2.IMREAD_COLOR)
            if image is None:
                image = np.full((meta['height'], meta['width'], 3), 245, dtype=np.uint8)
            height, width = image.shape[:2]
            mission_route = snapshot.get('mission_route', [])
            # Mission targets are landmarks/search points, not a drivable
            # polyline.  Connecting them across walls made the web route look
            # like an additional planner output.
            for index, point in enumerate(mission_route):
                px, py = self._map_pixel(meta, point['x'], point['y'], width, height)
                cv2.circle(image, (px, py), 5, (180, 80, 220), -1)
                cv2.putText(image, str(index + 1), (px + 7, py - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 80, 220), 1,
                            cv2.LINE_AA)
            route = snapshot.get('route', [])
            if len(route) >= 2:
                points = [self._map_pixel(meta, p['x'], p['y'], width, height) for p in route]
                for first, second in zip(points, points[1:]):
                    cv2.line(image, first, second, (0, 165, 255), 3, cv2.LINE_AA)
            for point in snapshot.get('scan', []):
                x, y = self._map_pixel(meta, point['x'], point['y'], width, height)
                if 0 <= x < width and 0 <= y < height:
                    cv2.circle(image, (x, y), 2, (0, 0, 255), -1)
            history = snapshot.get('history', [])
            for first, second in zip(history, history[1:]):
                p1 = self._map_pixel(meta, first[0], first[1], width, height)
                p2 = self._map_pixel(meta, second[0], second[1], width, height)
                cv2.line(image, p1, p2, (255, 180, 40), 2)
            pose = snapshot.get('pose')
            if pose:
                px, py = self._map_pixel(meta, pose['x'], pose['y'], width, height)
                cv2.circle(image, (px, py), 7, (0, 180, 0), -1)
                if pose.get('yaw') is not None:
                    heading_len = 0.35 / max(meta['resolution'], 1e-6)
                    hx = int(round(px + heading_len * math.cos(pose['yaw'])))
                    hy = int(round(py - heading_len * math.sin(pose['yaw'])))
                    cv2.arrowedLine(image, (px, py), (hx, hy), (0, 180, 0), 3, tipLength=0.25)
            else:
                cv2.putText(image, 'waiting for map->base_footprint TF', (12, 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 180), 2, cv2.LINE_AA)
            target = snapshot.get('target')
            if target:
                tx, ty = self._map_pixel(meta, target['x'], target['y'], width, height)
                cv2.drawMarker(image, (tx, ty), (255, 0, 255), cv2.MARKER_CROSS, 18, 3)
            overlay_path = os.path.join(self.snapshot_dir, '.latest.tmp.png')
            state_path = os.path.join(self.snapshot_dir, '.latest_state.tmp.json')
            if not cv2.imwrite(overlay_path, image):
                return
            with open(state_path, 'w', encoding='utf-8') as stream:
                json.dump(snapshot, stream, ensure_ascii=False, indent=2)
            os.replace(overlay_path, self.snapshot_image_path)
            os.replace(state_path, self.snapshot_state_path)
        except Exception as exc:
            # A malformed map/scan must not kill the read-only monitor.  The
            # JSON/SSE endpoint remains available even if one image frame is
            # temporarily unavailable.
            self._report_callback_error('snapshot writer', exc)

    def add_client(self, client):
        with self.state.lock:
            self.state.clients.add(client)

    def remove_client(self, client):
        with self.state.lock:
            self.state.clients.discard(client)

    def _publish_state(self):
        try:
            payload = json.dumps(self.snapshot(), ensure_ascii=True, separators=(',', ':'))
            with self.state.lock:
                clients = tuple(self.state.clients)
            for client in clients:
                try:
                    client.put_nowait(payload)
                except queue.Full:
                    try:
                        client.get_nowait()
                        client.put_nowait(payload)
                    except queue.Empty:
                        pass
        except Exception as exc:
            self._report_callback_error('state publisher', exc)

    def _report_callback_error(self, where, exc):
        now = time.monotonic()
        if now - self._last_callback_error_at < 1.0:
            return
        self._last_callback_error_at = now
        self.get_logger().error(
            f'{where} failed; monitor kept alive: {exc}\n{traceback.format_exc()}'
        )

    def destroy_node(self):
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryWebMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
