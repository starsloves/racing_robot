#!/usr/bin/env python3
"""Record the complete sensor -> EKF -> TF -> map/control chain.

This node is deliberately passive: it never publishes or changes navigation.
Every received message is written as JSONL together with the current ROS
publisher inventory, so a bad turn can be attributed to a command publisher
or to the first pose/TF edge that diverged.
"""

import json
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from origincar_msg.msg import Data
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from racing_common.session_file_log import SessionFileLog


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat(q):
    return {'x': float(q.x), 'y': float(q.y), 'z': float(q.z), 'w': float(q.w)}


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def vec(v):
    return {'x': float(v.x), 'y': float(v.y), 'z': float(v.z)}


class PoseChainAudit(Node):
    TOPICS = (
        '/robotpose', '/robotvel', '/PowerVoltage', '/odom', '/imu/data_raw', '/imu/data', '/odom_combined', '/scan',
        '/map', '/cmd_vel', '/stage2_cmd_vel', '/lane_cmd_vel',
        '/start_corner_pose_diagnostic', '/tf', '/tf_static',
    )

    def __init__(self):
        super().__init__('pose_chain_audit')
        self.declare_parameter('record_scan_ranges', True)
        self.declare_parameter('record_map_data', True)
        self.declare_parameter('inventory_period_sec', 1.0)
        self.declare_parameter('tf_period_sec', 0.20)
        self.declare_parameter('position_jump_min_m', 0.25)
        self.declare_parameter('position_speed_factor', 1.5)
        self.declare_parameter('position_jump_slack_m', 0.10)
        self.declare_parameter('yaw_jump_min_rad', 0.50)
        self.declare_parameter('yaw_jump_max_dt_sec', 0.50)
        self.declare_parameter('audit_subdir', 'pose_chain_audit')
        self.declare_parameter('audit_filename', 'pose_chain_audit.jsonl')
        self._log = SessionFileLog(
            str(self.get_parameter('audit_subdir').value),
            filename=str(self.get_parameter('audit_filename').value),
            session_title='pose chain audit',
        )
        self._last = {}
        self._inventory = {}
        self._param_client = self.create_client(GetParameters, '/ekf_filter_node/get_parameters')
        self._param_request_pending = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._write({
            'type': 'header',
            'schema': 'pose_chain_audit_v1',
            'pid': os.getpid(),
            'topics': list(self.TOPICS),
            'rules': {
                'odom_position_source': '/odom_combined pose x/y',
                'yaw_source': '/imu/data angular_velocity.z (EKF input)',
                'map_pose_source': 'TF map->odom_combined + odom_combined->base_footprint',
            },
        })

        self._sensor_qos = qos_profile_sensor_data
        self._tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, self._sensor_qos)
        self.create_subscription(Data, '/robotpose', self._robotpose_cb, 10)
        self.create_subscription(Data, '/robotvel', self._robotvel_cb, 10)
        self.create_subscription(Float32, '/PowerVoltage', self._voltage_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self._imu_raw_cb, self._sensor_qos)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, self._sensor_qos)
        self.create_subscription(Odometry, '/odom_combined', self._ekf_cb, self._sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, self._sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Twist, '/stage2_cmd_vel', self._stage2_cmd_cb, 10)
        self.create_subscription(Twist, '/lane_cmd_vel', self._lane_cmd_cb, 10)
        self._latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, '/start_corner_pose_diagnostic', self._diag_cb, self._latched_qos)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, self._latched_qos)
        self.create_subscription(TFMessage, '/tf', self._tf_cb, self._tf_qos)
        self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._inventory_timer = self.create_timer(
            max(0.2, float(self.get_parameter('inventory_period_sec').value)),
            self._inventory_tick,
        )
        self._tf_timer = self.create_timer(
            max(0.05, float(self.get_parameter('tf_period_sec').value)), self._tf_tick)
        self.get_logger().info(f'pose chain audit -> {self._log.path}')

    def _write(self, payload):
        payload.setdefault('recorded_at_sec', self.get_clock().now().nanoseconds / 1e9)
        self._log.write(json.dumps(payload, ensure_ascii=False, separators=(',', ':')))

    def _publisher_info(self, topic):
        try:
            infos = self.get_publishers_info_by_topic(topic, no_mangle=False)
        except Exception as exc:
            return [{'error': f'{type(exc).__name__}: {exc}'}]
        result = []
        for info in infos:
            result.append({
                'node_name': str(info.node_name),
                'node_namespace': str(info.node_namespace),
                'topic_type': str(info.topic_type),
                'qos_reliability': str(info.qos_profile.reliability),
                'qos_durability': str(info.qos_profile.durability),
                'gid': str(info.endpoint_gid),
            })
        return result

    def _inventory_tick(self):
        inventory = {topic: self._publisher_info(topic) for topic in self.TOPICS}
        self._inventory = inventory
        self._write({'type': 'publisher_inventory', 'publishers': inventory})
        if self._param_client.service_is_ready() and not self._param_request_pending:
            request = GetParameters.Request(names=[
                'frequency', 'sensor_timeout', 'world_frame', 'odom_frame',
                'base_link_frame', 'publish_tf', 'odom0', 'odom0_config',
                'imu0', 'imu0_config', 'imu0_relative',
            ])
            self._param_request_pending = True
            future = self._param_client.call_async(request)
            future.add_done_callback(self._param_response)

    @staticmethod
    def _parameter_value(value):
        kind = int(value.type)
        fields = {
            1: 'bool_value', 2: 'integer_value', 3: 'double_value',
            4: 'string_value', 5: 'byte_array_value', 6: 'bool_array_value',
            7: 'integer_array_value', 8: 'double_array_value', 9: 'string_array_value',
        }
        field = fields.get(kind)
        if field is None:
            return None
        value = getattr(value, field)
        return list(value) if isinstance(value, (tuple, list)) else value

    def _param_response(self, future):
        self._param_request_pending = False
        try:
            response = future.result()
            names = [
                'frequency', 'sensor_timeout', 'world_frame', 'odom_frame',
                'base_link_frame', 'publish_tf', 'odom0', 'odom0_config',
                'imu0', 'imu0_config', 'imu0_relative',
            ]
            self._write({'type': 'ekf_parameters', 'node': '/ekf_filter_node',
                         'parameters': {
                             name: self._parameter_value(value)
                             for name, value in zip(names, response.values)
                         }})
        except Exception as exc:
            self._write({'type': 'ekf_parameters_error',
                         'error': f'{type(exc).__name__}: {exc}'})

    def _base(self, topic, msg):
        header = getattr(msg, 'header', None)
        result = {'type': 'message', 'topic': topic, 'publishers': self._inventory.get(topic, [])}
        if header is not None:
            result['header'] = {
                'stamp_sec': stamp_sec(header.stamp),
                'frame_id': str(header.frame_id),
            }
            result['age_sec'] = self.get_clock().now().nanoseconds / 1e9 - stamp_sec(header.stamp)
        return result

    def _odom(self, topic, msg):
        out = self._base(topic, msg)
        p = msg.pose.pose
        t = msg.twist.twist
        out['child_frame_id'] = str(msg.child_frame_id)
        out['pose'] = {
            'position': vec(p.position), 'orientation': quat(p.orientation),
            'yaw_rad': yaw(p.orientation), 'covariance': [float(x) for x in msg.pose.covariance],
        }
        out['twist'] = {
            'linear': vec(t.linear), 'angular': vec(t.angular),
            'covariance': [float(x) for x in msg.twist.covariance],
        }
        return out

    def _odom_cb(self, msg):
        self._write(self._odom('/odom', msg))

    def _raw_data_cb(self, topic, msg):
        self._write({'type': 'message', 'topic': topic,
                     'publishers': self._inventory.get(topic, []),
                     'values': {'x': float(msg.x), 'y': float(msg.y), 'z': float(msg.z)}})

    def _robotpose_cb(self, msg):
        self._raw_data_cb('/robotpose', msg)

    def _robotvel_cb(self, msg):
        self._raw_data_cb('/robotvel', msg)

    def _voltage_cb(self, msg):
        self._write({'type': 'message', 'topic': '/PowerVoltage',
                     'publishers': self._inventory.get('/PowerVoltage', []),
                     'value': float(msg.data)})

    def _ekf_cb(self, msg):
        out = self._odom('/odom_combined', msg)
        current = (stamp_sec(msg.header.stamp), float(msg.pose.pose.position.x),
                   float(msg.pose.pose.position.y), yaw(msg.pose.pose.orientation))
        previous = self._last.get('/odom_combined')
        if previous is not None:
            dt = current[0] - previous[0]
            dx = current[1] - previous[1]
            dy = current[2] - previous[2]
            out['delta'] = {'dt_sec': dt, 'dx_m': dx, 'dy_m': dy,
                            'distance_m': math.hypot(dx, dy),
                            'yaw_delta_rad': math.atan2(math.sin(current[3] - previous[3]), math.cos(current[3] - previous[3]))}
            position_limit = max(
                float(self.get_parameter('position_jump_min_m').value),
                float(self.get_parameter('position_speed_factor').value)
                * abs(float(msg.twist.twist.linear.x)) * max(dt, 0.0)
                + float(self.get_parameter('position_jump_slack_m').value),
            )
            yaw_delta = out['delta']['yaw_delta_rad']
            reasons = []
            if dt <= 0.0:
                reasons.append('inverted_timestamp')
            if math.hypot(dx, dy) > position_limit:
                reasons.append('position_jump')
            if (dt <= float(self.get_parameter('yaw_jump_max_dt_sec').value)
                    and abs(yaw_delta) >= float(self.get_parameter('yaw_jump_min_rad').value)):
                reasons.append('yaw_jump')
            if reasons:
                out['anomaly'] = reasons
        self._last['/odom_combined'] = current
        self._write(out)

    def _imu(self, topic, msg):
        out = self._base(topic, msg)
        out['orientation'] = quat(msg.orientation)
        out['orientation_yaw_rad'] = yaw(msg.orientation)
        out['orientation_covariance'] = [float(x) for x in msg.orientation_covariance]
        out['angular_velocity'] = vec(msg.angular_velocity)
        out['angular_velocity_covariance'] = [float(x) for x in msg.angular_velocity_covariance]
        out['linear_acceleration'] = vec(msg.linear_acceleration)
        out['linear_acceleration_covariance'] = [float(x) for x in msg.linear_acceleration_covariance]
        self._write(out)

    def _imu_raw_cb(self, msg):
        self._imu('/imu/data_raw', msg)

    def _imu_cb(self, msg):
        self._imu('/imu/data', msg)

    def _scan_cb(self, msg):
        out = self._base('/scan', msg)
        out['scan'] = {
            'angle_min_rad': float(msg.angle_min),
            'angle_max_rad': float(msg.angle_max),
            'angle_increment_rad': float(msg.angle_increment),
            'time_increment_sec': float(msg.time_increment),
            'scan_time_sec': float(msg.scan_time),
            'range_min_m': float(msg.range_min), 'range_max_m': float(msg.range_max),
            'intensities': [float(x) for x in msg.intensities],
            'ranges_m': ([float(x) if math.isfinite(float(x)) else None for x in msg.ranges]
                         if bool(self.get_parameter('record_scan_ranges').value) else None),
            'range_count': len(msg.ranges),
        }
        self._write(out)

    def _cmd_cb(self, msg):
        out = self._base('/cmd_vel', msg)
        out['publishers_at_receive'] = self._publisher_info('/cmd_vel')
        out['twist'] = {'linear': vec(msg.linear), 'angular': vec(msg.angular)}
        self._write(out)

    def _stage2_cmd_cb(self, msg):
        out = self._base('/stage2_cmd_vel', msg)
        out['publishers_at_receive'] = self._publisher_info('/stage2_cmd_vel')
        out['twist'] = {'linear': vec(msg.linear), 'angular': vec(msg.angular)}
        self._write(out)

    def _lane_cmd_cb(self, msg):
        out = self._base('/lane_cmd_vel', msg)
        out['publishers_at_receive'] = self._publisher_info('/lane_cmd_vel')
        out['twist'] = {'linear': vec(msg.linear), 'angular': vec(msg.angular)}
        self._write(out)

    def _diag_cb(self, msg):
        self._write(dict(self._base('/start_corner_pose_diagnostic', msg), data=str(msg.data)))

    def _map_cb(self, msg):
        out = self._base('/map', msg)
        out['map'] = {
            'info': {
                'map_load_time_sec': stamp_sec(msg.info.map_load_time),
                'resolution_m': float(msg.info.resolution),
                'width': int(msg.info.width), 'height': int(msg.info.height),
                'origin_position': vec(msg.info.origin.position),
                'origin_orientation': quat(msg.info.origin.orientation),
            },
            'data': ([int(x) for x in msg.data]
                     if bool(self.get_parameter('record_map_data').value) else None),
            'cell_count': len(msg.data),
        }
        self._write(out)

    def _tf_message(self, topic, msg):
        for transform in msg.transforms:
            self._write({
                'type': 'tf', 'topic': topic,
                'publishers': self._inventory.get(topic, []),
                'header': {'stamp_sec': stamp_sec(transform.header.stamp),
                           'frame_id': str(transform.header.frame_id)},
                'child_frame_id': str(transform.child_frame_id),
                'translation': vec(transform.transform.translation),
                'rotation': quat(transform.transform.rotation),
                'yaw_rad': yaw(transform.transform.rotation),
            })

    def _tf_cb(self, msg):
        self._tf_message('/tf', msg)

    def _tf_static_cb(self, msg):
        self._tf_message('/tf_static', msg)

    def _tf_tick(self):
        for parent, child in (
            ('map', 'odom_combined'), ('odom_combined', 'base_footprint'),
            ('map', 'base_footprint'), ('base_footprint', 'base_link'),
            ('base_link', 'laser'),
        ):
            try:
                transform = self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self._write({
                    'type': 'tf_lookup', 'parent': parent, 'child': child,
                    'header': {'stamp_sec': stamp_sec(transform.header.stamp),
                               'frame_id': str(transform.header.frame_id)},
                    'translation': vec(transform.transform.translation),
                    'rotation': quat(transform.transform.rotation),
                    'yaw_rad': yaw(transform.transform.rotation),
                })
            except TransformException as exc:
                self._write({'type': 'tf_lookup_error', 'parent': parent, 'child': child,
                             'error': f'{type(exc).__name__}: {exc}'})

    def destroy_node(self):
        try:
            self._log.close()
        except (AttributeError, OSError):
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseChainAudit()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
