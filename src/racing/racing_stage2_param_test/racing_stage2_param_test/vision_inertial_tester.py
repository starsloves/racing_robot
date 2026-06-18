#!/usr/bin/env python3
"""
vision_inertial_tester.py — 纯视觉赛道控制（基于 plan 段序列）

原理：
  - 保留 plan 的段序列（build_ring_plan），知道什么时候该转弯
  - turn 段：开环定时转向，ω=0.75 rad/s
  - move 段：视觉赛道居中 PID，取代航向纠偏
  - 避障：激光减速 + 视觉保持居中
  - 日志：~/log/vision_record/vision_{时间戳}.csv
"""

import math
import os
import time
import csv
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from racing_stage2_param_test.lane_follow import VisionLaneEngine
from racing_stage2_param_test.direct_inertial_tester import DirectInertialTester


class VisionInertialTester(DirectInertialTester):
    """纯视觉赛道控制（基于 plan 段序列）。"""

    def __init__(self):
        # 缺省值（防 control loop 提前崩溃）
        self.vision_enabled = False
        self.vision_kp = 1.2
        self.vision_kd = 0.4
        self.vision_ki = 0.02
        self.vision_max_ang = 0.8
        self.vision_lost_to = 0.6
        self.engine = None
        self._offset = 0.0
        self._prev_offset = 0.0
        self._integral = 0.0
        self._lost_at = time.time()
        self._setup_done = False
        self.bridge = CvBridge()

        super().__init__()

        # 声明参数
        for name, default in [
            ('vision_enabled',         True),
            ('vision_model_path',      ''),
            ('vision_camera_topic',    '/aurora/rgb/image_raw'),
            ('vision_conf_threshold',  0.3),
            ('vision_mask_threshold',  0.5),
            ('vision_roi_bottom',      0.35),
            ('vision_kp_center',       1.2),
            ('vision_kd_center',       0.4),
            ('vision_ki_center',       0.02),
            ('vision_max_angular',     0.8),
            ('vision_lost_timeout',    0.6),
            ('vision_log_dir',         'log/vision_record'),
            ('vision_enable_log',      True),
        ]:
            self.declare_parameter(name, default)

        g = self.get_parameter
        self.vision_enabled = g('vision_enabled').value
        self.vision_kp      = g('vision_kp_center').value
        self.vision_kd      = g('vision_kd_center').value
        self.vision_ki      = g('vision_ki_center').value
        self.vision_max_ang = g('vision_max_angular').value
        self.vision_lost_to = g('vision_lost_timeout').value

        mp = g('vision_model_path').value or self._default_model_path()
        self.engine = VisionLaneEngine(
            mp, conf_thr=g('vision_conf_threshold').value,
            mask_thr=g('vision_mask_threshold').value,
            roi_bottom=g('vision_roi_bottom').value,
            logger=self.get_logger(),
        )

        cam_group = ReentrantCallbackGroup()
        self.create_subscription(
            Image, g('vision_camera_topic').value, self._cam_cb, 10,
            callback_group=cam_group,
        )
        self.vision_viz_pub = self.create_publisher(Image, '/lane_seg_viz', 10)
        self.vision_mask_pub = self.create_publisher(Image, '/lane_seg_mask', 10)

        # 日志
        self._log_writer = None
        self._log_file = None
        if g('vision_enable_log').value:
            self._init_log()

        self._setup_done = True
        self.get_logger().info(
            f'[Vision] enabled={self.vision_enabled} '
            f'engine={"OK" if self.engine.ready else "FAIL"}'
        )

    @staticmethod
    def _default_model_path():
        return os.path.normpath(os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'models', 'saidao_seg_model_quant.bin'
        ))

    def _init_log(self):
        d = os.path.expanduser(os.path.join('~', self.vision_log_dir)) if hasattr(self, 'vision_log_dir') else \
            os.path.expanduser('~/log/vision_record')
        os.makedirs(d, exist_ok=True)
        p = os.path.join(d, f'vision_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        try:
            self._log_file = open(p, 'w', newline='')
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow([
                't', 'elapsed', 'has_det', 'offset', 'P', 'I', 'D',
                'cmd_lin', 'cmd_ang', 'plan_i', 'seg_type', 'seg_desc',
                'front', 'left', 'right',
            ])
            self.get_logger().info(f'[Vision] 日志: {p}')
        except Exception as e:
            self.get_logger().error(f'[Vision] 日志失败: {e}')

    def _write_log(self, offset, P, I, D, linear, angular):
        if not self._log_writer:
            return
        try:
            seg = self.current_segment or {}
            self._log_writer.writerow([
                f'{time.time():.3f}',
                f'{time.time() - time.time():.1f}',
                '1' if offset is not None else '0',
                f'{offset:.4f}' if offset is not None else '',
                f'{P:.4f}', f'{I:.4f}', f'{D:.4f}',
                f'{linear:.3f}', f'{angular:.3f}',
                self.plan_index,
                seg.get('type', ''),
                seg.get('description', ''),
                f'{self.front_obstacle_distance:.3f}' if hasattr(self, 'front_obstacle_distance') else '',
                '', '',
            ])
            self._log_file.flush()
        except Exception:
            pass

    def cleanup(self):
        if self._log_file:
            self._log_file.close()

    # ── 相机回调 ─────────────────────────────────────────

    def _cam_cb(self, msg):
        if not self._setup_done or not self.engine or not self.engine.ready:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        t0 = time.time()
        r = self.engine.process(cv_img)
        infer_dt = time.time() - t0
        if infer_dt > 0.05:
            self.get_logger().warn(
                f'[BPU] inference {infer_dt*1000:.0f}ms (slow, may starve control loop)',
                throttle_duration_sec=2.0,
            )
        if r['has_detection'] and r['center_offset'] is not None:
            self._offset = r['center_offset']
            self._lost_at = time.time()
        # 可视化
        try:
            ts = self.get_clock().now().to_msg()
            for pub, key in [(self.vision_viz_pub, 'viz_overlay'),
                             (self.vision_mask_pub, 'viz_mask')]:
                m = self.bridge.cv2_to_imgmsg(r[key], 'bgr8')
                m.header.stamp = ts; m.header.frame_id = 'camera'
                pub.publish(m)
        except Exception:
            pass

    # ── 段切换 ───────────────────────────────────────────

    def start_segment(self, index):
        super().start_segment(index)
        if self.current_segment and self.current_segment.get('type') == 'turn':
            a = float(self.current_segment.get('angle_deg', 0))
            dur = abs(a) * (math.pi / 180.0) / self.turn_angular_speed
            total = dur * 1.15 + 0.5
            self.get_logger().info(
                f'[Vision] 转向 {a:+.0f}° ω={self.turn_angular_speed:.2f} '
                f'{dur:.1f}s+余量={total:.1f}s'
            )

    # ── turn：开环定时（正确参数） ──────────────────────

    def run_turn_segment(self):
        seg = self.current_segment
        a = float(seg.get('angle_deg', 0)) if seg else 0
        ls = float(seg.get('turn_linear_speed', self.turn_linear_speed))

        now = self.get_clock().now().nanoseconds / 1e9
        elap = (now - self.segment_started_at) if self.segment_started_at else 0
        dur = abs(a) * (math.pi / 180.0) / self.turn_angular_speed
        total = dur * 1.15 + 0.5

        self.get_logger().info(
            f'[Vision] 开环 {a:+.0f}° {elap:.1f}s/{total:.1f}s',
            throttle_duration_sec=0.5,
        )

        if elap >= total:
            # 完成后更新 current_yaw（开环估计）
            if self.current_yaw is not None:
                self.current_yaw = self.normalize_angle(
                    self.current_yaw + math.radians(a)
                )
            desc = seg.get('description', '?') if seg else '?'
            self.get_logger().info(f'[Vision] 转向完成 {desc} @ {elap:.2f}s')
            self.cmd_pub.publish(self.create_twist())
            self.start_segment(self.plan_index + 1)
            return

        ang = math.copysign(self.turn_angular_speed, a)
        self.cmd_pub.publish(self.create_twist(ls, ang))

    # ── move：视觉居中 PID ─────────────────────────────

    def _compute_move_lateral_angular(self) -> float:
        if (not self._setup_done or not self.vision_enabled
                or not self.engine or not self.engine.ready):
            return super()._compute_move_lateral_angular()

        if time.time() - self._lost_at > self.vision_lost_to:
            self._integral = 0.0
            return super()._compute_move_lateral_angular()

        off = self._offset
        dt = 1.0 / max(self.control_rate_hz, 1.0)
        self._integral += off * dt
        self._integral = float(np.clip(self._integral, -10, 10))
        deriv = (off - self._prev_offset) / max(dt, 1e-6)

        P = self.vision_kp * off
        I = self.vision_ki * self._integral
        D = self.vision_kd * deriv
        ang = -(P + I + D)
        ang = float(np.clip(ang, -self.vision_max_ang, self.vision_max_ang))
        self._prev_offset = off

        # 日志
        self.get_logger().info(
            f'[Vision] offset={off:+.3f} P={P:+.3f} D={D:+.3f} ang={ang:+.3f}',
            throttle_duration_sec=0.8,
        )
        self._write_log(off, P, I, D,
                        float(self.current_segment.get('speed', 0)) if self.current_segment else 0,
                        ang)
        return ang


def main(args=None):
    rclpy.init(args=args)
    node = VisionInertialTester()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
