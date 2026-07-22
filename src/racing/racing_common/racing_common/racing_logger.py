"""Unified logging and terminal output module for racing robot.

Usage::

    from racing_common.racing_logger import RacingLogger

    # In ROS node __init__:
    self.logger = RacingLogger(self, 'direct_inertial_test')

    # Later:
    self.logger.config('方向=顺时针 field_track=...')
    self.logger.segment('#3 type=move desc=rect_top L=2.59m')
    self.logger.feedback('开始执行，方向: 顺时针')
    self.logger.telemetry('wheel_odom', 't=123.45 mission=1 ...')
```

Output format: ``[TAG] message`` — written to both ROS logger and file.
"""

from .session_file_log import SessionFileLog


class RacingLogger:
    """Unified logger — writes [TAG] message to ROS logger + file."""

    def __init__(self, node, log_subdir='default', log_filename='latest.log',
                 session_title='session', defer_file=False):
        self._node = node
        self._log_subdir = log_subdir
        self._log_filename = log_filename
        self._session_title = session_title
        self._file_log = None
        if not defer_file:
            self.start_session()

    def start_session(self):
        """Open a new file session, replacing the previous latest log."""
        if self._file_log is None:
            self._file_log = SessionFileLog(
                self._log_subdir, self._log_filename,
                session_title=self._session_title,
            )

    # ── path property for callers that need the log file path ──

    @property
    def path(self):
        return None if self._file_log is None else self._file_log.path

    # ── low-level write ──

    def _write(self, tag, message, level='info', file_only=False):
        formatted = f'[{tag}] {message}'
        if not file_only:
            if level == 'info':
                self._node.get_logger().info(formatted)
            elif level == 'warn':
                self._node.get_logger().warn(formatted)
            elif level == 'error':
                self._node.get_logger().error(formatted)
        if self._file_log is not None:
            self._file_log.write(formatted)

    # ── public: generic tag ──

    def info(self, tag, message, file_only=False):
        self._write(tag, message, 'info', file_only=file_only)

    def warn(self, tag, message):
        self._write(tag, message, 'warn')

    def error(self, tag, message):
        self._write(tag, message, 'error')

    # ── shortcut: Stage2 tags ──
    # 默认 file_only=True 的：配置、启动、里程锚点、计划、段完成（仅文件）
    # 默认终端显示的：段、进度、反馈、任务、避障、超时

    def config(self, message):
        self._write('CONFIG', message, 'info', file_only=True)

    def startup(self, message):
        self._write('STARTUP', message, 'info', file_only=True)

    def mission(self, message):
        self._write('MISSION', message, 'info')

    def segment(self, message):
        self._write('SEGMENT', message, 'info')

    def progress(self, message):
        self._write('PROGRESS', message, 'info', file_only=True)

    def feedback(self, message):
        self._write('FEEDBACK', message, 'info')

    def telemetry(self, reason, message):
        self._write('TELEM', f'{reason} | {message}', 'info', file_only=True)

    def odom_wheel(self, message):
        self._write('ODOM_WHEEL', message, 'info', file_only=True)

    def odom_anchor(self, message):
        self._write('ODOM_ANCHOR', message, 'info', file_only=True)

    def plan(self, message):
        self._write('PLAN', message, 'info', file_only=True)

    def segment_done(self, message):
        self._write('SEGMENT_DONE', message, 'info', file_only=True)

    def corner_avoid(self, message):
        self._write('CORNER_AVOID', message, 'info')

    def timeout(self, message):
        self._write('TIMEOUT', message, 'warn')

    # ── shortcuts: Stage3 tags ──

    def stage3_ready(self, message):
        self._write('STAGE3', message, 'info')

    def stage3_plan(self, message):
        self._write('STAGE3_PLAN', message, 'info')

    def stage3_waypoint(self, message):
        self._write('STAGE3_WP', message, 'info')

    def stage3_param(self, message):
        self._write('STAGE3_PARAM', message, 'error')

    # ── lifecycle ──

    def close(self):
        if self._file_log is not None:
            self._file_log.close()
            self._file_log = None
