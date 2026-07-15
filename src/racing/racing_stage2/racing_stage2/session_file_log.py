"""Session file logger under workspace ``log/<subdir>/``.

Each run opens the target file with mode ``w`` (truncate), so the new session
overwrites the previous ``latest.log``.
"""

import os
from datetime import datetime


def resolve_workspace_root():
    cwd = os.getcwd()
    if os.path.isdir(os.path.join(cwd, 'src', 'racing')):
        return cwd
    dev_ws = os.environ.get('DEV_WS', '').strip()
    if dev_ws and os.path.isdir(dev_ws):
        return dev_ws
    return cwd


class SessionFileLog:
    """Append-only session log; opened with mode ``w`` so each run replaces the previous file."""

    def __init__(
        self,
        subdir,
        filename='latest.log',
        workspace_root=None,
        session_title='session',
    ):
        root = workspace_root or resolve_workspace_root()
        self.log_dir = os.path.join(root, 'log', subdir)
        os.makedirs(self.log_dir, exist_ok=True)
        self.path = os.path.join(self.log_dir, filename)
        self._file = open(self.path, 'w', encoding='utf-8')
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.write(f'=== {session_title} {stamp} ===')
        self.write(f'log_path: {self.path}')

    def write(self, line):
        if self._file is None:
            return
        self._file.write(str(line).rstrip() + '\n')
        self._file.flush()

    def close(self):
        if self._file is None:
            return
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.write(f'=== session closed {stamp} ===')
        self._file.close()
        self._file = None
