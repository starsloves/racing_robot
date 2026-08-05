"""Process lifetime helpers for nodes launched by the competition supervisor."""

import ctypes
import os
import signal


_LIBC = ctypes.CDLL(None)


def install_parent_death_signal(signal_number=signal.SIGTERM):
    """Terminate this node when its launch parent disappears on Linux."""
    parent_pid = os.getppid()
    try:
        # Linux PR_SET_PDEATHSIG = 1.
        if _LIBC.prctl(1, int(signal_number)) != 0:
            return False
        # Cover the fork/exec race where the parent died before prctl().
        if os.getppid() != parent_pid:
            os.kill(os.getpid(), signal_number)
        return True
    except (AttributeError, OSError):
        return False
