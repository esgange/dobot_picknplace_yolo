"""Run an owned detector; terminate it if the controller GUI disappears."""

import ctypes
import os
import signal
import sys


def main():
    parent, executable = int(sys.argv[1]), sys.argv[2]
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:  # Linux PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), "Cannot bind detector lifetime to GUI")
    if os.getppid() != parent:
        return  # Parent exited between spawn and prctl.
    os.execv(executable, [executable])


if __name__ == '__main__':
    main()
