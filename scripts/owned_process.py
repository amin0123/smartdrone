#!/usr/bin/env python3
"""Linux child process that cannot outlive its simulator parent.

Kit can exit inside its native shutdown handler, bypassing Python finally blocks.
The kernel death signal covers that path and a simulator crash. Only the child
started by this launcher is affected; never discover/kill other PX4 instances.
"""
import ctypes
import os
import signal
import sys


def main():
    parent_pid, command = int(sys.argv[1]), sys.argv[2:]
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), 'Cannot bind child lifetime to simulator')
    # Close the race if the parent died before prctl was installed.
    if os.getppid() != parent_pid:
        raise SystemExit('Simulator exited before its PX4 child started')
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
