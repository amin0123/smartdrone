"""Verify native simulator termination cannot leave its flight child running."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def test_owned_child_dies_with_parent():
    launcher = Path(__file__).resolve().parents[1] / 'scripts/owned_process.py'
    code = ("import os, subprocess, sys, time; "
            "child=subprocess.Popen([sys.executable, sys.argv[1], str(os.getpid()), "
            "sys.executable, '-c', 'import time; time.sleep(60)']); "
            "print(child.pid, flush=True); time.sleep(60)")
    parent = subprocess.Popen([sys.executable, '-c', code, str(launcher)],
                              stdout=subprocess.PIPE, text=True)
    child_pid = int(parent.stdout.readline())
    child_stat = Path(f'/proc/{child_pid}/stat')
    try:
        # Wait until exec has completed so this tests death-signal delivery as
        # well as the launcher's already-dead-parent race check.
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            command = Path(f'/proc/{child_pid}/cmdline').read_bytes()
            if b'import time; time.sleep(60)' in command and b'owned_process.py' not in command:
                break
            time.sleep(.01)
        else:
            raise AssertionError('Owned child did not exec')
        parent.kill()
        parent.wait(timeout=3)
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            if not child_stat.exists() or child_stat.read_text().split()[2] == 'Z':
                break
            time.sleep(.01)
        else:
            raise AssertionError('Child survived simulator-parent death')
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait(timeout=3)
        if child_stat.exists() and child_stat.read_text().split()[2] != 'Z':
            os.kill(child_pid, signal.SIGKILL)
