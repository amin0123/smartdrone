"""Bounded transport wait around Pegasus's existing PX4 actuator bridge."""
import time
from pymavlink import mavutil
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend


class BoundedPX4Backend(PX4MavlinkBackend):
    def __init__(self, config, receive_timeout=5.):
        super().__init__(config)
        self.receive_timeout, self.link_fault = receive_timeout, None

    def poll_mavlink_messages(self):
        if self.link_fault or not self._received_first_hearbeat:
            return
        blocking = self._received_first_actuator and self._enable_lockstep
        self._received_actuator = False
        deadline = time.monotonic()+self.receive_timeout
        try:
            while True:
                msg = self._connection.recv_match(blocking=blocking, timeout=.05)
                if msg is not None and msg.id == mavutil.mavlink.MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                    self._received_first_actuator = self._received_actuator = True
                    self.handle_control(msg.time_usec, msg.controls, msg.mode, msg.flags)
                if not blocking or self._received_actuator:
                    return
                if time.monotonic() >= deadline:
                    self.link_fault = 'PX4 actuator stream timed out during lockstep'
                    return
        except (OSError, EOFError) as exc:
            self.link_fault = f'PX4 transport failed: {exc}'
