"""Wall-clock lease prevents a healthy bridge from masking a dead controller."""
import math


class ControllerLease:
    def __init__(self, timeout: float = .5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Lease timeout must be positive')
        self.timeout = timeout
        self.last_received = float('-inf')
        self.last_sequence = -1
        self.started_at = float('inf')

    def accept(self, sequence: int, now: float) -> bool:
        if sequence <= self.last_sequence or not math.isfinite(now):
            return False
        if not self.fresh(now):
            self.started_at = now
        self.last_sequence, self.last_received = sequence, now
        return True

    def fresh(self, now: float) -> bool:
        return 0 <= now - self.last_received <= self.timeout

    def warmed_up(self, now: float, seconds: float = 1.2) -> bool:
        return self.fresh(now) and now - self.started_at >= seconds

    def reset(self) -> None:
        self.last_sequence = -1
        self.last_received = float('-inf')
        self.started_at = float('inf')
