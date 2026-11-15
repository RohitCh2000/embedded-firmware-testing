"""
firmware_logic.py -- Firmware behaviour models for validation testing.

A small, realistic, stdlib-only Python model of three pieces of embedded
firmware behaviour that are commonly exercised in firmware validation /
HIL-style test suites:

1. Sensor reading validation
   Range / plausibility checks for the three sensor types used on a
   typical AMR (autonomous mobile robot) platform:
     * LiDAR   -- distance in metres
     * IMU     -- acceleration (g) and angular rate (deg/s)
     * Encoder -- wheel speed in RPM, plus a plausibility ("jump") check
                  against the previous sample

2. Watchdog / fault-recovery state machine
   NORMAL -> DEGRADED -> SAFE_STOP, with recovery.
   A supervised task must "feed" the watchdog (heartbeat) regularly:
     * heartbeat missed past the warning timeout  -> DEGRADED
     * heartbeat missed past the fault timeout    -> SAFE_STOP
     * heartbeat resumed while DEGRADED           -> back to NORMAL
     * SAFE_STOP is latched: it only recovers via an explicit reset()
       once heartbeats are flowing again -- a robot must not silently
       drive out of a safe stop.

   All timing is driven by caller-supplied timestamps (milliseconds),
   never by the wall clock, so tests are fully deterministic -- exactly
   how you would test this on a HIL rig with a simulated time base.

3. Ring buffer for sensor samples
   A fixed-capacity circular buffer as used in firmware to keep the
   most recent N samples: when full, the oldest sample is overwritten.

Author: personal project (no employer code). Python 3.10+, stdlib only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Generic, Iterable, List, Optional, TypeVar

__all__ = [
    "LIDAR_MIN_M", "LIDAR_MAX_M",
    "IMU_ACCEL_MAX_G", "IMU_GYRO_MAX_DPS",
    "ENCODER_MAX_RPM", "ENCODER_MAX_RPM_JUMP",
    "is_valid_lidar_range",
    "is_valid_imu_sample",
    "is_valid_encoder_rpm",
    "is_plausible_encoder_rpm",
    "SensorValidator",
    "WatchdogState",
    "FaultRecoveryWatchdog",
    "RingBuffer",
]

# ---------------------------------------------------------------------------
# 1. Sensor reading validation
# ---------------------------------------------------------------------------

# LiDAR: typical short/mid-range 2D LiDAR used on AMRs (e.g. 0.02 m blind
# zone, 40 m max range).
LIDAR_MIN_M = 0.02
LIDAR_MAX_M = 40.0

# IMU: consumer/industrial MEMS IMU configured for +/-16 g and
# +/-2000 deg/s full scale (common MPU-9250 / BMI088 style ranges).
IMU_ACCEL_MAX_G = 16.0
IMU_GYRO_MAX_DPS = 2000.0

# Wheel encoder: maximum plausible wheel speed for a warehouse AMR,
# and the maximum plausible change between two consecutive samples
# taken at the normal control-loop rate (a larger jump means a
# wiring / decoding fault, not real acceleration).
ENCODER_MAX_RPM = 5000.0
ENCODER_MAX_RPM_JUMP = 1500.0


def _is_finite_number(value: object) -> bool:
    """True only for real, finite int/float values (not bool/NaN/inf)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def is_valid_lidar_range(distance_m: object) -> bool:
    """Return True if *distance_m* is a plausible LiDAR range reading.

    Valid means: a finite number inside [LIDAR_MIN_M, LIDAR_MAX_M].
    ``None``, NaN, infinities, negative values and the 0.0 that many
    drivers emit for "no return" are all rejected.
    """
    if not _is_finite_number(distance_m):
        return False
    return LIDAR_MIN_M <= float(distance_m) <= LIDAR_MAX_M


def is_valid_imu_sample(
    accel_xyz_g: Iterable[float],
    gyro_xyz_dps: Iterable[float],
) -> bool:
    """Return True if an IMU sample is within configured full scale.

    *accel_xyz_g* and *gyro_xyz_dps* must each be a 3-element sequence.
    Every axis must be finite and within +/-IMU_ACCEL_MAX_G /
    +/-IMU_GYRO_MAX_DPS respectively.  Wrong-length or non-numeric
    samples are rejected rather than raising, mirroring how a firmware
    validation layer drops a corrupt frame.
    """
    try:
        accel = list(accel_xyz_g)  # type: ignore[arg-type]
        gyro = list(gyro_xyz_dps)  # type: ignore[arg-type]
    except TypeError:
        return False
    if len(accel) != 3 or len(gyro) != 3:
        return False
    if not all(_is_finite_number(v) for v in accel + gyro):
        return False
    if any(abs(float(v)) > IMU_ACCEL_MAX_G for v in accel):
        return False
    if any(abs(float(v)) > IMU_GYRO_MAX_DPS for v in gyro):
        return False
    return True


def is_valid_encoder_rpm(rpm: object) -> bool:
    """Return True if *rpm* is finite and within +/-ENCODER_MAX_RPM."""
    if not _is_finite_number(rpm):
        return False
    return -ENCODER_MAX_RPM <= float(rpm) <= ENCODER_MAX_RPM


def is_plausible_encoder_rpm(previous_rpm: float, current_rpm: float) -> bool:
    """Return True if *current_rpm* is a plausible step from *previous_rpm*.

    Both samples must themselves be valid, and the change between them
    must not exceed ENCODER_MAX_RPM_JUMP.  A larger step in one control
    cycle is physically implausible for a loaded AMR wheel and points
    to an encoder decoding / wiring fault.
    """
    if not is_valid_encoder_rpm(previous_rpm):
        return False
    if not is_valid_encoder_rpm(current_rpm):
        return False
    return abs(float(current_rpm) - float(previous_rpm)) <= ENCODER_MAX_RPM_JUMP


@dataclass
class SensorValidator:
    """Stateful validator that also tracks consecutive bad samples.

    Firmware validation layers rarely trip on a single glitch: they
    count consecutive invalid samples and only raise a fault once a
    threshold is reached.  This class models that behaviour for the
    encoder plausibility check; the stateless helpers above cover the
    pure range checks.
    """

    fault_threshold: int = 3
    _last_rpm: Optional[float] = None
    _consecutive_bad: int = 0

    def __post_init__(self) -> None:
        if self.fault_threshold < 1:
            raise ValueError("fault_threshold must be >= 1")

    @property
    def consecutive_bad(self) -> int:
        return self._consecutive_bad

    @property
    def fault_active(self) -> bool:
        """True once consecutive bad samples reach the threshold."""
        return self._consecutive_bad >= self.fault_threshold

    def check_encoder(self, rpm: float) -> bool:
        """Feed one encoder sample; return True if it was accepted.

        A sample is accepted when it is in range and, if a previous
        sample exists, is a plausible step from it.  Rejected samples
        do not update the last-known-good speed.
        """
        accepted = is_valid_encoder_rpm(rpm) and (
            self._last_rpm is None
            or is_plausible_encoder_rpm(self._last_rpm, rpm)
        )
        if accepted:
            self._last_rpm = float(rpm)
            self._consecutive_bad = 0
        else:
            self._consecutive_bad += 1
        return accepted

    def reset(self) -> None:
        self._last_rpm = None
        self._consecutive_bad = 0


# ---------------------------------------------------------------------------
# 2. Watchdog / fault-recovery state machine
# ---------------------------------------------------------------------------

class WatchdogState(Enum):
    NORMAL = auto()
    DEGRADED = auto()
    SAFE_STOP = auto()


class FaultRecoveryWatchdog:
    """Watchdog supervisor with NORMAL / DEGRADED / SAFE_STOP states.

    Parameters
    ----------
    warning_timeout_ms:
        Heartbeat silence longer than this moves NORMAL -> DEGRADED.
    fault_timeout_ms:
        Heartbeat silence longer than this moves any state -> SAFE_STOP.
        Must be strictly greater than *warning_timeout_ms*.

    Usage (all timestamps are caller-supplied, monotonic milliseconds)::

        wd = FaultRecoveryWatchdog(warning_timeout_ms=100,
                                   fault_timeout_ms=250)
        wd.feed(0)            # supervised task is alive at t=0
        wd.check(50)          # -> NORMAL
        wd.check(150)         # -> DEGRADED  (no heartbeat for 150 ms)
        wd.feed(160)          # heartbeat resumes
        wd.check(160)         # -> NORMAL    (recovered from DEGRADED)
        wd.check(500)         # -> SAFE_STOP (340 ms of silence)
        wd.feed(510)
        wd.check(510)         # still SAFE_STOP (latched)
        wd.reset(510)         # explicit reset -> NORMAL
    """

    def __init__(self, warning_timeout_ms: int, fault_timeout_ms: int) -> None:
        if warning_timeout_ms < 0:
            raise ValueError("warning_timeout_ms must be >= 0")
        if fault_timeout_ms <= warning_timeout_ms:
            raise ValueError(
                "fault_timeout_ms must be > warning_timeout_ms"
            )
        self.warning_timeout_ms = int(warning_timeout_ms)
        self.fault_timeout_ms = int(fault_timeout_ms)
        self._state = WatchdogState.NORMAL
        self._last_heartbeat_ms: Optional[int] = None
        self._transitions: List[WatchdogState] = [WatchdogState.NORMAL]

    @property
    def state(self) -> WatchdogState:
        return self._state

    @property
    def transitions(self) -> List[WatchdogState]:
        """History of states entered, starting with NORMAL."""
        return list(self._transitions)

    def _set_state(self, new_state: WatchdogState) -> None:
        if new_state is not self._state:
            self._state = new_state
            self._transitions.append(new_state)

    def feed(self, now_ms: int) -> None:
        """Record a heartbeat from the supervised task at *now_ms*."""
        self._last_heartbeat_ms = int(now_ms)

    def check(self, now_ms: int) -> WatchdogState:
        """Evaluate the watchdog at *now_ms* and return the state.

        SAFE_STOP is latched here: no amount of elapsed-time logic in
        check() leaves SAFE_STOP -- only reset() does.
        """
        if self._state is WatchdogState.SAFE_STOP:
            return self._state
        if self._last_heartbeat_ms is None:
            # Never fed: treat the moment of the first check as the
            # start of supervision rather than faulting immediately.
            self._last_heartbeat_ms = int(now_ms)
            return self._state
        silence_ms = int(now_ms) - self._last_heartbeat_ms
        if silence_ms < 0:
            raise ValueError("timestamps must be monotonic (non-decreasing)")
        if silence_ms > self.fault_timeout_ms:
            self._set_state(WatchdogState.SAFE_STOP)
        elif silence_ms > self.warning_timeout_ms:
            self._set_state(WatchdogState.DEGRADED)
        else:
            self._set_state(WatchdogState.NORMAL)
        return self._state

    def reset(self, now_ms: int) -> WatchdogState:
        """Explicitly reset from SAFE_STOP / DEGRADED to NORMAL.

        The reset only succeeds if a heartbeat has been received at or
        after *now_ms* minus the warning window -- i.e. the supervised
        task is demonstrably alive again.  Otherwise the current state
        is kept and returned.
        """
        if self._last_heartbeat_ms is None:
            return self._state
        silence_ms = int(now_ms) - self._last_heartbeat_ms
        if silence_ms <= self.warning_timeout_ms:
            self._set_state(WatchdogState.NORMAL)
        return self._state


# ---------------------------------------------------------------------------
# 3. Ring buffer for sensor samples
# ---------------------------------------------------------------------------

T = TypeVar("T")


class RingBuffer(Generic[T]):
    """Fixed-capacity circular buffer; oldest items are overwritten.

    Mirrors the ring buffers used in firmware to hold the most recent
    N sensor samples without any dynamic allocation after start-up.

        >>> buf = RingBuffer[int](capacity=3)
        >>> for v in (1, 2, 3, 4):
        ...     buf.push(v)
        >>> buf.to_list()   # oldest -> newest, 1 was overwritten by 4
        [2, 3, 4]
    """

    def __init__(self, capacity: int) -> None:
        if capacity < 1:
            raise ValueError("capacity must be >= 1")
        self._capacity = int(capacity)
        self._data: List[Optional[T]] = [None] * self._capacity
        self._head = 0   # index of the oldest item
        self._size = 0   # number of items currently stored

    @property
    def capacity(self) -> int:
        return self._capacity

    def __len__(self) -> int:
        return self._size

    def is_empty(self) -> bool:
        return self._size == 0

    def is_full(self) -> bool:
        return self._size == self._capacity

    def push(self, item: T) -> None:
        """Append *item*, overwriting the oldest item when full."""
        write_idx = (self._head + self._size) % self._capacity
        self._data[write_idx] = item
        if self._size == self._capacity:
            # Buffer was full: the write just replaced the oldest item,
            # so the oldest item is now the next slot.
            self._head = (self._head + 1) % self._capacity
        else:
            self._size += 1

    def peek_oldest(self) -> T:
        """Return (without removing) the oldest item."""
        if self.is_empty():
            raise IndexError("peek from empty RingBuffer")
        return self._data[self._head]  # type: ignore[return-value]

    def peek_newest(self) -> T:
        """Return (without removing) the newest item."""
        if self.is_empty():
            raise IndexError("peek from empty RingBuffer")
        idx = (self._head + self._size - 1) % self._capacity
        return self._data[idx]  # type: ignore[return-value]

    def pop_oldest(self) -> T:
        """Remove and return the oldest item (FIFO order)."""
        if self.is_empty():
            raise IndexError("pop from empty RingBuffer")
        item = self._data[self._head]
        self._data[self._head] = None
        self._head = (self._head + 1) % self._capacity
        self._size -= 1
        return item  # type: ignore[return-value]

    def to_list(self) -> List[T]:
        """Return stored items as a list, oldest -> newest."""
        return [
            self._data[(self._head + i) % self._capacity]  # type: ignore[misc]
            for i in range(self._size)
        ]

    def __iter__(self):
        return iter(self.to_list())

    def clear(self) -> None:
        self._data = [None] * self._capacity
        self._head = 0
        self._size = 0
