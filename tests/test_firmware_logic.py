"""
test_firmware_logic.py -- unittest suite for src/firmware_logic.py.

Stdlib only. Run from the project root:

    python3 -m unittest discover -s tests -v

Covers sensor range validation (valid / invalid / boundary values),
encoder plausibility and consecutive-fault counting, the watchdog
NORMAL -> DEGRADED -> SAFE_STOP state machine including recovery and
SAFE_STOP latching, and ring-buffer wrap-around behaviour.
"""

import os
import sys
import unittest

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "src"),
)

from firmware_logic import (  # noqa: E402
    ENCODER_MAX_RPM,
    IMU_ACCEL_MAX_G,
    IMU_GYRO_MAX_DPS,
    LIDAR_MAX_M,
    LIDAR_MIN_M,
    FaultRecoveryWatchdog,
    RingBuffer,
    SensorValidator,
    WatchdogState,
    is_plausible_encoder_rpm,
    is_valid_encoder_rpm,
    is_valid_imu_sample,
    is_valid_lidar_range,
)


class TestLidarValidation(unittest.TestCase):
    def test_valid_mid_range_reading(self):
        self.assertTrue(is_valid_lidar_range(5.0))
        self.assertTrue(is_valid_lidar_range(0.5))

    def test_boundary_values_are_valid(self):
        self.assertTrue(is_valid_lidar_range(LIDAR_MIN_M))
        self.assertTrue(is_valid_lidar_range(LIDAR_MAX_M))

    def test_out_of_range_readings_rejected(self):
        self.assertFalse(is_valid_lidar_range(0.0))            # "no return"
        self.assertFalse(is_valid_lidar_range(LIDAR_MIN_M - 0.001))
        self.assertFalse(is_valid_lidar_range(LIDAR_MAX_M + 0.1))
        self.assertFalse(is_valid_lidar_range(-1.0))

    def test_non_finite_and_non_numeric_rejected(self):
        self.assertFalse(is_valid_lidar_range(float("nan")))
        self.assertFalse(is_valid_lidar_range(float("inf")))
        self.assertFalse(is_valid_lidar_range(None))
        self.assertFalse(is_valid_lidar_range("5.0"))


class TestImuValidation(unittest.TestCase):
    def test_valid_sample_at_rest(self):
        # ~1 g on Z (gravity), everything else ~0: a healthy resting IMU.
        self.assertTrue(
            is_valid_imu_sample((0.01, -0.02, 0.99), (0.5, -0.3, 0.1))
        )

    def test_boundary_values_are_valid(self):
        self.assertTrue(
            is_valid_imu_sample(
                (IMU_ACCEL_MAX_G, -IMU_ACCEL_MAX_G, 0.0),
                (IMU_GYRO_MAX_DPS, -IMU_GYRO_MAX_DPS, 0.0),
            )
        )

    def test_over_range_axis_rejected(self):
        self.assertFalse(
            is_valid_imu_sample((IMU_ACCEL_MAX_G + 0.1, 0.0, 0.0), (0, 0, 0))
        )
        self.assertFalse(
            is_valid_imu_sample((0, 0, 0), (0, 0, -IMU_GYRO_MAX_DPS - 1))
        )

    def test_malformed_samples_rejected(self):
        self.assertFalse(is_valid_imu_sample((0.0, 0.0), (0, 0, 0)))
        self.assertFalse(is_valid_imu_sample((0, 0, 0), (0, 0)))
        self.assertFalse(is_valid_imu_sample((float("nan"), 0, 0), (0, 0, 0)))
        self.assertFalse(is_valid_imu_sample(None, (0, 0, 0)))


class TestEncoderValidation(unittest.TestCase):
    def test_valid_speeds_including_reverse_and_limits(self):
        self.assertTrue(is_valid_encoder_rpm(0.0))
        self.assertTrue(is_valid_encoder_rpm(1200.0))
        self.assertTrue(is_valid_encoder_rpm(-1200.0))   # reverse drive
        self.assertTrue(is_valid_encoder_rpm(ENCODER_MAX_RPM))
        self.assertTrue(is_valid_encoder_rpm(-ENCODER_MAX_RPM))

    def test_out_of_range_speeds_rejected(self):
        self.assertFalse(is_valid_encoder_rpm(ENCODER_MAX_RPM + 1))
        self.assertFalse(is_valid_encoder_rpm(-ENCODER_MAX_RPM - 1))
        self.assertFalse(is_valid_encoder_rpm(float("inf")))

    def test_plausible_and_implausible_jumps(self):
        self.assertTrue(is_plausible_encoder_rpm(1000.0, 1800.0))
        self.assertTrue(is_plausible_encoder_rpm(1000.0, 1000.0))
        # 1000 -> 4000 RPM in one control cycle is not physically plausible.
        self.assertFalse(is_plausible_encoder_rpm(1000.0, 4000.0))
        self.assertFalse(is_plausible_encoder_rpm(1000.0, float("nan")))

    def test_validator_counts_consecutive_faults(self):
        v = SensorValidator(fault_threshold=3)
        self.assertTrue(v.check_encoder(1000.0))    # first sample, accepted
        self.assertTrue(v.check_encoder(1500.0))    # plausible step
        self.assertFalse(v.check_encoder(4500.0))   # implausible jump: 1 bad
        self.assertFalse(v.fault_active)
        self.assertFalse(v.check_encoder(4600.0))   # still vs last-good: 2 bad
        self.assertFalse(v.check_encoder(4700.0))   # 3 bad -> fault active
        self.assertTrue(v.fault_active)
        self.assertEqual(v.consecutive_bad, 3)

    def test_validator_recovers_on_good_sample_and_reset(self):
        v = SensorValidator(fault_threshold=2)
        self.assertTrue(v.check_encoder(500.0))
        self.assertFalse(v.check_encoder(4000.0))
        self.assertFalse(v.check_encoder(4100.0))
        self.assertTrue(v.fault_active)
        # A plausible sample (vs last-known-good 500) clears the counter.
        self.assertTrue(v.check_encoder(600.0))
        self.assertFalse(v.fault_active)
        v.reset()
        self.assertEqual(v.consecutive_bad, 0)


class TestWatchdog(unittest.TestCase):
    def make_watchdog(self):
        return FaultRecoveryWatchdog(warning_timeout_ms=100,
                                     fault_timeout_ms=250)

    def test_invalid_timeouts_rejected(self):
        with self.assertRaises(ValueError):
            FaultRecoveryWatchdog(warning_timeout_ms=250,
                                  fault_timeout_ms=250)
        with self.assertRaises(ValueError):
            FaultRecoveryWatchdog(warning_timeout_ms=300,
                                  fault_timeout_ms=250)

    def test_stays_normal_with_regular_heartbeats(self):
        wd = self.make_watchdog()
        wd.feed(0)
        self.assertIs(wd.check(50), WatchdogState.NORMAL)
        wd.feed(60)
        self.assertIs(wd.check(150), WatchdogState.NORMAL)

    def test_warning_timeout_triggers_degraded(self):
        wd = self.make_watchdog()
        wd.feed(0)
        self.assertIs(wd.check(100), WatchdogState.NORMAL)    # at boundary
        self.assertIs(wd.check(101), WatchdogState.DEGRADED)  # just past it

    def test_fault_timeout_triggers_safe_stop(self):
        wd = self.make_watchdog()
        wd.feed(0)
        self.assertIs(wd.check(200), WatchdogState.DEGRADED)
        self.assertIs(wd.check(251), WatchdogState.SAFE_STOP)

    def test_recovery_from_degraded_to_normal_on_heartbeat(self):
        wd = self.make_watchdog()
        wd.feed(0)
        self.assertIs(wd.check(150), WatchdogState.DEGRADED)
        wd.feed(160)  # supervised task alive again
        self.assertIs(wd.check(160), WatchdogState.NORMAL)
        self.assertEqual(
            wd.transitions,
            [WatchdogState.NORMAL, WatchdogState.DEGRADED,
             WatchdogState.NORMAL],
        )

    def test_safe_stop_is_latched_until_explicit_reset(self):
        wd = self.make_watchdog()
        wd.feed(0)
        self.assertIs(wd.check(300), WatchdogState.SAFE_STOP)
        wd.feed(310)  # heartbeats resume...
        self.assertIs(wd.check(310), WatchdogState.SAFE_STOP)  # ...still stopped
        self.assertIs(wd.reset(310), WatchdogState.NORMAL)     # explicit reset

    def test_reset_fails_without_recent_heartbeat(self):
        wd = self.make_watchdog()
        wd.feed(0)
        self.assertIs(wd.check(300), WatchdogState.SAFE_STOP)
        # No new heartbeat: reset must not clear the safe stop.
        self.assertIs(wd.reset(400), WatchdogState.SAFE_STOP)


class TestRingBuffer(unittest.TestCase):
    def test_push_and_fifo_order(self):
        buf = RingBuffer(capacity=3)
        self.assertTrue(buf.is_empty())
        for v in (10, 20, 30):
            buf.push(v)
        self.assertTrue(buf.is_full())
        self.assertEqual(len(buf), 3)
        self.assertEqual(buf.to_list(), [10, 20, 30])
        self.assertEqual(buf.pop_oldest(), 10)
        self.assertEqual(buf.to_list(), [20, 30])

    def test_wraparound_overwrites_oldest(self):
        buf = RingBuffer(capacity=3)
        for v in (1, 2, 3, 4, 5):
            buf.push(v)
        # 1 and 2 were overwritten; order is still oldest -> newest.
        self.assertEqual(buf.to_list(), [3, 4, 5])
        self.assertEqual(buf.peek_oldest(), 3)
        self.assertEqual(buf.peek_newest(), 5)
        self.assertEqual(list(buf), [3, 4, 5])
        buf.push(6)
        self.assertEqual(buf.to_list(), [4, 5, 6])

    def test_pop_until_empty_then_raises(self):
        buf = RingBuffer(capacity=2)
        buf.push("a")
        buf.push("b")
        buf.push("c")  # overwrites "a"
        self.assertEqual(buf.pop_oldest(), "b")
        self.assertEqual(buf.pop_oldest(), "c")
        self.assertTrue(buf.is_empty())
        with self.assertRaises(IndexError):
            buf.pop_oldest()
        with self.assertRaises(IndexError):
            buf.peek_newest()

    def test_clear_and_invalid_capacity(self):
        with self.assertRaises(ValueError):
            RingBuffer(capacity=0)
        buf = RingBuffer(capacity=4)
        buf.push(1)
        buf.push(2)
        buf.clear()
        self.assertTrue(buf.is_empty())
        self.assertEqual(buf.to_list(), [])
        buf.push(9)  # still usable after clear
        self.assertEqual(buf.to_list(), [9])


if __name__ == "__main__":
    unittest.main()
