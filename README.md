# ✅ Embedded Firmware Testing

![Python](https://img.shields.io/badge/Python-3-blue) ![Tests](https://img.shields.io/badge/tests-24%20passing-success) ![GoogleTest concepts](https://img.shields.io/badge/testing-unittest-brightgreen)

Automated firmware-validation examples: sensor plausibility checks, a **watchdog / fault-recovery state machine**, and a ring buffer — with a full Python test suite.

*Personal project based on professional firmware verification experience (150+ automated test cases at Honeywell) — no employer code.*

## ✨ Highlights
- **24 tests, all passing** (`Ran 24 tests — OK`)
- Watchdog state machine: NORMAL → DEGRADED → SAFE_STOP, with latched safe stop and heartbeat recovery
- Sensor validation for LiDAR, IMU and encoders, including boundary and malformed-input cases

## Code, Build & Sample Output

Real, runnable code — not just a README scaffold:

- `src/firmware_logic.py` — firmware-behaviour module (Python, stdlib only):
  sensor reading validation for LiDAR (0.02–40 m), IMU (±16 g / ±2000 °/s)
  and wheel encoders (±5000 RPM plus a plausibility/jump check and a
  consecutive-fault counter), a watchdog / fault-recovery state machine
  (`NORMAL -> DEGRADED -> SAFE_STOP`, latched safe stop, heartbeat-driven
  recovery, deterministic caller-supplied timestamps), and a fixed-capacity
  ring buffer for sensor samples with wrap-around.
- `tests/test_firmware_logic.py` — 24 `unittest` test cases covering valid /
  invalid / boundary sensor values, watchdog timeout triggering
  `SAFE_STOP`, recovery to `NORMAL`, safe-stop latching and reset, and ring
  buffer wrap-around, FIFO order and empty-buffer errors.

**Run it** (no pip installs needed — Python 3 stdlib only), from the
project root:

```bash
python3 -m unittest discover -s tests -v
```

**Sample output** (actual captured run, also saved in `sample_output.txt`):

```text
test_out_of_range_speeds_rejected (test_firmware_logic.TestEncoderValidation.test_out_of_range_speeds_rejected) ... ok
test_plausible_and_implausible_jumps (test_firmware_logic.TestEncoderValidation.test_plausible_and_implausible_jumps) ... ok
test_valid_speeds_including_reverse_and_limits (test_firmware_logic.TestEncoderValidation.test_valid_speeds_including_reverse_and_limits) ... ok
test_validator_counts_consecutive_faults (test_firmware_logic.TestEncoderValidation.test_validator_counts_consecutive_faults) ... ok
test_validator_recovers_on_good_sample_and_reset (test_firmware_logic.TestEncoderValidation.test_validator_recovers_on_good_sample_and_reset) ... ok
test_boundary_values_are_valid (test_firmware_logic.TestImuValidation.test_boundary_values_are_valid) ... ok
test_malformed_samples_rejected (test_firmware_logic.TestImuValidation.test_malformed_samples_rejected) ... ok
test_over_range_axis_rejected (test_firmware_logic.TestImuValidation.test_over_range_axis_rejected) ... ok
test_valid_sample_at_rest (test_firmware_logic.TestImuValidation.test_valid_sample_at_rest) ... ok
test_boundary_values_are_valid (test_firmware_logic.TestLidarValidation.test_boundary_values_are_valid) ... ok
test_non_finite_and_non_numeric_rejected (test_firmware_logic.TestLidarValidation.test_non_finite_and_non_numeric_rejected) ... ok
test_out_of_range_readings_rejected (test_firmware_logic.TestLidarValidation.test_out_of_range_readings_rejected) ... ok
test_valid_mid_range_reading (test_firmware_logic.TestLidarValidation.test_valid_mid_range_reading) ... ok
test_clear_and_invalid_capacity (test_firmware_logic.TestRingBuffer.test_clear_and_invalid_capacity) ... ok
test_pop_until_empty_then_raises (test_firmware_logic.TestRingBuffer.test_pop_until_empty_then_raises) ... ok
test_push_and_fifo_order (test_firmware_logic.TestRingBuffer.test_push_and_fifo_order) ... ok
test_wraparound_overwrites_oldest (test_firmware_logic.TestRingBuffer.test_wraparound_overwrites_oldest) ... ok
test_fault_timeout_triggers_safe_stop (test_firmware_logic.TestWatchdog.test_fault_timeout_triggers_safe_stop) ... ok
test_invalid_timeouts_rejected (test_firmware_logic.TestWatchdog.test_invalid_timeouts_rejected) ... ok
test_recovery_from_degraded_to_normal_on_heartbeat (test_firmware_logic.TestWatchdog.test_recovery_from_degraded_to_normal_on_heartbeat) ... ok
test_reset_fails_without_recent_heartbeat (test_firmware_logic.TestWatchdog.test_reset_fails_without_recent_heartbeat) ... ok
test_safe_stop_is_latched_until_explicit_reset (test_firmware_logic.TestWatchdog.test_safe_stop_is_latched_until_explicit_reset) ... ok
test_stays_normal_with_regular_heartbeats (test_firmware_logic.TestWatchdog.test_stays_normal_with_regular_heartbeats) ... ok
test_warning_timeout_triggers_degraded (test_firmware_logic.TestWatchdog.test_warning_timeout_triggers_degraded) ... ok

----------------------------------------------------------------------
Ran 24 tests in 0.002s

OK
```

All 24 tests pass.


---
*Created by Rohith Reddy Chinthala — Embedded Software Engineer at Honeywell, Saint Louis, MO.*
