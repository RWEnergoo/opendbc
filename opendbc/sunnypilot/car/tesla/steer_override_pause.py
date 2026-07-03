"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

# Delay after the driver releases a hard override before steering resumes, indexed
# by the 2-bit STEER_OVERRIDE_RESUME_DELAY flag value (TeslaSteerOverrideResumeDelay param)
STEER_OVERRIDE_RESUME_DELAYS = [0.5, 1.0, 1.5, 2.0]  # seconds

# Don't resume while the requested angle is far from the actual angle, so steering
# doesn't snap back mid-correction (e.g. while the driver is still in a curve)
RESUME_MAX_ANGLE_DELTA = 10.0  # degrees

DT_CTRL = 0.01  # carcontroller runs at 100Hz


class SteerOverridePause:
  """Pauses lateral actuation after a hard steering override (EPAS3S_handsOnLevel >= 3)
  and resumes it once the driver has relaxed their grip for a configurable delay and
  the commanded angle is close to the current angle."""

  def __init__(self, CP_SP: structs.CarParamsSP):
    self.enabled = bool(CP_SP.flags & TeslaFlagsSP.STEER_OVERRIDE_PAUSES)

    delay_idx = (1 if CP_SP.flags & TeslaFlagsSP.STEER_OVERRIDE_RESUME_DELAY_BIT0 else 0) + \
                (2 if CP_SP.flags & TeslaFlagsSP.STEER_OVERRIDE_RESUME_DELAY_BIT1 else 0)
    self.resume_delay_frames = int(STEER_OVERRIDE_RESUME_DELAYS[delay_idx] / DT_CTRL)

    self.paused = False
    self.resume_timer = 0

  def update(self, lat_active: bool, latActive: bool, hands_on_level: int, steering_pressed: bool,
             desired_angle: float, actual_angle: float) -> bool:
    if not self.enabled:
      return lat_active

    if not latActive:
      # openpilot lateral is fully off (disengaged or MADS paused elsewhere): reset
      self.paused = False
      self.resume_timer = 0
      return lat_active

    if hands_on_level >= 3:
      self.paused = True
      self.resume_timer = 0
    elif self.paused:
      relaxed = not steering_pressed
      angle_ok = abs(desired_angle - actual_angle) < RESUME_MAX_ANGLE_DELTA
      if relaxed and angle_ok:
        self.resume_timer += 1
      else:
        self.resume_timer = 0

      if self.resume_timer >= self.resume_delay_frames:
        self.paused = False
        self.resume_timer = 0

    return lat_active and not self.paused
