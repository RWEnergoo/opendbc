"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

# Base delay after the resume conditions are continuously met before steering resumes,
# indexed by the 2-bit STEER_OVERRIDE_RESUME_DELAY flag value (TeslaSteerOverrideResumeDelay param)
STEER_OVERRIDE_RESUME_DELAYS = [0.25, 0.5, 1.0, 2.0]  # seconds

# Resume conditions (road test route 00000011 showed the v1 conditions caused a violent
# pause/resume war at 100-500ms intervals while the driver was mid-maneuver):
# - the wheel must actually be at rest, not mid-motion
RESUME_MAX_WHEEL_RATE = 15.0  # deg/s
# - the driver's grip must be genuinely released (raw torsion torque, not the filtered flag)
RESUME_MAX_TORQUE = 0.8  # Nm
# - the requested angle must be close to the actual angle, scaled with speed: 10 deg at
#   parking speed is harmless, at highway speed it is a violent snap
RESUME_ANGLE_DELTA_BP = [0.0, 3.0, 10.0, 20.0, 30.0]  # m/s
RESUME_ANGLE_DELTA_V = [20.0, 12.0, 6.0, 3.0, 1.5]  # deg

# A firm sustained grip (handsOnLevel 2) pauses too, so the EPS stops fighting the driver
# with full force long before the hard hands-on-3 threshold
PAUSE_FIRM_GRIP_FRAMES = 30  # 0.3s at 100Hz

# If steering resumes and the driver immediately overrides again, the resume was premature:
# back off exponentially instead of fighting at 10Hz
BACKOFF_WINDOW_FRAMES = 300   # re-override within 3s of a resume doubles the required quiet time
BACKOFF_MAX_MULT = 8
BACKOFF_RESET_FRAMES = 1000   # 10s without any override resets the backoff

# In MADS-lateral-only mode a hard override makes MADS itself pause (latActive drops) and
# silently re-enable as soon as handsOnLevel dips below 3. Resetting this helper on that
# brief latActive dip erased the pause and backoff, causing recurring full-force fights in
# tight low-speed curves (road test route 00000018). Keep state across short dips; only
# reset after lateral has been off for a sustained period (a real disengage).
RESET_AFTER_INACTIVE_FRAMES = 500  # 5s

DT_CTRL = 0.01  # carcontroller runs at 100Hz


class SteerOverridePause:
  """Pauses lateral actuation on a driver steering override and resumes it only once the
  wheel is at rest, the grip is released, and the requested angle is close - so steering
  hands back exactly when the driver has put the wheel where openpilot wants it."""

  def __init__(self, CP_SP: structs.CarParamsSP):
    self.enabled = bool(CP_SP.flags & TeslaFlagsSP.STEER_OVERRIDE_PAUSES)

    delay_idx = (1 if CP_SP.flags & TeslaFlagsSP.STEER_OVERRIDE_RESUME_DELAY_BIT0 else 0) + \
                (2 if CP_SP.flags & TeslaFlagsSP.STEER_OVERRIDE_RESUME_DELAY_BIT1 else 0)
    self.resume_delay_frames = int(STEER_OVERRIDE_RESUME_DELAYS[delay_idx] / DT_CTRL)

    self.paused = False
    self.resume_timer = 0
    self.firm_grip_frames = 0
    self.backoff_mult = 1
    self.frames_since_resume = BACKOFF_WINDOW_FRAMES
    self.frames_since_override = BACKOFF_RESET_FRAMES
    self.inactive_frames = 0

  def update(self, lat_active: bool, latActive: bool, hands_on_level: int, steering_disengage: bool,
             v_ego: float, desired_angle: float, actual_angle: float,
             steering_torque: float, steering_rate: float, standstill: bool) -> bool:
    if not self.enabled:
      return lat_active

    if not latActive:
      # keep the pause and backoff across brief dips (MADS pausing/re-enabling on the same
      # override); only a sustained lateral-off is a real disengage worth resetting for
      self.inactive_frames += 1
      if self.inactive_frames >= RESET_AFTER_INACTIVE_FRAMES:
        self.paused = False
        self.resume_timer = 0
        self.firm_grip_frames = 0
        self.backoff_mult = 1
      return lat_active
    self.inactive_frames = 0

    self.firm_grip_frames = self.firm_grip_frames + 1 if hands_on_level >= 2 else 0
    override = hands_on_level >= 3 or steering_disengage or self.firm_grip_frames >= PAUSE_FIRM_GRIP_FRAMES

    if override:
      if not self.paused and self.frames_since_resume < BACKOFF_WINDOW_FRAMES:
        # the previous resume was premature: require a longer quiet period next time
        self.backoff_mult = min(self.backoff_mult * 2, BACKOFF_MAX_MULT)
      self.paused = True
      self.resume_timer = 0
      self.frames_since_override = 0
    else:
      self.frames_since_override += 1
      if self.frames_since_override >= BACKOFF_RESET_FRAMES:
        self.backoff_mult = 1

      if self.paused:
        grip_released = hands_on_level <= 1 and abs(steering_torque) < RESUME_MAX_TORQUE
        wheel_at_rest = abs(steering_rate) < RESUME_MAX_WHEEL_RATE
        angle_ok = abs(desired_angle - actual_angle) < float(np.interp(v_ego, RESUME_ANGLE_DELTA_BP, RESUME_ANGLE_DELTA_V))

        # never take the wheel back while the car is stationary (parking maneuvers)
        if grip_released and wheel_at_rest and angle_ok and not standstill:
          self.resume_timer += 1
        else:
          self.resume_timer = 0

        if self.resume_timer >= self.resume_delay_frames * self.backoff_mult:
          self.paused = False
          self.resume_timer = 0
          self.frames_since_resume = 0

    if not self.paused:
      self.frames_since_resume += 1

    return lat_active and not self.paused
