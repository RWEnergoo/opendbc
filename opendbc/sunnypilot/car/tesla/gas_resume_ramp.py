"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

# Panda safety forbids any longitudinal actuation while the gas pedal is pressed, so
# openpilot commands 0 during a gas override. Stock Tesla blends light pedal input with
# soft braking; the closest safe equivalent is ramping braking back in gently after the
# pedal is released instead of snapping to the planner's full deceleration.
GAS_RESUME_JERK = 2.0  # m/s^3, max increase in braking after gas release
DT_LONG = 0.04  # longitudinal command interval (every 4th 100Hz frame)


class GasResumeRamp:
  def __init__(self, CP_SP: structs.CarParamsSP):
    self.enabled = bool(CP_SP.flags & TeslaFlagsSP.SOFT_BRAKE_AFTER_GAS)
    self.ramp_accel = None

  def update(self, accel: float, long_active: bool, gas_pressed: bool) -> float:
    if not self.enabled or not long_active:
      self.ramp_accel = None
      return accel

    if gas_pressed:
      self.ramp_accel = 0.0
      return accel

    if self.ramp_accel is not None:
      self.ramp_accel -= GAS_RESUME_JERK * DT_LONG
      if accel >= self.ramp_accel:
        # planner wants less braking than the ramp allows: ramp finished
        self.ramp_accel = None
      else:
        accel = self.ramp_accel

    return accel
