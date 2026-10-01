"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

# Pedal window of the SOFT_GAS_THRESHOLD feature, must match carstate.py and safety tesla.h
SOFT_GAS_MAX_PEDAL = 10.0  # %


class GasBrakeBlend:
  """Stock-TACC-like blending within the soft gas window: braking force scales down
  linearly with pedal position (5% pedal -> 50% of planned braking, 10% -> none).
  The transition back to full braking is inherently smooth because the scale follows
  the pedal as it returns to rest."""

  def __init__(self, CP_SP: structs.CarParamsSP):
    self.enabled = bool(CP_SP.flags & TeslaFlagsSP.SOFT_GAS_THRESHOLD)

  def update(self, accel: float, long_active: bool, accel_pedal_pos: float) -> float:
    if not self.enabled or not long_active or accel >= 0.0:
      return accel

    scale = 1.0 - min(max(accel_pedal_pos / SOFT_GAS_MAX_PEDAL, 0.0), 1.0)
    return accel * scale
