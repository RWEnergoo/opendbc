"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class TeslaFlagsSP(IntFlag):
  HAS_VEHICLE_BUS = 1  # 3-finger infotainment press signal is present on the VEHICLE bus with the deprecated Tesla harness installed
  COOP_STEERING = 2  # Coop steering
  BUTTON_CANCELS = 4  # treat DI PRE_CANCEL (user cruise button press while engaged) as a cancel button event
  STEER_OVERRIDE_PAUSES = 8  # hard steering override pauses lateral only; longitudinal keeps running (stock TACC-like)
  # 2-bit index into STEER_OVERRIDE_RESUME_DELAYS, see steer_override_pause.py
  STEER_OVERRIDE_RESUME_DELAY_BIT0 = 16
  STEER_OVERRIDE_RESUME_DELAY_BIT1 = 32
  # 2-bit index into BUTTON_CANCEL_HOLD_DURATIONS, see carstate_ext.py
  BUTTON_CANCEL_HOLD_BIT0 = 64
  BUTTON_CANCEL_HOLD_BIT1 = 128


class TeslaSafetyFlagsSP:
  HAS_VEHICLE_BUS = 1
