"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class TeslaFlagsSP(IntFlag):
  HAS_VEHICLE_BUS = 1  # Multi-finger infotainment press signal is present on the VEHICLE bus with the deprecated Tesla harness installed
  COOP_STEERING = 2  # Coop steering
  MADS_SCREEN_BUTTON_3_FINGER = 4
  MADS_SCREEN_BUTTON_4_FINGER = 8
  MADS_SCREEN_BUTTON_5_FINGER = 16
  # fork flags start at 32: upstream owns bits 1-16 above
  BUTTON_CANCELS = 32  # treat a right-wheel press / DI PRE_CANCEL while engaged as a cancel button event
  STEER_OVERRIDE_PAUSES = 64  # steering override pauses lateral only; longitudinal keeps running (stock TACC-like)
  # 2-bit index into STEER_OVERRIDE_RESUME_DELAYS, see steer_override_pause.py
  STEER_OVERRIDE_RESUME_DELAY_BIT0 = 128
  STEER_OVERRIDE_RESUME_DELAY_BIT1 = 256
  SOFT_GAS_THRESHOLD = 512  # light accelerator (<= 10%) does not count as gas pressed; braking blends with pedal
  SIM_VEHICLE_BUS_LOSS = 1024  # dev/test: starve the button trigger's vehicle bus reads to exercise the failover path
  GAP_ADJUST_TILT = 2048  # right wheel tilt = following distance / experimental mode holds


class MadsScreenButtonType:
  OFF = 0
  THREE_FINGER = 1
  FOUR_FINGER = 2
  FIVE_FINGER = 3


class TeslaSafetyFlagsSP:
  HAS_VEHICLE_BUS = 1
  MADS_SCREEN_BUTTON_3_FINGER = 2
  MADS_SCREEN_BUTTON_4_FINGER = 4
  MADS_SCREEN_BUTTON_5_FINGER = 8
  SOFT_GAS_THRESHOLD = 16  # moved off bit 2: upstream took it for the MADS screen button
