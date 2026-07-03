"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import DBC, CANBUS
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.infotainment_3_finger_press = 0
    self.pre_cancel_prev = False
    self.scroll_wheel_pressed_prev = False
    self.cruise_enabled_frames = 0

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    if self.CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
      cp_adas = can_parsers[Bus.adas]

      prev_infotainment_3_finger_press = self.infotainment_3_finger_press
      self.infotainment_3_finger_press = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])

      ret.buttonEvents = [*create_button_events(self.infotainment_3_finger_press, prev_infotainment_3_finger_press,
                                                {3: ButtonType.lkas})]

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]

    if self.CP_SP.flags & TeslaFlagsSP.BUTTON_CANCELS:
      cancel = False

      # The DI briefly reports PRE_CANCEL when a cancel request reaches it while engaged.
      # Stock openpilot treats PRE_CANCEL as engaged and keeps commanding ACC_ON, swallowing
      # the user's cancel. Surface the rising edge as a cancel button so the press disengages.
      cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
      pre_cancel = cruise_state == "PRE_CANCEL"
      if pre_cancel and not self.pre_cancel_prev:
        cancel = True
      self.pre_cancel_prev = pre_cancel

      # The scroll wheel click is normally handled by the AP computer, which openpilot replaces,
      # so the press goes nowhere and no PRE_CANCEL appears. Read the click directly from
      # UI_warning and treat it as a cancel while engaged. The half-second guard keeps the
      # engaging click itself (button press -> DI engages) from immediately canceling.
      scroll_wheel_pressed = cp_party.vl["UI_warning"]["scrollWheelPressed"] == 1
      self.cruise_enabled_frames = self.cruise_enabled_frames + 1 if ret.cruiseState.enabled else 0
      if scroll_wheel_pressed and not self.scroll_wheel_pressed_prev and self.cruise_enabled_frames > 50:
        cancel = True
      self.scroll_wheel_pressed_prev = scroll_wheel_pressed

      if cancel:
        ret.buttonEvents = [*ret.buttonEvents, structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel)]

    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)
    speed_limit = cp_ap_party.vl["DAS_status"]["DAS_fusedSpeedLimit"]
    if self.can_define.dv["DAS_status"]["DAS_fusedSpeedLimit"].get(int(speed_limit), None) in ["NONE", "UNKNOWN_SNA"]:
      ret_sp.speedLimit = 0
    else:
      if speed_units == "KPH":
        ret_sp.speedLimit = speed_limit * CV.KPH_TO_MS
      elif speed_units == "MPH":
        ret_sp.speedLimit = speed_limit * CV.MPH_TO_MS

  @staticmethod
  def get_parser(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> dict[StrEnum, CANParser]:
    messages = {}

    if CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
      messages[Bus.adas] = CANParser(DBC[CP.carFingerprint][Bus.adas], [], CANBUS.vehicle)

    return messages
