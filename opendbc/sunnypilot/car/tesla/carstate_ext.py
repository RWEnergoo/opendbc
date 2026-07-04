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

# Cancel trigger behavior is hardware-determined, no user setting (findings from
# labeled protocol test route 00000009--399b5802f2):
# - With the VEHICLE bus, VCLEFT_switchStatus carries clean per-wheel signals
#   (swcRightPressed / swcLeftPressed / per-wheel scroll ticks), so the cancel
#   trigger uses ONLY the right wheel press and a plain click cancels instantly:
#   scroll ticks, volume clicks and the left-wheel chill-mode hold can never cancel.
# - Without the vehicle bus, the only wheel signal is UI_warning.scrollWheelPressed,
#   which fires on BOTH wheels for clicks (100-200ms pulses), every scroll tick
#   (~100ms pulses) and holds. An instant trigger is unsafe there (every scroll tick
#   would cancel), so a 0.5s continuous hold is required: inherently scroll- and
#   short-click-proof; only a long left-wheel hold remains indistinguishable.
INSTANT_HOLD_FRAMES = 1
FALLBACK_HOLD_FRAMES = 50  # 0.5s at 100Hz
REARM_RELEASE_FRAMES = 20  # 200ms debounce: the cancel press must be fully released before a new press re-arms


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.infotainment_3_finger_press = 0
    self.pre_cancel_prev = False
    self.scroll_pressed_frames = 0
    self.cruise_enabled_frames = 0
    self.cancel_sent = False
    self.rearm_state = 0  # 0 = re-engagement allowed, 1 = awaiting full release, 2 = awaiting fresh press
    self.button_cancel_rearm = False  # carcontroller sends a standing silent cancel while set
    self.released_frames = 0
    self.press_started_engaged = False

    self.cancel_hold_frames = INSTANT_HOLD_FRAMES if CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS else FALLBACK_HOLD_FRAMES
    self.right_pressed = False

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    if self.CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
      cp_adas = can_parsers[Bus.adas]

      prev_infotainment_3_finger_press = self.infotainment_3_finger_press
      self.infotainment_3_finger_press = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])

      ret.buttonEvents = [*create_button_events(self.infotainment_3_finger_press, prev_infotainment_3_finger_press,
                                                {3: ButtonType.lkas})]

      # VCLEFT_switchStatus is multiplexed and the parser ignores the mux, so only
      # read the per-wheel switch signals from index-1 frames
      if int(cp_adas.vl["VCLEFT_switchStatus"]["VCLEFT_switchStatusIndex"]) == 1:
        self.right_pressed = cp_adas.vl["VCLEFT_switchStatus"]["VCLEFT_swcRightPressed"] == 2  # SWITCH_ON

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

      # The wheel click is normally handled by the AP computer, which openpilot replaces, so the
      # press goes nowhere and no PRE_CANCEL appears. Read it from the bus ourselves: preferably
      # the right-wheel-specific press on the vehicle bus, otherwise the shared UI_warning bit
      # (both wheels + scroll ticks; see BUTTON_CANCEL_HOLD_DURATIONS notes).
      if self.CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
        scroll_wheel_pressed = self.right_pressed
      else:
        scroll_wheel_pressed = cp_party.vl["UI_warning"]["scrollWheelPressed"] == 1
      self.cruise_enabled_frames = self.cruise_enabled_frames + 1 if ret.cruiseState.enabled else 0
      self.scroll_pressed_frames = self.scroll_pressed_frames + 1 if scroll_wheel_pressed else 0
      if not scroll_wheel_pressed:
        self.cancel_sent = False
      elif self.scroll_pressed_frames == 1:
        # only a press that STARTED while engaged may cancel, so holding the
        # engaging click itself never bounces back off (matters at short hold settings)
        self.press_started_engaged = self.cruise_enabled_frames > 50

      if self.scroll_pressed_frames >= self.cancel_hold_frames and not self.cancel_sent and self.press_started_engaged:
        cancel = True
        self.cancel_sent = True

      # The car itself can treat the tail of the cancel click as an engage command, which would
      # bounce everything straight back on. Instead of a timed window, block PCM re-engagement
      # causally with an explicit state machine: after a cancel, first the cancel press must be
      # fully released (debounced), and only a fresh press after that re-allows engagement.
      # NOTE: the release counter must be reset by the press BEFORE any rearm decision, otherwise
      # the cancel press itself instantly rearms off its own stale pre-press count (the Instant
      # re-engage bug found on the road, 2026-07-04).
      if cancel:
        self.rearm_state = 1
      if scroll_wheel_pressed:
        if self.rearm_state == 2:
          self.rearm_state = 0  # fresh press after full release: the driver wants to re-engage
        self.released_frames = 0
      else:
        self.released_frames += 1
        if self.rearm_state == 1 and self.released_frames >= REARM_RELEASE_FRAMES:
          self.rearm_state = 2
      if self.rearm_state != 0:
        ret.blockPcmEnable = True
      # While rearming, the carcontroller sends a standing ACC_CANCEL_GENERIC_SILENT instead of
      # ACC_ON, so the DI never completes the click-tail engage - no engage/disengage chime
      self.button_cancel_rearm = self.rearm_state != 0

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
