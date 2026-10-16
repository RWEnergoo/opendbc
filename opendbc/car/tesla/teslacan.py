from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams, TeslaFlags


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  # Returns the raw 3-bit DAS_steeringControlType (byte 2 bits 7:5) for a control type
  # (0 NONE, 1 ANGLE_CONTROL, 2 LANE_KEEP_ASSIST, 3 EMERGENCY_LANE_KEEP, 4 FSD).
  # FSD 14 firmware uses all 3 bits. Legacy firmware only uses the top 2 bits, so its values sit one bit higher
  if flags & TeslaFlags.FSD_14:
    return ctrl_type
  else:
    return ctrl_type << 1


class TeslaCAN:
  def __init__(self, CP, packer):
    self.CP = CP
    self.packer = packer

  def create_steering_control(self, angle, enabled, control_type):
    # control_type is ANGLE_CONTROL (1), or LANE_KEEP_ASSIST (2) with coop steering.
    # FSD 14 firmware (EPS (92),E4H015.09.2 and newer) ignores FSD (4), which the old 2-bit encoding produced
    # for ANGLE_CONTROL on these cars, so the value is now written at its true 3-bit width like upstream
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": get_steer_ctrl_type(self.CP.flags, control_type if enabled else 0),
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    set_speed = min(max(v_ego + accel, 0) * CV.MS_TO_KPH, 400)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF
