from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.carlog import carlog

class CarState(CarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    # Use CarStateBase.out / out_sp as rolling previous-state buffers
  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # External panda is index 1 -> buses 4-7. Use bus 4 for main traffic.
    cp_main = CANParser("bmw_sp2018_flexray", [("vehicle_speed", float("nan")), ("EPS_Angle", float("nan")), ("NEW_MSG_38", float("nan")), ("steer_torque", float("nan"))], bus=5)
    # One-time DBC config; avoid doing this in the control loop
    cp_main.dbc.name_to_msg["vehicle_speed"].ignore_checksum = True
    cp_main.dbc.name_to_msg["EPS_Angle"].ignore_checksum = True
    cp_main.dbc.name_to_msg["vehicle_speed"].ignore_counter = True
    cp_main.dbc.name_to_msg["EPS_Angle"].ignore_counter = True
    # Yaw and driver torque parsing; ignore checks for now
    cp_main.dbc.name_to_msg["NEW_MSG_38"].ignore_checksum = True
    cp_main.dbc.name_to_msg["NEW_MSG_38"].ignore_counter = True
    cp_main.dbc.name_to_msg["steer_torque"].ignore_checksum = True
    cp_main.dbc.name_to_msg["steer_torque"].ignore_counter = True

    cp_sas = CANParser("bmw_sp2018_flexray", [("ACC", float("nan"))], bus=4)
    # ACC RX is currently synthetic; ignore checks until real CRC/counter implemented
    cp_sas.dbc.name_to_msg["ACC"].ignore_checksum = True
    cp_sas.dbc.name_to_msg["ACC"].ignore_counter = True

    kcan_messages = [
      ("TurnSignals", float("nan")),
      ("Acceleration", float("nan")),
      ("Brake", float("nan")),
      ("gear", float("nan")),
      ('maybe_radar', float('nan')),
      ('NEW_MSG_3F9', float('nan')),
    ]

    cp_kcan = CANParser("bmw_sp2018_kcan",kcan_messages, bus=0)

    return {
      Bus.main: cp_main,
      Bus.adas: cp_sas,
      Bus.chassis: cp_kcan,
    }

  def _demux_last(self, cp: CANParser, msg: str, cc_sig: str, val_sig: str, cycle_base: int) -> tuple[bool, float]:
    cc_list = cp.vl_all[msg].get(cc_sig, [])
    val_list = cp.vl_all[msg].get(val_sig, [])
    for i in range(len(cc_list) - 1, -1, -1):
      if int(cc_list[i]) == cycle_base:
        return True, float(val_list[i])
    return False, 0.0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.main]
    cp_sas = can_parsers[Bus.adas]
    cp_kcan = can_parsers[Bus.chassis]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Previous state snapshot (avoids extra allocations and getattr fallback)
    prev = self.out

    # fl = cp.vl["wheel_speed"].get("FL", 0.0)
    # fr = cp.vl["wheel_speed"].get("FR", 0.0)
    # rl = cp.vl["wheel_speed"].get("RL", 0.0)
    # rr = cp.vl["wheel_speed"].get("RR", 0.0)
    # self.parse_wheel_speeds(ret, fl, fr, rl, rr, CV.KPH_TO_MS)

    # "gas" pedal
    ret.gasPressed = cp_kcan.vl["Acceleration"]["accelerator"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_kcan.vl["Brake"]["brake_pedal"] > 0

    # Batch demux using helper; BMW DBC uses fixed cycle codes
    veh_found, veh_speed_kph = self._demux_last(cp, "vehicle_speed", "cycle_count", "veh_speed", cycle_base=3)

    if veh_found:
      ret.vEgoRaw = veh_speed_kph * CV.KPH_TO_MS
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
      ret.vEgoCluster = ret.vEgoRaw
    else:
      ret.vEgoRaw = prev.vEgoRaw
      ret.vEgo = prev.vEgo
      ret.aEgo = prev.aEgo
      ret.vEgoCluster = float(prev.vEgoCluster)

    # Steering angle: choose cycle_count == 0 if present
    eps_found, eps_angle = self._demux_last(cp, "EPS_Angle", "cycle_count", "steering_angle", cycle_base=0)
    if eps_found:
      ret.steeringAngleDeg = float(eps_angle)
    else:
      ret.steeringAngleDeg = float(prev.steeringAngleDeg)

    ret.standstill = ret.vEgoRaw < 0.01

    gear_val = cp_kcan.vl["NEW_MSG_3F9"]["maybe_gear"]
    gear_int = int(gear_val)
    if gear_int == 5:
      ret.gearShifter = structs.CarState.GearShifter.drive
    elif gear_int == 2:
      ret.gearShifter = structs.CarState.GearShifter.reverse
    elif gear_int == 3:
      ret.gearShifter = structs.CarState.GearShifter.park
    else:
      # Unknown mapping, keep previous
      ret.gearShifter = prev.gearShifter

    # use radar message which is 1 regardless if pedal is pushed
    long_active = bool(int(cp_kcan.vl["maybe_radar"]["long_active"]))

    ret.cruiseState.enabled = long_active

    ret.cruiseState.available = True

    # Yaw rate (deg/s -> rad/s), demux cycle base 0
    yaw_found, yaw_deg_s = self._demux_last(cp, "NEW_MSG_38", "cycle_count", "yaw", cycle_base=0)
    if yaw_found:
      ret.yawRate = float(yaw_deg_s) * CV.DEG_TO_RAD
    else:
      ret.yawRate = float(prev.yawRate)

    # Driver steering torque (native units from CAN)
    steering_torque_found, steering_torque = self._demux_last(cp, "steer_torque", "cycle_count", "driver_steer_torque", cycle_base=0)
    if steering_torque_found:
      ret.steeringTorque = float(steering_torque)
    else:
      ret.steeringTorque = float(prev.steeringTorque)

    # Blinkers
    ret.leftBlinker = cp_kcan.vl["TurnSignals"]["LeftTurn"] != 0
    ret.rightBlinker = cp_kcan.vl["TurnSignals"]["RightTurn"] != 0

    return ret, ret_sp


