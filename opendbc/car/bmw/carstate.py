from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.carlog import carlog
from opendbc.car.bmw.values import STEER_THRESHOLD

class CarState(CarStateBase):
  def __init__(self, CP: structs.CarParams):
    super().__init__(CP)

  @staticmethod
  def get_can_parsers(CP):

    # ---------- MAIN FLEXRAY GATEWAY (bus 5) ----------
    main_msgs = [
        ("vehicle_speed",   200),   # flexray → CAN gateway, ~200 Hz
        ("EPS_Angle",       200),
        ("NEW_MSG_38",      200),
        ("steer_torque",    200),
    ]

    cp_main = CANParser("bmw_sp2018_flexray", main_msgs, bus=5)

    # disable checksum/counter for all main messages
    for msg_name, _ in main_msgs:
        m = cp_main.dbc.name_to_msg[msg_name]
        m.ignore_checksum = True
        m.ignore_counter = True


    # ---------- SAS / ACC FRAME (bus 4) ----------
    sas_msgs = [
        ("ACC", 200),
    ]

    cp_sas = CANParser("bmw_sp2018_flexray", sas_msgs, bus=4)

    for msg_name, _ in sas_msgs:
        m = cp_sas.dbc.name_to_msg[msg_name]
        m.ignore_checksum = True
        m.ignore_counter = True


    # ---------- KCAN CHASSIS (bus 0) ----------
    kcan_msgs = [
        ("TurnSignals",    1),
        ("Acceleration",   50),
        ("Brake",          50),
        ("gear",           2),
        ("maybe_radar",    6),
        ("NEW_MSG_3F9",    3),
    ]

    cp_kcan = CANParser("bmw_sp2018_kcan", kcan_msgs, bus=0)

    return {
      Bus.main: cp_main,
      Bus.adas: cp_sas,
      Bus.chassis: cp_kcan,
    }

  def update(self, can_parsers) -> tuple[structs.CarState]:
    cp = can_parsers[Bus.main]
    cp_sas = can_parsers[Bus.adas]
    cp_kcan = can_parsers[Bus.chassis]

    ret = structs.CarState()

    # Previous state snapshot (avoids extra allocations and getattr fallback)
    prev = self.out

    # "gas" pedal
    ret.gasPressed = cp_kcan.vl["Acceleration"]["accelerator"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_kcan.vl["Brake"]["brake_pedal"] > 0

    ret.vEgoRaw = cp.vl["vehicle_speed"]["veh_speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgoRaw

    # steering is in both flexray (100 hz) and kcan (50 hz).
    # ret.steeringAngleDeg = cp_kcan.vl["steering_wheel"]["steering_wheel_angle_deg"]
    ret.steeringAngleDeg = cp.vl["EPS_Angle"]["steering_angle"]

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

    # not validated yet
    # Yaw rate (deg/s -> rad/s), demux cycle base 0
    # ret.yawRate = cp.vl["NEW_MSG_38"]["yaw"] * CV.DEG_TO_RAD

    # Driver steering torque (native units from CAN)
    ret.steeringTorque = cp.vl["steer_torque"]["driver_steer_torque"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # Blinkers
    ret.leftBlinker = cp_kcan.vl["TurnSignals"]["LeftTurn"] != 0
    ret.rightBlinker = cp_kcan.vl["TurnSignals"]["RightTurn"] != 0

    return ret


