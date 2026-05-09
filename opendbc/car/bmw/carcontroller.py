import numpy as np

from opendbc.can import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    # External panda is index 1 -> buses 4-7; use bus 4 for TX
    self.packer = CANPacker(dbc_names[Bus.main])
    self.cnt = 0
    self.cycle = 0
    self.apply_angle_last = 0.0
    self.frame = 0 
    self.send_empty_frame = False
    self.current_fault_sent_empty_frame = False
    self.num_steering_fault_frames = 0
    self.filtered_rate = 0.
    
    # Vehicle model for angle limiting (use BMW CP)
    self.VM = VehicleModel(CP)
    self.params = CarControllerParams()

  def _acc_cnt(self):
    self.cnt = (self.cnt + 1) % 16
    return self.cnt

  def _acc_cycle(self):
    self.cycle = (self.cycle + 1) % 4
    return self.cycle

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # reset sent empty frame tracker
    if not CS.out.steerFaultTemporary or not CC.enabled:
      self.current_fault_sent_empty_frame = False
      self.num_steering_fault_frames = 0

    lat_active = bool(CC.latActive)
    lat_desired = lat_active
    desired_wheel_angle = float(actuators.steeringAngleDeg)

    desired_wheel_angle = apply_steer_angle_limits_vm(
        desired_wheel_angle, 
        self.apply_angle_last, 
        CS.out.vEgoRaw, 
        CS.out.steeringAngleDeg,
        lat_active, 
        CarControllerParams, 
        self.VM
    )

    # scale by 0.8
    angle_rate = (desired_wheel_angle - self.apply_angle_last) * 100. * 0.45

    alpha = 0.5
    self.filtered_rate = (alpha * angle_rate) + ((1.0 - alpha) * self.filtered_rate)

    self.apply_angle_last = desired_wheel_angle

    self.send_empty_frame = False
    if CC.enabled and not CC.latActive and CS.out.steerFaultTemporary:

      # lat_desired = True

      # check to see if there is a steering fault. If so, send a single frame
      # with no steering command
      # if not self.current_fault_sent_empty_frame:
      self.send_empty_frame = True

    elif CC.enabled and not CC.latActive and not CS.out.steerFaultTemporary and CS.out.vEgo > 2.:

      # if the steering fault has cleared resume sending frames
      self.send_empty_frame = False

    elif not CC.latActive:

      # otherwise send empty frame (typically for OP disabled)
      return actuators, []

    # 2. TRANSLATE TO PINION DOMAIN FOR FLEXRAY
    current_ratio = float(np.interp(
        abs(desired_wheel_angle), 
        self.params.VSS_ANGLE_DEG,
        self.params.VSS_RATIO
    ))

    # experimental: Account for variable steering ratio
    # center_ratio = self.params.VSS_RATIO[0]
    # current_pinion_scale = current_ratio / center_ratio
    # eps_angle_req = desired_wheel_angle * current_pinion_scale
    eps_angle_req = desired_wheel_angle

    if self.frame % 2 == 0:
      force_weaken_red = np.interp(CS.out.vEgo, self.params.WEAKEN_FORCE_BP, self.params.WEAKEN_FORCE_V)

      # if self.send_empty_frame and self.num_steering_fault_frames < 5:
      if self.send_empty_frame:
        values = {
          "cycle_count": 1,
          "crc1": 0,
          "cnt1": 0,
          "always_0x9": 9,
          "steering_angle_req": 0.0,
          "maybe_steer_req_rate": 0.0,
          "TJA_ready": 0,
          "assist_mode": 0,
          "wayback_en1_lane_keeping_trigger": 0,
          "lane_keeping_triggered": 0,
          "like_assist_torque_reserve": 0x00,
          "constants": 0x03ff17fe,
          "wayback_en_2": 0,
          "steering_engaged": 1,
          "maybe_assist_force_enhance": 0,
          "maybe_assist_force_weaken": 0,
        }
        # self.send_empty_frame = False
        self.num_steering_fault_frames += 1
        # self.current_fault_sent_empty_frame = True
      else:
        values = {
          "cycle_count": 1,
          "crc1": 0,
          "cnt1": 0,
          "always_0x9": 9,
          "steering_angle_req": eps_angle_req,
          "maybe_steer_req_rate": self.filtered_rate,
          "TJA_ready": 0,
          "assist_mode": 1 if lat_desired else 0,
          "wayback_en1_lane_keeping_trigger": 0,
          "lane_keeping_triggered": 0,
          "like_assist_torque_reserve": 0xA0 if lat_desired else 0x00,
          "constants": 0x03ff17fe,
          "wayback_en_2": 0,
          "steering_engaged": 2 if lat_desired else 1,
          "maybe_assist_force_enhance": 0xA2,
          "maybe_assist_force_weaken": force_weaken_red,
        }

      frame_id = self.packer.dbc.name_to_msg.get("ACC").address
      values["cycle_count"] = 1 
      values["crc1"] = frame_id & 0xFF
      values["cnt1"] = (frame_id >> 8) & 0b111

      msg = self.packer.make_can_msg("ACC", 5, values)
      can_sends.append(msg)

    self.frame += 1
    new_actuators = actuators.as_builder()
    
    new_actuators.steeringAngleDeg = desired_wheel_angle    
    return new_actuators, can_sends