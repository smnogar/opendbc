from collections import deque

import numpy as np

from opendbc.can import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import CarControllerParams


class AngleTrim:
  """Deadband feedforward plus a slow integral trim on the wheel angle.

  openpilot's angle path is open loop, so the EPS deadband (eats small corrections at
  low speed) and any gain left over after EPS_GAIN go straight into curvature error.
  The deadband term pushes the request past the deadband on the side the target is
  moving toward. The trim compares where the wheel should be, given its measured dead
  time and lag, with the KCAN wheel angle, and integrates the difference into the
  request. Trim output is in steering_angle_req units. Runs at the transmit cadence.

  It also learns the EPS angle zero, used to hold the EPS where it is when
  re-requesting after a handback.
  """
  def __init__(self, P):
    self.P = P
    self.dt = DT_CTRL * P.STEER_STEP
    self.alpha = self.dt / (P.WHEEL_RESPONSE_TC + self.dt)
    self.delay = deque(maxlen=max(1, round(P.WHEEL_RESPONSE_DELAY / self.dt)))
    self.eps_offset = P.EPS_ANGLE_OFFSET_INIT
    self.reset()

  def reset(self):
    self.trim = 0.
    self.delay.clear()
    self.expected = None
    self.target_last = None
    self.active_time = 0.
    self.deadband_dir = 0.
    self.deadband_ref = None
    self.deadband_offset = 0.

  def eps_measured(self, eps_angle):
    return eps_angle - self.eps_offset

  def update_eps_offset(self, eps_angle, wheel_angle, wheel_rate, v_ego):
    # near center the EPS angle is the wheel angle plus a zero offset that drifts a
    # little between drives; learn it from any driving, engaged or not
    P = self.P
    if abs(wheel_angle) < P.EPS_ANGLE_OFFSET_MAX_WHEEL and abs(wheel_rate) < P.EPS_ANGLE_OFFSET_MAX_RATE and v_ego > P.TRIM_MIN_SPEED:
      compliance = float(np.interp(v_ego, P.EPS_COMPLIANCE_BP, P.EPS_COMPLIANCE_V))
      err = eps_angle - wheel_angle * compliance - self.eps_offset
      self.eps_offset += err * self.dt / P.EPS_ANGLE_OFFSET_TAU

  def _update_deadband(self, wheel_target, v_ego):
    # latch the direction of travel; the offset has to persist through steady curves,
    # so it is not scaled by rate
    P = self.P
    if self.deadband_ref is None:
      self.deadband_ref = wheel_target
    if wheel_target > self.deadband_ref + P.DEADBAND_HYST:
      self.deadband_dir, self.deadband_ref = 1., wheel_target
    elif wheel_target < self.deadband_ref - P.DEADBAND_HYST:
      self.deadband_dir, self.deadband_ref = -1., wheel_target
    goal = self.deadband_dir * float(np.interp(v_ego, P.DEADBAND_BP, P.DEADBAND_V))
    step = P.DEADBAND_SLEW * self.dt
    self.deadband_offset += float(np.clip(goal - self.deadband_offset, -step, step))

  def update(self, wheel_target, wheel_angle, v_ego, driver_torque, steering_pressed, eps_active):
    P = self.P

    self._update_deadband(wheel_target, v_ego)

    # model of the wheel response to the target: dead time then first-order lag
    self.delay.append(wheel_target)
    if self.expected is None:
      self.expected = wheel_target
    self.expected += self.alpha * (self.delay[0] - self.expected)

    target_rate = 0. if self.target_last is None else (wheel_target - self.target_last) / self.dt
    self.target_last = wheel_target

    if eps_active:
      self.active_time += self.dt

    freeze = (not eps_active or
              self.active_time < P.TRIM_ENGAGE_HOLDOFF or
              v_ego < P.TRIM_MIN_SPEED or
              steering_pressed or
              abs(driver_torque) > P.TRIM_MAX_DRIVER_TORQUE or
              abs(target_rate) > P.TRIM_MAX_TARGET_RATE)

    if not freeze:
      err = self.expected - wheel_angle
      if abs(err) > P.TRIM_ERR_DEADZONE:
        ki = float(np.interp(v_ego, P.TRIM_KI_BP, P.TRIM_KI_V))
        self.trim = float(np.clip(self.trim + ki * err * self.dt, -P.TRIM_MAX, P.TRIM_MAX))

    return self.trim + self.deadband_offset


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    # External panda is index 1 -> buses 4-7; use bus 4 for TX
    self.packer = CANPacker(dbc_names[Bus.main])
    self.cnt = 0
    self.cycle = 0
    self.apply_angle_last = 0.0
    self.frame = 0
    self.filtered_rate = 0.

    # Vehicle model for angle limiting (use BMW CP)
    self.VM = VehicleModel(CP)
    self.params = CarControllerParams()
    self.angle_trim = AngleTrim(self.params)

    # EPS handback recovery
    self.fault_frames = 0
    self.quiet_frames = 0
    self.recovery_frames = 0
    self.recovery_attempts = deque()
    self.hold_frames = 0
    self.release_frames = 0

  def _acc_cnt(self):
    self.cnt = (self.cnt + 1) % 16
    return self.cnt

  def _acc_cycle(self):
    self.cycle = (self.cycle + 1) % 4
    return self.cycle

  def _update_recovery(self, CC, CS):
    """Returns True while we should keep or re-request steering after an EPS handback.

    When the EPS hands back it stays with the driver until it sees a fresh request.
    steerFaultTemporary keeps latActive off, so without this we only ever send release
    frames and the fault never clears until cruise is toggled.

    Two phases, both leaving the fault and its alert untouched:
      1. ramp-out hold: while the EPS is still handing back (states 18/30) and the driver
         is hands-light, keep requesting at the current EPS angle so it can resume
         without completing the ~1 s ramp-out
      2. re-request: once it is with the driver (state 14), send a short release and then
         a fresh request, the sequence that recovered every logged fault
    """
    P = self.params
    fault = CC.enabled and CS.out.steerFaultTemporary

    if abs(CS.out.steeringTorque) < P.RECOVERY_QUIET_TORQUE and not CS.out.steeringPressed:
      self.quiet_frames += 1
    else:
      self.quiet_frames = 0
    quiet = self.quiet_frames * DT_CTRL >= P.RECOVERY_QUIET_TIME

    self.fault_frames = self.fault_frames + 1 if fault else 0

    # done once openpilot steers again, or nothing to recover
    if not CC.enabled or CC.latActive:
      self.recovery_frames = 0
      self.hold_frames = 0
      self.release_frames = 0
      return False

    # 1. ramp-out hold
    if self.hold_frames > 0 or (fault and self.recovery_frames == 0 and self.release_frames == 0 and P.RAMP_OUT_HOLD):
      handing_back = CS.eps_state in (P.EPS_STATE_ACTIVE, P.EPS_STATE_RAMP_OUT)
      if not fault and self.hold_frames > 0:
        # EPS resumed and the fault cleared: keep requesting until openpilot steers again
        self.hold_frames = 0
        self.recovery_frames = 1
        return True
      if fault and handing_back and quiet and CS.out.vEgo > P.RECOVERY_MIN_SPEED and \
         self.hold_frames * DT_CTRL < P.RAMP_OUT_HOLD_MAX_TIME:
        self.hold_frames += 1
        return True
      if self.hold_frames > 0:
        # EPS finished handing back, or the driver took over: release before asking again
        self.hold_frames = 0
        self.release_frames = round(P.RECOVERY_RELEASE_TIME / DT_CTRL)

    if self.release_frames > 0:
      self.release_frames -= 1
      return False

    # 2. re-request
    if self.recovery_frames > 0:
      self.recovery_frames += 1
      elapsed = self.recovery_frames * DT_CTRL
      # keep asking until latActive returns; give up if the EPS does not take over,
      # or if the fault cleared but openpilot did not resume steering
      if (fault and elapsed > P.RECOVERY_ATTEMPT_TIME) or elapsed > P.RECOVERY_ATTEMPT_TIME + 1.0:
        self.recovery_frames = 0
        self.fault_frames = 0  # wait a full ramp-out again before any further attempt
        return False
      return True

    if not fault:
      return False

    while self.recovery_attempts and (self.frame - self.recovery_attempts[0]) * DT_CTRL > P.RECOVERY_ATTEMPT_WINDOW:
      self.recovery_attempts.popleft()

    can_retry = (self.fault_frames * DT_CTRL >= P.RECOVERY_MIN_FAULT_TIME and
                 CS.eps_state == P.EPS_STATE_DRIVER and
                 quiet and
                 CS.out.vEgo > P.RECOVERY_MIN_SPEED and
                 len(self.recovery_attempts) < P.RECOVERY_MAX_ATTEMPTS)
    if can_retry:
      self.recovery_attempts.append(self.frame)
      self.recovery_frames = 1
      return True
    return False

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []
    P = self.params

    transmit = self.frame % P.STEER_STEP == 0
    lat_active = bool(CC.latActive)
    desired_wheel_angle = float(actuators.steeringAngleDeg)

    recovering = self._update_recovery(CC, CS)

    if transmit:
      self.angle_trim.update_eps_offset(CS.eps_angle, CS.out.steeringAngleDeg, CS.out.steeringRateDeg, CS.out.vEgo)

    if not lat_active:
      # start the next engagement from where the wheel is, whatever the frame parity
      self.apply_angle_last = float(CS.out.steeringAngleDeg)
      self.filtered_rate = 0.
      self.angle_trim.reset()

    # Rate limiting runs at the transmit cadence, NOT every frame.
    # get_max_angle_delta_vm() returns a delta budget for one STEER_STEP
    # (it multiplies by DT_CTRL * STEER_STEP). Spending that budget on every
    # 100 Hz frame while only transmitting every STEER_STEP frames allowed
    # STEER_STEP times the intended lateral jerk, which let the commanded angle
    # slew faster than the EPS could follow. The resulting tracking error made
    # the EPS hand control back to the driver (driver_steering_active -> 254),
    # which surfaces as a spurious "Steering Assist Temporarily Unavailable".
    if transmit and lat_active:
      desired_wheel_angle = apply_steer_angle_limits_vm(
          desired_wheel_angle,
          self.apply_angle_last,
          CS.out.vEgoRaw,
          CS.out.steeringAngleDeg,
          lat_active,
          CarControllerParams,
          self.VM
      )

      # delta now spans DT_CTRL * STEER_STEP, so convert to deg/s with that period
      # to keep this feedforward at the same physical scale as before
      angle_rate = (desired_wheel_angle - self.apply_angle_last) / (DT_CTRL * P.STEER_STEP) * 0.45

      dt = DT_CTRL * P.STEER_STEP
      alpha = dt / (P.STEER_RATE_FILTER_TC + dt)
      self.filtered_rate = (alpha * angle_rate) + ((1.0 - alpha) * self.filtered_rate)

      self.apply_angle_last = desired_wheel_angle
    else:
      # hold between transmits so carOutput reports what is actually commanded
      desired_wheel_angle = self.apply_angle_last

    send_empty_frame = False
    if recovering:
      pass
    elif CC.enabled and not lat_active and CS.out.steerFaultTemporary:
      # EPS has handed back: release until recovery decides to ask again
      send_empty_frame = True
    elif CC.enabled and not lat_active and not CS.out.steerFaultTemporary and CS.out.vEgo > 2.:
      # send frames without a steering request
      pass
    elif not lat_active:
      # otherwise send nothing (typically for OP disabled)
      self.frame += 1
      return actuators, []

    if transmit:
      if recovering:
        # request that holds the EPS where it is now, so there is no torque step
        eps_angle_req = self.angle_trim.eps_measured(CS.eps_angle)
        steer_req_rate = 0.
      else:
        # TRANSLATE TO THE EPS DOMAIN FOR FLEXRAY
        # account for variable steering ratio
        current_ratio = float(np.interp(abs(desired_wheel_angle), P.VSS_ANGLE_DEG, P.VSS_RATIO))
        current_pinion_scale = current_ratio / P.VSS_RATIO[0]   # < 1
        eps_target = desired_wheel_angle * current_pinion_scale

        # CANCEL THE EPS SPEED-DEPENDENT OVER-GAIN (feedforward)
        # The EPS applies more angle than requested, increasingly so with speed. openpilot's
        # lateral loop assumes a unity-gain actuator, so divide the excess back out to keep
        # the forward path at ~1.0. See EPS_GAIN_BP/V in values.py for the measurements.
        eps_gain = float(np.interp(CS.out.vEgo, P.EPS_GAIN_BP, P.EPS_GAIN_V))
        eps_angle_req = eps_target / eps_gain

        # DEADBAND FEEDFORWARD + WHEEL ANGLE FEEDBACK TRIM
        if lat_active:
          eps_angle_req += self.angle_trim.update(desired_wheel_angle, CS.out.steeringAngleDeg, CS.out.vEgo,
                                                  CS.out.steeringTorque, CS.out.steeringPressed,
                                                  CS.eps_state == P.EPS_STATE_ACTIVE)
        steer_req_rate = self.filtered_rate

      steer_req = lat_active or recovering
      force_weaken_red = np.interp(CS.out.vEgo, P.WEAKEN_FORCE_BP, P.WEAKEN_FORCE_V)

      if send_empty_frame:
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
      else:
        values = {
          "cycle_count": 1,
          "crc1": 0,
          "cnt1": 0,
          "always_0x9": 9,
          "steering_angle_req": eps_angle_req,
          "maybe_steer_req_rate": steer_req_rate,
          "TJA_ready": 0,
          "assist_mode": 1 if steer_req else 0,
          "wayback_en1_lane_keeping_trigger": 0,
          "lane_keeping_triggered": 0,
          "like_assist_torque_reserve": 0xA0 if steer_req else 0x00,
          "constants": 0x03ff17fe,
          "wayback_en_2": 0,
          "steering_engaged": 2 if steer_req else 1,
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
