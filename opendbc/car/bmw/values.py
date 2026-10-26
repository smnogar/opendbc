import numpy as np
from dataclasses import dataclass, field
from opendbc.car.docs_definitions import CarDocs

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL


@dataclass(frozen=True, kw_only=True)
class BMWCarSpecs(CarSpecs):
  mass: float
  wheelbase: float
  steerRatio: float
  centerToFrontRatio: float
  tireStiffnessFactor: float


@dataclass
class BMWSP2018PlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'bmw_sp2018_flexray'})

@dataclass
class BMWCarDocs(CarDocs):
  name: str
  package: str = "5AU"  # Default BMW ADAS package name

class CAR(Platforms):
  # --- BMW 5 Series (G30) ---
  BMW_G30 = BMWSP2018PlatformConfig(
    [BMWCarDocs(name="BMW 5 Series (G30)")],
    BMWCarSpecs(
      mass=2000.,
      wheelbase=3.105,
      steerRatio=16.3,
      centerToFrontRatio=0.5,
      tireStiffnessFactor=1.0,
    ),
  )

  # --- BMW i4 (G26) ---
  BMW_G26_I4 = BMWSP2018PlatformConfig(
    [BMWCarDocs(name="BMW i4 (G26)")],
    BMWCarSpecs(
      mass=2125.,
      wheelbase=2.856,
      steerRatio=15.0,
      centerToFrontRatio=0.54,
      tireStiffnessFactor=1.25,
    ),
  )


DBC = CAR.create_dbc_map()

# when to register steering torque as steering pressed
STEER_THRESHOLD = 1.0

# Lateral limits and controller parameters for BMW angle control
class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    360,  # deg

    # # UP: Steering INTO a curve (slightly smoothed to prevent jerk faults)
    # # 2.5 deg/frame @ 10ms = 250 deg/sec
    # ([0., 5., 25.], [2.5, 1.5, 0.3]),

    # # DOWN: Returning to CENTER (allowed to move faster)
    # ([0., 5., 25.], [3.5, 2.5, 0.5]),

    ([], []),
    ([], []),

    # Vehicle model-based limits (accounting for ~6% road bank)
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * 0.06),
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * 0.06),

    # per STEER_STEP, not per second: 2.5 deg per 10 ms = 250 deg/s
    MAX_ANGLE_RATE=2.5,
  )

  # Angle command is sent every frame (100 Hz). The FlexRay ACC slot only carries one
  # every 20 ms, but the pico takes the newest queued command per slot, so sending twice
  # as often halves its average age (~5 ms instead of ~10) and leaves a spare frame per
  # slot if one arrives late.
  STEER_STEP = 1
  # time constant of the low-pass on the steer rate field; was alpha = 0.5 per frame at 50 Hz
  STEER_RATE_FILTER_TC = 0.02  # s

  # reduce steering torque on highway for easier driver intervention. Not sure
  # how much of a difference this actually makes.
  WEAKEN_FORCE_BP = [22., 31.]
  WEAKEN_FORCE_V  = [250, 250]

  # --- EPS speed-dependent over-gain compensation (feedforward) ---
  # The EPS delivers more wheel angle than we request, and the excess grows with speed.
  # Achieved KCAN wheel angle / commanded wheel angle in steady curves:
  #   2026-07 drives (uncompensated): 1.06 @ 11 m/s, 1.12 @ 17, 1.24 @ 23, 1.27 @ 29, 1.27 @ 35
  #   2026-10 commutes, running [1.00, 1.04, 1.14, 1.20] @ [12, 18, 24, 30], still showed
  #   1.03-1.09 residual, i.e. static gain rather than resonance.
  # The trim loop below removes what is left of the error.
  EPS_GAIN_BP = [8., 12., 17., 23., 29., 35.]
  EPS_GAIN_V  = [1.00, 1.05, 1.09, 1.22, 1.27, 1.28]

  # --- Wheel angle feedback trim (residual gain + offsets) ---
  # A slow integral loop on the KCAN wheel angle, the same angle openpilot's vehicle model
  # and paramsd use. It removes whatever static error the EPS_GAIN feedforward leaves,
  # which drifts with season and tires: steady-curve gain was 1.27 at 29 m/s in July
  # and ~1.23 in October.
  # It used to close on the EPS motor angle instead, mapped to the wheel through a fixed
  # zero offset and load ratio. On the 2026-10-18/19 drives both of those drifted (offset
  # 2.1 deg at 3-8 m/s vs 1.5 on the highway; EPS/wheel 1.13 at 26-32 m/s vs the 1.075
  # assumed), so the loop tracked the EPS precisely while the wheel ended up ~4% short.
  # KCAN wheel response to the request: ~0.24 s dead time plus ~0.2 s first-order lag
  WHEEL_RESPONSE_DELAY = 0.24  # s
  WHEEL_RESPONSE_TC = 0.2      # s
  # Faster at low speed where the deadband is wide; 1.0 keeps >45 deg of phase margin
  # with the ~0.44 s total wheel lag
  TRIM_KI_BP = [5., 15.]     # m/s
  TRIM_KI_V = [1.0, 0.4]     # 1/s
  TRIM_MAX = 1.5             # deg, request units
  TRIM_ERR_DEADZONE = 0.05   # deg, avoids hunting inside the EPS deadband
  TRIM_MIN_SPEED = 3.        # m/s
  TRIM_MAX_TARGET_RATE = 15. # deg/s, freeze during fast transients
  TRIM_MAX_DRIVER_TORQUE = 0.6
  TRIM_ENGAGE_HOLDOFF = 1.0  # s after the EPS takes over

  # --- EPS angle, used to hold the EPS where it is when re-requesting after a handback ---
  # EPS_Angle.eps_angle_smooth is in steering_angle_req units. Its zero sits above the
  # request zero and drifts a little between drives, so it is learned near center.
  # Under rack load the EPS angle reads higher than the wheel (measured EPS / KCAN ratio).
  EPS_COMPLIANCE_BP = [6., 10., 20., 26., 30.]
  EPS_COMPLIANCE_V  = [0.99, 1.02, 1.03, 1.06, 1.08]
  EPS_ANGLE_OFFSET_INIT = 1.55  # deg, request units (27-36 counts measured across drives)
  EPS_ANGLE_OFFSET_TAU = 10.    # s
  EPS_ANGLE_OFFSET_MAX_WHEEL = 3.  # deg, only learn near center
  EPS_ANGLE_OFFSET_MAX_RATE = 5.   # deg/s, and while the wheel is nearly still

  # Deadband feedforward: push the request past the deadband on the side the target is
  # moving toward. The integrator alone cannot follow it, since the error flips sign on
  # every reversal. 0.6x the measured half-width; in simulation against the identified
  # EPS model it stayed beneficial with the real deadband anywhere from 0.5x to 1.5x.
  # Latching on the measured angle error instead was worse: it sees the deadband too late.
  DEADBAND_BP = [4.5, 7.5, 10.5, 13.5, 17.5, 22.5]       # m/s
  DEADBAND_V  = [0.69, 0.35, 0.23, 0.23, 0.11, 0.11]     # deg, request units
  DEADBAND_HYST = 0.05       # deg of wheel motion before the latched direction flips
  DEADBAND_SLEW = 2.         # deg/s

  # --- EPS handback recovery ---
  # target_steering_torque.eps_state
  EPS_STATE_ACTIVE = 18
  EPS_STATE_RAMP_OUT = 30
  EPS_STATE_DRIVER = 14
  # After a handback the EPS stays with the driver until it sees a fresh steering
  # request; in every logged fault it took over again 50-100 ms after one. Wait for the
  # ramp-out to finish and the driver to be hands-light, then ask again. This has to
  # complete inside selfdrived's 3 s soft-disable window to avoid a disengage.
  RECOVERY_MIN_FAULT_TIME = 1.0    # s, ramp-out takes ~0.9 s
  RECOVERY_QUIET_TORQUE = 0.5      # Nm
  RECOVERY_QUIET_TIME = 0.5        # s
  RECOVERY_ATTEMPT_TIME = 0.5      # s of re-requesting before giving up
  RECOVERY_MAX_ATTEMPTS = 2
  RECOVERY_ATTEMPT_WINDOW = 30.    # s
  RECOVERY_MIN_SPEED = 2.          # m/s
  RECOVERY_RELEASE_TIME = 0.2      # s of release frames between a ramp-out hold and the re-request
  # Keep requesting while the EPS is still ramping out (states 18/30) instead of releasing,
  # so it can resume without completing the ~1 s ramp-out. Only one logged ramp-out had
  # requests continue through it, and it resumed after 0.69 s; untested beyond that.
  RAMP_OUT_HOLD = False
  RAMP_OUT_HOLD_MAX_TIME = 2.0     # s, ramp-out takes ~1 s

  # --- Variable Sport Steering (VSS) Calibration ---
  VSS_ANGLE_DEG = np.array([0.0, 1.0, 5.0, 10.0, 30.0, 60.0, 90.0, 180.0, 360.0])
  VSS_ANGLE_RAD = VSS_ANGLE_DEG * np.pi / 180.0

  # slower response (hand crafted)
  VSS_RATIO = np.array([
    14.100,
    14.091,
    14.073,
    14.050,
    13.978,
    13.840,
    13.641,
    13.160,
    12.292,
  ])

  # actually measured
  # VSS_RATIO = np.array([
  #   15.464,  # Center
  #   15.45451,
  #   15.41675,
  #   15.37,
  #   15.188,
  #   14.93,
  #   14.69,
  #   14.07,
  #   13.34,
  # ])

  # VSS_RATIO = np.array([
  #   15.499,
  #   15.495,
  #   15.483,
  #   15.465,
  #   15.389,
  #   15.189,
  #   14.965,
  #   14.211,
  #   12.5,
  # ])