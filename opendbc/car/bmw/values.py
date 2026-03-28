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
      tireStiffnessFactor=1.35,
    ),
  )


DBC = CAR.create_dbc_map()

# when to register steering torque as steering pressed
STEER_THRESHOLD = 1.0

# Lateral limits and controller parameters for BMW angle control
class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # Absolute ceiling; 200 deg is plenty for standard OP driving
    200,  # deg
    
    # UP: Steering INTO a curve (slightly smoothed to prevent jerk faults)
    # 2.5 deg/frame @ 10ms = 250 deg/sec
    ([0., 5., 25.], [2.5, 1.5, 0.3]),
    
    # DOWN: Returning to CENTER (allowed to move faster)
    ([0., 5., 25.], [3.5, 2.5, 0.5]),

    # Vehicle model-based limits (accounting for ~6% road bank)
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * 0.06),  
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * 0.06),                 

    # MAX_ANGLE_RATE is usually redundant if passing the UP/DOWN arrays above.
    # If your specific OP fork requires it, set it to match your 0 mph UP limit.
    MAX_ANGLE_RATE=2.5,  
  )

  # Angle command is sent every other frame (~50 Hz when DT_CTRL=100 Hz)
  STEER_STEP = 2

  # reduce steering torque on highway for easier driver intervention. Not sure
  # how much of a difference this actually makes.
  WEAKEN_FORCE_BP = [22., 31.]
  WEAKEN_FORCE_V  = [250, 250]

  # --- Variable Sport Steering (VSS) Calibration ---
  VSS_ANGLE_DEG = np.array([0.00, 1.00, 5.00, 10.00, 30.00, 60.00, 90.00, 180.00])
  VSS_ANGLE_RAD = VSS_ANGLE_DEG * np.pi / 180.0
  
  # The TRUE physical steering ratios at the respective angles above
  VSS_RATIO = np.array([
    14.100,  # Center
    14.072, 
    13.959, 
    13.846, 
    13.578, 
    13.325, 
    13.127, 
    12.634   # Off-center / Full Lock
  ])