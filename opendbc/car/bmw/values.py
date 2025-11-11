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
    ),
  )

  # --- BMW i4 (G26) ---
  BMW_G26_I4 = BMWSP2018PlatformConfig(
    [BMWCarDocs(name="BMW i4 (G26)")],
    BMWCarSpecs(
      mass=2150.,
      wheelbase=2.865,
      steerRatio=15.5,
      centerToFrontRatio=0.5,
    ),
  )


DBC = CAR.create_dbc_map()


# Lateral limits and controller parameters for BMW angle control
class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # Assume EPAS faults above this angle; tune with testing
    360,  # deg
    # BMW uses vehicle-model limiting; rate tables unused here
    ([], []),
    ([], []),

    # Vehicle model-based limits (start conservative; adjust after road test)
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * 0.04),  # ~3.4-3.5 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * 0.04),                 # ~3.4-3.5 m/s^3

    # prevent EPS faults and improve low-speed comfort
    MAX_ANGLE_RATE=5,  # deg/20ms frame
  )

  # Angle command is sent every other frame (~50 Hz when DT_CTRL=100 Hz)
  STEER_STEP = 2

