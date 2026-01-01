from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.bmw.carcontroller import CarController
from opendbc.car.bmw.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "bmw"

    # Multi-panda: keep internal panda (index 0) for peripherals/fan only, external panda (index 1) for CAN
    # panda[0] -> NO_OUTPUT, panda[1] -> ALL_OUTPUT (temporary until BMW safety is ready)
    ret.safetyConfigs = [
      get_safety_config(structs.CarParams.SafetyModel.noOutput),   # internal panda
      get_safety_config(structs.CarParams.SafetyModel.allOutput),  # external panda
    ]

    # Angle-only lateral; keep stock longitudinal
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True

    ret.steerActuatorDelay = 0.27
    ret.steerLimitTimer = 0.8

    # system is capable of steering at standstill, but disabled due to
    # unnecessary movements close to and at standstill
    ret.steerAtStandstill = False
    
    ret.radarUnavailable = True

    return ret
