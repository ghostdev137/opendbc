from panda import Panda
from opendbc.car import get_safety_config, car
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.nissan.values import CAR


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: car.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> car.CarParams:
    ret.carName = "nissan"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.nissan)]
    ret.autoResumeSng = False

    ret.steerLimitTimer = 1.0

    ret.steerActuatorDelay = 0.1

    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    if candidate == CAR.NISSAN_ALTIMA:
      # Altima has EPS on C-CAN unlike the others that have it on V-CAN
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_NISSAN_ALT_EPS_BUS

    return ret
