#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.byd.carcontroller import get_safety_CP
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round

STEERING_MODULE_ADAS = 0x1E2
LKAS_HUD_ADAS = 0x316
PCM_BUTTONS = 0x3B0


def safety_max_can(max_angle_float, can_offset=0):
  # Matches C: max_angle_can = (int)(max_angle * 10 + 1.) which is floor(max_angle * 10) + 1
  return int(max_angle_float * 10 + 1.) + can_offset


class TestBydSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (STEERING_MODULE_ADAS, LKAS_HUD_ADAS)}
  FWD_BLACKLISTED_ADDRS = {2: [STEERING_MODULE_ADAS, LKAS_HUD_ADAS]}
  TX_MSGS = [[STEERING_MODULE_ADAS, 0], [LKAS_HUD_ADAS, 0], [PCM_BUTTONS, 0]]

  MAIN_BUS = 0
  CAM_BUS = 2

  STEER_ANGLE_MAX = 390  # deg, EPS fault limit
  DEG_TO_CAN = 10

  # BYD uses get_max_angle_delta_vm and get_max_angle_vm for lateral accel and jerk limits
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    self.VM = VehicleModel(get_safety_CP())
    self.packer = CANPackerSafety("byd_atto3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {"STEER_ANGLE": angle, "STEER_REQ": 1 if enabled else 0, "STEER_REQ_ACTIVE_LOW": 0 if enabled else 1}
    if increment_timer:
      self.safety.set_timer(self.__class__.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("STEERING_MODULE_ADAS", self.MAIN_BUS, values)

  cnt_angle_cmd = 0

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE_2": angle}
    return self.packer.make_can_msg_safety("STEER_MODULE_2", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    # ACC_STATE: 0=OFF, 3=ACC_ACTIVE
    values = {"ACC_STATE": 3 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", self.CAM_BUS, values)

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_CLEAN": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEELSPEED_CLEAN", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": 1 if brake else 0}
    return self.packer.make_can_msg_safety("DRIVE_STATE", self.MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_safety("PEDAL", self.MAIN_BUS, values)

  def _driver_torque_msg(self, torque: float):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("STEERING_TORQUE", self.MAIN_BUS, values)

  def test_steering_wheel_disengage(self):
    # BYD disengages when the driver holds the wheel against angle control. The EPS measurement
    # is attenuated too far to see that, so this reads column torque and debounces it.
    frames = CarControllerParams.STEER_DRIVER_DISENGAGE_FRAMES
    for sign in (-1, 1):
      for torque in (CarControllerParams.STEER_DRIVER_DISENGAGE, CarControllerParams.STEER_DRIVER_DISENGAGE + 5):
        should_disengage = torque > CarControllerParams.STEER_DRIVER_DISENGAGE

        self.safety.set_controls_allowed(True)
        for _ in range(frames - 1):
          self.assertTrue(self._rx(self._driver_torque_msg(sign * torque)))
          self.assertFalse(self.safety.get_steering_disengage_prev())
          self.assertTrue(self.safety.get_controls_allowed())

        self.assertTrue(self._rx(self._driver_torque_msg(sign * torque)))
        self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())
        self.assertEqual(not should_disengage, self.safety.get_controls_allowed())

        # one frame under the threshold resets the debounce, and controls stay disallowed
        self.assertTrue(self._rx(self._driver_torque_msg(0)))
        self.assertFalse(self.safety.get_steering_disengage_prev())
        self.assertEqual(not should_disengage, self.safety.get_controls_allowed())

  def test_rx_checksum(self):
    # 8-byte BYD frames use (~sum) in the last byte. Fleet logs match that algo on
    # STEERING_TORQUE, WHEELSPEED_CLEAN, and ACC_HUD_ADAS. STEER_MODULE_2 is 4-bit and
    # DRIVE_STATE has no checksum, so those stay ignored.
    checked = (
      self._driver_torque_msg(0),
      self._speed_msg(0),
      self._pcm_status_msg(False),
    )
    for msg in checked:
      self.assertTrue(self._rx(msg))
      msg.data[7] ^= 0xFF
      self.assertFalse(self._rx(msg))

    ignored = (
      self._angle_meas_msg(0),
      self._user_brake_msg(False),
    )
    for msg in ignored:
      self.assertTrue(self._rx(msg))
      msg.data[len(msg.data) - 1] ^= 0x0F
      self.assertTrue(self._rx(msg))

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match both CAN encoding (0.0713 kph/LSB) and VEHICLE_SPEED_FACTOR=1000 rounding in UPDATE_VEHICLE_SPEED
      sent = speed + 1
      sent_can = away_round(sent / 0.0713 * 3.6) * 0.0713 / 3.6
      speed = round(sent_can * 1000) / 1000 - 1
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        max_angle_float = get_max_angle_vm(speed, self.VM, CarControllerParams)

        # at limit (safety tolerance adds 1 CAN unit)
        max_angle_can = safety_max_can(max_angle_float)
        max_angle_can = min(max_angle_can, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        max_angle = sign * max_angle_can / self.DEG_TO_CAN
        self.safety.set_desired_angle_last(sign * max_angle_can)

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        over_can = safety_max_can(max_angle_float, 1)
        over_can_clipped = min(over_can, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        over_angle = sign * over_can_clipped / self.DEG_TO_CAN
        self._tx(self._angle_cmd_msg(over_angle, True))

        # at low speeds max angle is above STEER_ANGLE_MAX, so adding 1 has no effect
        should_tx = over_can >= self.STEER_ANGLE_MAX * self.DEG_TO_CAN
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(over_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match both CAN encoding (0.0713 kph/LSB) and VEHICLE_SPEED_FACTOR=1000 rounding in UPDATE_VEHICLE_SPEED
      sent = speed + 1
      sent_can = away_round(sent / 0.0713 * 3.6) * 0.0713 / 3.6
      speed = round(sent_can * 1000) / 1000 - 1
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        max_delta_float = get_max_angle_delta_vm(speed, self.VM, CarControllerParams)

        # Stay within limits
        # Up
        max_delta_can = safety_max_can(max_delta_float)
        max_angle_delta = sign * max_delta_can / self.DEG_TO_CAN
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        over_delta_can = safety_max_can(max_delta_float, 1)
        max_angle_delta = sign * over_delta_can / self.DEG_TO_CAN
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(sign * over_delta_can)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestBydIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("byd_atto3")

  def _msg(self, gear, bus=0):
    return self.packer.make_can_msg_safety("DRIVE_STATE", bus, {"GEAR": gear})

  def test_ignition_on_for_every_valid_gear(self):
    for gear in (1, 2, 3, 4):
      self.safety.ignition_can_hook(self._msg(gear))
      self.assertTrue(self.safety.get_ignition_can(), f"gear {gear}")

  def test_ignition_stays_on_when_gear_zero(self):
    # Off is the 2s timeout, not GEAR==0
    self.safety.ignition_can_hook(self._msg(4))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_ignores_other_bus(self):
    self.safety.ignition_can_hook(self._msg(4, bus=2))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
