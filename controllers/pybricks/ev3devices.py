# SPDX-License-Identifier: MIT
# Copyright (c) 2018-2020 The Pybricks Authors

"""LEGO® MINDSTORMS® EV3 motors and sensors."""

from .parameters import Direction as _Direction
from .parameters import Port
from ._common import Motor  # noqa E402
from .robot import RobotSingleton as _Robot
from .util import port_connection_error_msg
from controller import Gyro as _Gyro
from controller import TouchSensor as _TouchSensor
from controller import DistanceSensor as _DistanceSensor
import random 

class TouchSensor:
    """LEGO® MINDSTORMS® EV3 Touch Sensor."""
    def __init__(self, port):
        self.robot = _Robot().get_robot()
        self.time_step = _Robot().get_time_step()
        self.touch_sensor = self._initialize_touch_sensor(port)
        
        
    def _initialize_touch_sensor(self, port):
        port_mapping = {
            Port.A: "A",
            Port.B: "B",
            Port.C: "C",
            Port.D: "D",
            Port.S1: "S1",
            Port.S2: "S2",
            Port.S3: "S3",
            Port.S4: "S4"
        }
        
        sensor_name = port_mapping.get(port)
        if sensor_name is None:
            raise ValueError(port_connection_error_msg())
        
        touch_sensor = self.robot.getDevice(sensor_name)

        if not isinstance(touch_sensor, _TouchSensor):
            raise ValueError(port_connection_error_msg())

        touch_sensor.enable(self.time_step)
        return touch_sensor
    
    def pressed(self):
        """Checks if the sensor is pressed.

        Returns:
            bool: ``True`` if the sensor is pressed, ``False`` if it is not pressed.
        """
        return self.touch_sensor.getValue()

class GyroSensor:
    """LEGO® MINDSTORMS® EV3 Gyro Sensor."""

    def __init__(self, port, positive_direction=_Direction.CLOCKWISE):
        """

        Arguments:
            port (Port): Port to which the sensor is connected.
            positive_direction (Direction):
                Positive rotation direction when looking at the red dot on top
                of the sensor.

        """
        self.robot = _Robot().get_robot()
        self.time_step = _Robot().get_time_step()
        self.gyro = self._initialize_gyro(port)
        self.positive_direction = positive_direction
        self.accumulated_angle = 0
        self.angle_offset = 0
        
    def _initialize_gyro(self, port):
            port_mapping = {
                Port.A: "A",
                Port.B: "B",
                Port.C: "C",
                Port.D: "D",
                Port.S1: "S1",
                Port.S2: "S2",
                Port.S3: "S3",
                Port.S4: "S4"
            }
            
            sensor_name = port_mapping.get(port)
            if sensor_name is None:
                raise ValueError(port_connection_error_msg())

            gyro = self.robot.getDevice(sensor_name)

            if not isinstance(gyro, _Gyro):
                raise ValueError(port_connection_error_msg())

            gyro.enable(self.time_step)
            return gyro

    def speed(self):
        """Gets the speed (angular velocity) of the sensor.

        Returns:
            :ref:`speed`: Sensor angular velocity.

        """
        if self.positive_direction == _Direction.CLOCKWISE:
            return int(-self.gyro.getValues()[2] * 180.0 / 3.14)

        return int(self.gyro.getValues()[2] * 180.0 / 3.14)

    def update_gyro_angle(self):
        """Updates the angle of the sensor.

        This method should be called at regular intervals to keep track of
        the accumulated angle of the sensor.

        """
        if self.positive_direction == _Direction.CLOCKWISE:
            self.accumulated_angle -= self.gyro.getValues()[2] * 180.0 / 3.14 * self.time_step / 1000
        else:
            self.accumulated_angle += self.gyro.getValues()[2] * 180.0 / 3.14 * self.time_step / 1000

    def angle(self):
        """Gets the accumulated angle of the sensor.

        Returns:
            :ref:`angle`: Rotation angle.

        """
        # Adding +/- 3 degrees of noise to the angle
        return int(self.accumulated_angle - self.angle_offset) + random.randint(-3, 3)

    def reset_angle(self, angle):
        """Sets the rotation angle of the sensor to a desired value.

        Arguments:
            angle (:ref:`angle`): Value to which the angle should be reset.
        """
        self.angle_offset = self.accumulated_angle - angle

    def _calibrate(self):
        """Calibrates the sensor.

        This process sets the speed and angle to zero and ensures that the
        angle value does not drift.

        Make sure that the sensor does not move while calibrating.

        This process can take up to 15 seconds.
        """
        print("Wait 15 seconds to calibrate the Gyro Sensor when using the real robot")


class UltrasonicSensor:
    """LEGO® MINDSTORMS® EV3 Ultrasonic Sensor."""

    def __init__(self, port):
        """UltrasonicSensor(port)

        Arguments:
            port (Port): Port to which the sensor is connected.

        """
        self.robot = _Robot().get_robot()
        self.time_step = _Robot().get_time_step()
        self.ultrasonic_sensor = self._initialize_ultrasonic_sensor(port)

    def _initialize_ultrasonic_sensor(self, port):
        port_mapping = {
            Port.A: "A",
            Port.B: "B",
            Port.C: "C",
            Port.D: "D",
            Port.S1: "S1",
            Port.S2: "S2",
            Port.S3: "S3",
            Port.S4: "S4"
        }

        sensor_name = port_mapping.get(port)
        if sensor_name is None:
            raise ValueError(port_connection_error_msg())

        ultrasonic_sensor = self.robot.getDevice(sensor_name)

        if not isinstance(ultrasonic_sensor, _DistanceSensor):
            raise ValueError(port_connection_error_msg())

        ultrasonic_sensor.enable(self.time_step)
        return ultrasonic_sensor

    def distance(self, silent=False):
        """Measures the distance between the sensor and an object using
        ultrasonic sound waves.

        Arguments:
            silent (bool): Choose ``True`` to turn the sensor off after
                           measuring the distance. This reduces interference
                           with other ultrasonic sensors. If you do
                           this too frequently, the sensor can freeze.
                           If this happens, unplug it and plug it back in.

        Returns:
            :ref:`distance`: Distance.

        """
        if silent:
            self.ultrasonic_sensor.enable(self.time_step)  # Re-enable the sensor

        distance = int(self.ultrasonic_sensor.getValue())  # Assuming getValue() returns the distance

        if silent:
            self.ultrasonic_sensor.disable()  # Disable the sensor after measurement
        return distance

    def presence(self):
        """Checks for the presence of other ultrasonic sensors by detecting
        ultrasonic sounds.

        If the other ultrasonic sensor is operating in silent mode, you can
        only detect the presence of that sensor while it is taking a
        measurement.

        Returns:
            bool: ``True`` if ultrasonic sounds are detected,
            ``False`` if not.
        """
        return self.sensor.isDetected()
    