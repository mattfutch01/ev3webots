# SPDX-License-Identifier: MIT
# Copyright (c) 2018-2020 The Pybricks Authors

"""Generic cross-platform module for typical devices like lights, displays,
speakers, and batteries."""

from .parameters import Direction, Stop, Button, Port
from controller import Keyboard
from .robot import RobotSingleton as _Robot
from .util import rad_to_deg, deg_to_rad, port_connection_error_msg
from .parameters import Color
from controller import Motor as _Motor
import os 
import threading

class Motor():
    """Generic class to control motors with built-in rotation sensors."""

    def __init__(self, port,
                 positive_direction=Direction.CLOCKWISE,
                 gears=None):
        """

        Arguments:
            port (Port): Port to which the motor is connected.
            positive_direction (Direction): Which direction the motor should
                turn when you give a positive speed value or
                angle.
            gears (list):
                List of gears linked to the motor.

                For example: ``[12, 36]`` represents a gear train with a
                12-tooth and a 36-tooth gear. Use a list of lists for multiple
                gear trains, such as ``[[12, 36], [20, 16, 40]]``.

                When you specify a gear train, all motor commands and settings
                are automatically adjusted to account for the resulting gear
                ratio.  The motor direction remains unchanged by this.
        """
        self.robot = _Robot().get_robot()
        self.time_step = _Robot().get_time_step()
        self.motor = self._initialize_motor(port)
        self.position_sensor = self._initialize_position_sensor()
        self.positive_direction = positive_direction
        self.gears = gears or []
        self.angle_offset = 0.0

    def _initialize_motor(self, port):
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
        motor_name = port_mapping.get(port)

        if motor_name is None:
            raise ValueError(port_connection_error_msg())

        motor = self.robot.getDevice(motor_name)

        if not isinstance(motor, _Motor):
            raise ValueError(port_connection_error_msg())

        return motor

    def _initialize_position_sensor(self):
        position_sensor = self.motor.getPositionSensor()
        position_sensor.enable(self.time_step)
        return position_sensor

    def angle(self):
        """Gets the rotation angle of the motor.

        Returns:
            :ref:`angle`: Motor angle.

        """
        return round(rad_to_deg(self.position_sensor.getValue()) - self.angle_offset)

    def speed(self):
        """Gets the speed of the motor.

        Returns:
            :ref:`speed`: Motor speed.

        """
        return rad_to_deg(self.motor.getVelocity())

    def reset_angle(self, angle):
        """Sets the accumulated rotation angle of the motor to a desired value.

        Arguments:
            angle (:ref:`angle`): Value to which the angle should be reset.
        """
        self.angle_offset = round(rad_to_deg(self.position_sensor.getValue()) - angle)

    def hold(self):
        """Stops the motor and actively holds it at its current angle."""
        self.motor.setVelocity(0.0)


    def run(self, speed):
        """Runs the motor at a constant speed.

        The motor accelerates to the given speed and keeps running at this
        speed until you give a new command.

        Arguments:
            speed (:ref:`speed`): Speed of the motor.
        """
        speed_direction = 1
        if self.positive_direction == Direction.CLOCKWISE:
            self.motor.setPosition(float('+inf'))
            speed_direction = 1.0
        else:
            self.motor.setPosition(-float('+inf'))
            speed_direction = -1.0

        self.motor.setVelocity(speed_direction * deg_to_rad(speed))

    def run_time(self, speed, time, then=Stop.HOLD, wait=True):
        """Runs the motor at a constant speed for a given amount of time.

        The motor accelerates to the given speed, keeps running at this speed,
        and then decelerates. The total maneuver lasts for exactly the given
        amount of ``time``.

        Arguments:
            speed (:ref:`speed`): Speed of the motor.
            time (:ref:`time`): Duration of the maneuver.
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the maneuver to complete before continuing
                         with the rest of the program.
        """

        def run_time_thread():
            start_time = self.robot.getTime()
            while self.robot.getTime() - start_time < time / 1000.0:
                self.run(speed)
                self.robot.step(self.time_step)

            if then == Stop.HOLD:
                self.hold()
            elif then == Stop.COAST:
                self.motor.setVelocity(0.0)
        if wait:
            start_time = self.robot.getTime()
            while self.robot.getTime() - start_time < time / 1000.0:
                self.run(speed)  
                self.robot.step(self.time_step)                      

            if then == Stop.HOLD:
                self.hold()
            elif then == Stop.COAST:
                self.motor.setVelocity(0.0)
        else:
            motor_thread_handler = threading.Thread(target=run_time_thread)
            motor_thread_handler.start()

    def run_angle(self, speed, rotation_angle, then=Stop.HOLD, wait=True):
        """
        Arguments:
            speed (:ref:`speed`): Speed of the motor.
            rotation_angle (:ref:`angle`): Angle by which the motor should rotate.
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the maneuver to complete before continuing
                         with the rest of the program.
        """
        if speed > 0:
            self.positive_direction = Direction.CLOCKWISE if rotation_angle > 0 else Direction.COUNTERCLOCKWISE
        else:
            self.positive_direction = Direction.COUNTERCLOCKWISE if rotation_angle > 0 else Direction.CLOCKWISE

        def run_angle_thread():
            signed_speed = abs(speed)
            start_run_angle = abs(self.angle())
            abs_rotation_angle = abs(rotation_angle)

            while abs(abs(self.angle()) - start_run_angle) < abs_rotation_angle:
                self.run(signed_speed)

            if then == Stop.HOLD:
                self.hold()
            elif then == Stop.COAST:
                self.motor.setVelocity(0.0)

        if wait:
            signed_speed = abs(speed)
            start_run_angle = abs(self.angle())
            abs_rotation_angle = abs(rotation_angle)

            while abs(abs(self.angle()) - start_run_angle) < abs_rotation_angle:
                self.run(signed_speed)
                self.robot.step(self.time_step)

            if then == Stop.HOLD:
                self.hold()
            elif then == Stop.COAST:
                self.motor.setVelocity(0.0)
        else:
            motor_thread_handler = threading.Thread(target=run_angle_thread)
            motor_thread_handler.start()

    def run_target(self, speed, target_angle, then=Stop.HOLD, wait=True):
        """Runs the motor at a constant speed towards a
        given target angle.

        The direction of rotation is automatically selected based on the target
        angle. It does NOT matter if ``speed`` is positive or negative.

        Arguments:
            speed (:ref:`speed`): Speed of the motor.
            target_angle (:ref:`angle`): Angle that the motor should
                                         rotate to.
            then (Stop): What to do after coming to a standstill.
            wait (bool): Wait for the motor to reach the target
                         before continuing with the rest of the
                         program.
        """   
        self.positive_direction = Direction.CLOCKWISE if target_angle > self.angle() else Direction.COUNTERCLOCKWISE
     
        def run_target_thread():
            abs_speed = abs(speed)

            while abs(self.angle()) < abs(target_angle):
                self.run(abs_speed)
        
            if then == Stop.HOLD:
                self.hold()
            elif then == Stop.COAST:
                self.motor.setVelocity(0.0)

        if wait:
            abs_speed = abs(speed)

            while abs(self.angle()) < abs(target_angle):
                self.run(abs_speed)
                self.robot.step(self.time_step)

            if then == Stop.HOLD:
                self.hold()
            elif then == Stop.COAST:
                self.motor.setVelocity(0.0)
        else:
            motor_thread_handler = threading.Thread(target=run_target_thread)
            motor_thread_handler.start()

    def run_until_stalled(self, speed, then=Stop.COAST, duty_limit=None):
        """Runs the motor at a constant speed until it stalls.

        Arguments:
            speed (:ref:`speed`): Speed of the motor.
            then (Stop): What to do after coming to a standstill.
            duty_limit (:ref:`percentage`): Torque limit during this
                command. This is useful to avoid applying the full motor
                torque to a geared or lever mechanism.

        Returns:
            :ref:`angle`: Angle at which the motor becomes stalled.
        """
        print("run_until_stalled function is not implemented because friction isn't high enough, use bumper sensor instead.")

    def track_target(self, target_angle):
        """Tracks a target angle. This is similar to :meth:`.run_target`, but
        the usual smooth acceleration is skipped: it will move to the target
        angle as fast as possible. This method is useful if you want to
        continuously change the target angle.

        Arguments:
            target_angle (:ref:`angle`): Target angle that the motor should
                                         rotate to.

        """
        self.positive_direction = Direction.CLOCKWISE if target_angle > self.angle() else Direction.COUNTERCLOCKWISE

        def run_target_thread():
            while abs(self.angle()) < abs(target_angle):
                self.run(450)
            self.motor.setVelocity(0.0)

        if abs(abs(target_angle) - abs(self.angle())) > 3:
            motor_thread_handler = threading.Thread(target=run_target_thread)
            motor_thread_handler.start()


    def dc(self, duty):
        """Rotates the motor at a given duty cycle (also known as "power").

        This method lets you use a motor just like a simple DC motor.

        Arguments:
            duty (:ref:`percentage`): The duty cycle (-100.0 to 100).
        """
        self.motor.setVelocity(duty / 100.0 * self.motor.getMaxVelocity())
     
    def stop(self):
        """Stops the motor and lets it spin freely.
        
        The motor gradually stops due to friction."""
        self.motor.setVelocity(0)

    def brake(self):
        """Passively brakes the motor.
        
        The motor stops due to friction, plus the voltage that
        is generated while the motor is still moving."""
        self.motor.setVelocity(0)    

class Speaker:
    """Plays beeps and sounds using a speaker."""
    """https://pybricks.com/ev3-micropython/media.html"""

    def __init__(self):
        self.robot = _Robot().get_robot()
        self.speaker = self.robot.getDevice("Speaker")
        self.beep_volume = 1.0 
        self.pcm_volume = 1.0
        self.speed = 1.0
        self.pitch = 0

    def beep(self, frequency=500, duration=100):
        """Play a beep/tone.

        Arguments:
            frequency (:ref:`frequency`):
                Frequency of the beep. Frequencies below 100
                are treated as 100.
            duration (:ref:`time`):
                Duration of the beep. If the duration is less
                than 0, then the method returns immediately and the frequency
                play continues to play indefinitely.
        """
        print("Beep!")
        self.speaker.playSound(self.speaker, self.speaker, '../pybricks/sounds/beep.wav', self.beep_volume, 1, 1, False)

    def play_notes(self, notes, tempo=120):
        """Plays a sequence of musical notes.

        For example, you can play: ['C4/4', 'C4/4', 'G4/4', 'G4/4'].

        Arguments:
            notes (iter):
                A sequence of notes to be played (see format below).
            tempo (int):
                Beats per minute where a quarter note is one beat.
        """
        pass

    def play_file(self, file_name):
        """Plays a sound file.

        Arguments:
            file_name (str):
                Path to the sound file, including the file extension.
        """
        self.speaker.playSound(self.speaker, self.speaker, file_name, self.pcm_volume, 1, 1, False)

    def say(self, text):
        """Says a given text string.

        You can configure the language and voice of the text using
        :meth:`set_speech_options`.

        Arguments:
            text (str): What to say.
        """
        self.speaker.speak(text, self.pcm_volume)

    def set_speech_options(self, language=None, voice=None, speed=None, pitch=None):
        """Configures speech settings used by the :meth:`say` method.

        Any option that is set to None will not be changed. If an option
        is set to an invalid value :meth:`say` will use the default value
        instead.

        Arguments:
            language (str):
                Language of the text. For example, you can choose 'en'
                (English) or 'de' (German). A list of all available
                languages is given below.
            voice (str):
                The voice to use. For example, you can choose 'f1' (female
                voice variant 1) or 'm3' (male voice variant 3). A list of
                all available voices is given below.
            speed (int):
                Number of words per minute.
            pitch (int):
                Pitch (0 to 99). Higher numbers make the voice higher pitched
                and lower numbers make the voice lower pitched.
        """
        language_mapping = {
            'en-us': 'en-US',
            'en': 'en-US',
            'de': 'de-DE',
            'es': 'es-ES',
            'fr-fr': 'fr-FR',
            'it': 'it-IT',
            'en-uk-rp': 'en-UK',
            'en-uk-north': 'en-UK',
            'en-uk-wmids': 'en-UK'
        }
        if os.name == 'nt':
                self.speaker.setEngine('microsoft')

        self.speaker.setLanguage(language_mapping.get(language, 'en-US'))

    def set_volume(self, volume, which='_all_'):
        """Sets the speaker volume.

        Arguments:
            volume (:ref:`percentage`):
                Volume of the speaker.
            which (str):
                Which volume to set. 'Beep' sets the volume for
                :meth:`beep` and :meth:`play_notes`. 'PCM' sets the volume
                for :meth:`play_file` and :meth:`say`. '_all_' sets both
                at the same time.
        """
        if which in ('_all_', 'Beep'):
            self.beep_volume  = volume / 100.0 
        if which in ('_all_', 'PCM'):
            self.pcm_volume  = volume / 100.0 
            
class ColorLight:
    """Control a multi-color light."""
    def __init__(self):
        self.robot = _Robot().get_robot()
        self.led = self.robot.getDevice("LED")
        self.on(Color.GREEN)
        
    def on(self, color):
        """Turns on the light at the specified color.

        Arguments:
            color (Color): Color of the light. The light turns off if you
                           choose ``None`` or a color that is not available.
        """
        if color in Color:
            self.led.set(color.value)
        else:
            self.led.set(0)

    def off(self):
        """Turns off the light."""
        self.led.set(0)

class KeyPad:
    def __init__(self):
        self.robot = _Robot().get_robot()
        self.time_step = _Robot().get_time_step()
        self.keyboard = Keyboard()
        self.keyboard.enable(self.time_step)
        self.last_press_time = 0
        self.debounce_duration = 0.15  # Adjust this value as needed

    def pressed(self):
        """Checks which buttons are currently pressed.

        :returns: List of pressed buttons.
        :rtype: List of :class:`Button`
        """
        pressed_buttons = []

        current_time = self.robot.getTime()
        key = self.keyboard.getKey()

        if current_time - self.last_press_time >= self.debounce_duration:
            if key == Keyboard.UP:
                print("Button.UP")
                pressed_buttons.append(Button.UP)
            elif key == Keyboard.DOWN:
                print("Button.DOWN")
                pressed_buttons.append(Button.DOWN)
            elif key == Keyboard.LEFT:
                print("Button.LEFT")
                pressed_buttons.append(Button.LEFT)
            elif key == Keyboard.RIGHT:
                print("Button.RIGHT")
                pressed_buttons.append(Button.RIGHT)
            elif key == 4:  # Assuming 4 represents the Enter key
                print("Button.CENTER")
                pressed_buttons.append(Button.CENTER)
            
            self.last_press_time = current_time

        return pressed_buttons



class Battery:
    """Get the status of a battery."""

    def voltage(self):
        """Gets the voltage of the battery.

        Returns:
            :ref:`voltage`: Battery voltage.
        """
        return 8.0

    def current(self):
        """Gets the current supplied by the battery.

        Returns:
            :ref:`current`: Battery current.

        """
        return 3.0