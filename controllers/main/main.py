#!/usr/bin/env pybricks-micropython

# Simulator Final
# Timothy Deng 730607227
# Matthew Futch 730519054

# ==========================Comment out when running on the real robot===================
import sys
import random
import pathlib
import os
import math

# Note: if you have matplotlib and want to visualize your robot's estimated pose in Webots (see extra credit),
# uncomment next line:
# from data_visualizer import DataVisualizer  # Note: requires matplotlib to be installed! 
# =======================================================================================

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile, ImageFile

class MyController:
    def __init__(self):
        # SENSORS
        self.ev3 = EV3Brick()
        self.leftM = Motor(Port.A)
        self.rightM = Motor(Port.B)
        self.touchSensor = TouchSensor(Port.S1)
        self.touchSensor2 = TouchSensor(Port.S2)
        self.gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)
        self.ultra = UltrasonicSensor(Port.S3)
        self.stopwatch = StopWatch()
        self.ev3.speaker.set_volume(100, "PCM")

        # CONSTANTS
        self.r = 0.03175
        self.L = 0.127

        self.state = "WAITING"

        # POSITION TRACKING
        self.position_list = [0.5, 0, math.pi/2]
        self.goal = [3.0, 3.0]
        self.lastRangle = 0
        self.lastLangle = 0

        # PID 
        self.Kp = 0.15
        self.Kd = -0.1
        self.lastDist = [0] * 5
        self.counter = 0 
        self.previous_error = 0
        self.distanceToWall = 300

    def actuate_motors(self, speed):
        self.leftM.run(speed)
        self.rightM.run(speed)

    def stop_motors(self):
        self.leftM.hold()
        self.rightM.hold()

    def turn_right(self, speed):
        self.leftM.run(speed)
        self.rightM.run(-speed)

    def compute_position(self, leftAngle, rightAngle, delta_time):
        if delta_time == 0:
            delta_time = 32
        dt = delta_time / 1000.0

        deltaLAngle = math.radians(leftAngle - self.lastLangle)
        deltaRAngle = math.radians(rightAngle - self.lastRangle)

        left_velocity = deltaLAngle * self.r / dt
        right_velocity = deltaRAngle * self.r / dt
        if abs(deltaLAngle - deltaRAngle) < 0.1:
            velocity = (left_velocity + right_velocity) / 2.0
            delta_x = velocity * math.cos(self.position_list[2]) * dt
            delta_y = velocity * math.sin(self.position_list[2]) * dt
            new_x = self.position_list[0] + delta_x
            new_y = self.position_list[1] + delta_y
        
        else:
            omega = (right_velocity - left_velocity) / self.L
            R = (self.L / 2.0) * ((left_velocity + right_velocity) / (right_velocity - left_velocity))
            ICC_x = self.position_list[0] - R * math.sin(self.position_list[2])
            ICC_y = self.position_list[1] + R * math.cos(self.position_list[2])

            omega_dt = omega * dt
            cos_odt = math.cos(omega_dt)
            sin_odt = math.sin(omega_dt)

            new_x = cos_odt * (self.position_list[0] - ICC_x) - sin_odt * (self.position_list[1] - ICC_y) + ICC_x
            new_y = sin_odt * (self.position_list[0] - ICC_x) + cos_odt * (self.position_list[1] - ICC_y) + ICC_y

        self.position_list = [new_x, new_y, math.radians(self.gyro.angle())]
        #self.ev3.screen.print("%0.2f" % self.position_list[0], "%0.2f" % self.position_list[1], "%0.2f" % math.radians(self.gyro.angle()))
        self.ev3.screen.draw_pixel(math.floor(self.position_list[0] * 30), math.floor(self.position_list[1] * 30), Color.RED)
        self.lastLangle = leftAngle
        self.lastRangle = rightAngle

        self.stopwatch.reset()


    def compute_pid(self):
        self.lastDist[self.counter] = min(self.ultra.distance(), 800)
        self.counter += 1
        self.counter = self.counter % 5
        current_distance = sum(self.lastDist) / 5
        error = self.distanceToWall - current_distance
        self.stopwatch.reset()

        derivative = error - self.previous_error
        
        errorAdjustment = error * (1 + 0.5*abs(error) / self.distanceToWall)
        speed_adjust = (self.Kp * errorAdjustment) + (self.Kd * derivative)
        speed_adjust = max(min(speed_adjust, 60), -60)

        self.previous_error = error

        base_speed = 225
        left_speed = base_speed + speed_adjust
        right_speed = base_speed - speed_adjust

        return int(left_speed), int(right_speed)
        
    def run(self):
        while True:
            """if self.position_list[0] > 4 or self.position_list[1] > 4 or self.position_list[0] < 0 or self.position_list[1] < 0:
                break"""
            if self.state == "WAITING":
                self.gyro.reset_angle(90)
                if Button.CENTER in self.ev3.buttons.pressed():
                    self.stopwatch.reset()
                    self.state = "FIND_MIDLINE"

            elif self.state == "FIND_MIDLINE":
                self.turn_right(180)
                self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                if abs(math.tan(math.radians(self.gyro.angle())) - 6 / 5) < 0.1:
                    self.state = "FOLLOW_MIDLINE"

            elif self.state == "FOLLOW_MIDLINE":
                self.actuate_motors(360)
                self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                if abs(self.position_list[0] - self.goal[0]) < 0.1 and abs(self.position_list[1] - self.goal[1]) < 0.1:
                    self.stop_motors()
                    self.ev3.speaker.beep()
                    break

                if self.touchSensor.pressed() or self.touchSensor2.pressed() or self.ultra.distance() < 190:
                    self.actuate_motors(-180)
                    wait(800)
                    self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                    self.turn_right(180)
                    wait(400)
                    self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                    self.state = "FOLLOWING_WALL"

            elif self.state == "FOLLOWING_WALL":
                # PID Movement control
                left_speed, right_speed = self.compute_pid()
                self.leftM.run(left_speed)
                self.rightM.run(right_speed)
                self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                # Touch Sensor Backup
                if self.touchSensor.pressed() or self.touchSensor2.pressed() or self.ultra.distance() < 190:
                    self.stop_motors()
                    self.ev3.speaker.play_file('oof.wav')
                    self.state = "REVERSE"

                if abs(self.position_list[0] - self.goal[0]) < 0.1 and abs(self.position_list[1] - self.goal[1]) < 0.1:
                    self.stop_motors()
                    self.ev3.speaker.beep()
                    break
                #self.ev3.screen.print(abs(self.position_list[0] * 6 / 5 - self.position_list[1] - 3/5))
                if abs(self.position_list[0] * 6 / 5 - self.position_list[1] - 3/5 ) < 0.2:
                    self.state = "FIND_MIDLINE"

            elif self.state == "REVERSE":
                self.actuate_motors(-180)
                wait(800)
                self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                self.turn_right(180)
                wait(400)
                self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                self.actuate_motors(180)
                wait(800)
                self.compute_position(self.leftM.angle(), self.rightM.angle(), self.stopwatch.time())
                self.state = "FOLLOWING_WALL"

def main():
    lab4 = MyController()
    lab4.run()

if __name__ == "__main__":
    main()
