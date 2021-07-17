#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.nxtdevices import ColorSensor as nxtColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

import math, time
from scan import *
from pid import *

ev3 = EV3Brick()
Houses = [[], [], []]

frontClaw = Motor(Port.A)
backClaw = Motor(Port.D)
leftMotor = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor =  Motor(Port.C)

#nxtCol = nxtColorSensor(Port.S1) 
HTCol = Ev3devSensor(Port.S1) 
gyro = GyroSensor(Port.S2)
colLeft = ColorSensor(Port.S3)
colRight = ColorSensor(Port.S4)

stopwatch = StopWatch()
base = Base(leftMotor, rightMotor, colLeft, colRight)

LineTrack = PID_LineTrack(base, 0.18, 0, 5, 40)
GyroStraight = PID_GyroStraight(base, 1.2, 0, 5, gyro)
GyroTurn = PID_GyroTurn(base, 1.1, 0.0002, 2, gyro)
gyro.reset_angle(0)
start = stopwatch.time()
while True:
   LineTrack.move(LineTrack.base.colLeft, 50, 50)
scanHouse(Houses[0], HTCol, LineTrack, 50, ev3)
wait(5000)
#PID_LineSquare(base, 60, 0.3, 0.0001, 0.5)

# base.run_target(100, 200)

# while colLeft.reflection() > 15:
#   LineTrack.move(colRight, 100, 40)
# base.stop()

# while colLeft.reflection() < 80:
#   base.run(40, 40)
# base.stop()

# while colLeft.reflection() > 15:
#   base.run(-40, -40)
# base.stop()  
# base.run_target(70, 320)

# GyroTurn.turn(90)
# # wall align
# base.run_time(100, 1)
# gyro.reset_angle(0)
# scanHouse(Houses[0], nxtCol, GyroStraight, 50, ev3)


# base.reset()
# while rightMotor.angle() < 100:
#   GyroStraight.move(80)
# base.stop()

# while colRight.reflection() > 10:
#   LineTrack.move(80)
# base.stop()

# base.reset()
# while rightMotor.angle() < 200:
#   GyroStraight.move(80)
# base.stop()



  