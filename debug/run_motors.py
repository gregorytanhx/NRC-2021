#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

ev3 = EV3Brick()
try: motorA = Motor(Port.A)
except: pass
try: motorB = Motor(Port.B)
except: pass
try: motorC = Motor(Port.C)
except: pass
try: motorD = Motor(Port.D)
except: pass
speed = 600
motor1 = motorA
motor2 = motorD
pressed = False
clock = StopWatch()
lastPressedTime = 0
while True:

  buttons = ev3.buttons.pressed()

  if Button.LEFT in buttons:
    motor1.run(speed)
  elif Button.RIGHT in buttons:
    motor1.run(-speed)
  else:
    motor1.hold()
  if Button.UP in buttons:
    motor2.run(speed)
  elif Button.DOWN in buttons:
    motor2.run(-speed)
  else:
    motor2.hold()
  if Button.CENTER in buttons:
    if motor1 == motorA:
      motor1 = motorB
      motor2 = motorC
    else:
      motor1 = motorA
      motor2 = motorD