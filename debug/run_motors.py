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

motor1 = motorA
motor2 = motorD
while True:
  buttons = ev3.buttons.pressed()
  if Button.LEFT in buttons:
    motor1.run(1000)
  elif Button.RIGHT in buttons:
    motor1.run(-1000)
  if Button.UP in buttons:
    motor2.run(1000)
  elif Button.DOWN in buttons:
    motor2.run(-1000)
  if Button.CENTER in buttons:
    if motor1 == motorA:
      motor1 = motorB
      motor2 = motorC
    else:
      motor1 = motorA
      motor2 = motorD