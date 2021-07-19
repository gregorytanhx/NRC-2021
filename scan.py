#!/usr/bin/env pybricks-micropython
import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.nxtdevices import ColorSensor as nxtColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

stopwatch = StopWatch()

def scanHouseEV3(house, sensor, gyrostraight, speed, ev3, degrees = 500):
  gyrostraight.base.reset()
  while gyrostraight.base.leftMotor.angle() <= degrees:
    start = stopwatch.time()
    detected = False
    gyrostraight.move(speed)
    r, g, b = sensor.read('RGB-RAW')
    if r + g + b > 15:
      detected = True
     
      if r - b >= 3 and r - g >= 3:
        house.append(Color.YELLOW)
      elif b - r >= 3 and b - g >= 3:
        house.append(Color.BLUE)
      elif g - r >= 3 and g - b >= 3:
        house.append(Color.GREEN) 
      else:
        detected = False
    if detected:
      while r + g + b > 12:
        r, g, b = sensor.read('RGB-RAW')
        gyrostraight.move(speed)
      detected = False

  print(house)
