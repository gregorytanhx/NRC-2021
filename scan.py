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

def scanHouseEV3(house, sensor, gyrostraight, speed, ev3):
  while gyrostraight.base.colRight.reflection() < 80:
    start = stopwatch.time()
    detected = False
    gyrostraight.move(speed)
    r, g, b = sensor.read('RGB-RAW')
    print(r,g,b)
    if r + g + b > 10:
      detected = True
     
      if r >= 8:
        house.append(Color.YELLOW)
      elif b >= 8:
        house.append(Color.BLUE)
      elif g >= 5 and b >= 5:
        house.append(Color.GREEN) 
      else:
        detect = False
    if detected:
      while r + g + b > 10:
        r, g, b = sensor.read('RGB-RAW')
        gyrostraight.move(speed)
      detected = False
  gyrostraight.base.stop()
  print(house)
