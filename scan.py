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

def scanHouse(house, sensor, linetrack, speed, ev3):
  while linetrack.base.colRight.reflection() > 20:
    detected = True
    linetrack.move(linetrack.base.colLeft, )
    
    r, g, b, w = sensor.read('RAW')
    
    if r >= 36 and w >= 130 and g >= 60:
      house.append(Color.YELLOW)
    elif w >= 120 and g >= 53:
      house.append(Color.GREEN)
    elif w >= 110:
      house.append(Color.BLUE)
    else:
      detected = False

    if detected:
      while w >= 105:
        r, g, b, w = sensor.read('RAW')
        base.run(speed, speed)
      detected = False
        
  base.stop()


  print(house)
