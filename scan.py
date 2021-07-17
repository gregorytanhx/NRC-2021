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
  linetrack.move(linetrack.base.colLeft, speed, 50)
  ev3.speaker.set_volume(100)
  while linetrack.base.colRight.reflection() > 15:
    detected = True
    linetrack.move(linetrack.base.colLeft, speed, 50)
    
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
        linetrack.move(linetrack.base.colLeft, speed, 50)
      detected = False
        
  linetrack.base.stop()
  for i in range(len(house)):
    if house[i] == Color.BLACK or house[i] == Color.BLUE:
      ev3.speaker.play_file(SoundFile.BLUE)
    elif house[i] == Color.YELLOW:
      ev3.speaker.play_file(SoundFile.YELLOW)
    elif house[i] == Color.GREEN:
      ev3.speaker.play_file(SoundFile.GREEN)
    wait(1000)

  print(house)
