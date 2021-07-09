from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor


def CorrectSpeed(x):
  return (x/100) * 1400 
  
class Base:
  def __init__(self, 
               leftMotor: Motor, 
               rightMotor: Motor, 
               colRight: ColorSensor, 
               colLeft: ColorSensor):
    
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor
    self.colLeft = colLeft
    self.colRight = colRight
    self.clock = StopWatch()
    
  def stop(self):
    leftMotor.hold()
    rightMotor.hold()
    
  def reset(self):
    leftMotor.reset_angle(0)
    rightMotor.reset_angle(0)
    
  def lineSquare(self, color: int, speed: float):
    run1 = True
    run2 = True
    while run1 or run2:
      if self.colLeft.color() == color:
        self.leftMotor.hold()
        run1 = False
      else:
        self.leftMotor.run(CorrectSpeed(speed))
  
      if self.rightSensor.color() == color:
        self.rightMotor.hold()
        run2 = False
      else:
        self.rightMotor.run(CorrectSpeed(speed))
        
  def run(self, leftSpeed: float, rightSpeed: float):
    self.leftMotor.run(CorrectSpeed(leftSpeed))
    self.rightMotor.run(CorrectSpeed(rightSpeed))
    
  def run_time(self, speed: float, time: int):
    # time in seconds
    start = self.clock.time()
    while self.clock.time() - start < time * 1000:
      self.run(speed, speed)
    self.stop()