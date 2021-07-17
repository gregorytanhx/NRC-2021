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
               colLeft: ColorSensor, 
               colRight: ColorSensor):
    
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor
    self.colLeft = colLeft
    self.colRight = colRight
    self.clock = StopWatch()
    
  def stop(self):
    self.leftMotor.brake()
    self.rightMotor.brake()
  
  def hold(self):
    self.leftMotor.hold()
    self.rightMotor.hold()
    
  def reset(self):
    self.leftMotor.reset_angle(0)
    self.rightMotor.reset_angle(0)
    
  def run(self, leftSpeed: float, rightSpeed: float):
    self.leftMotor.run(CorrectSpeed(leftSpeed))
    self.rightMotor.run(CorrectSpeed(rightSpeed))
    
  def run_time(self, speed: float, time: int):
    # time in seconds
    start = self.clock.time()
    while self.clock.time() - start < time * 1000:
      self.run(speed, speed)
      print(self.leftMotor.angle(), self.rightMotor.angle()) 
    self.stop()
    
  def run_target(self, speed, angle):
    self.reset()
    self.leftMotor.run_target(CorrectSpeed(speed), angle, wait=False)
    self.rightMotor.run_target(CorrectSpeed(speed), angle, wait=True)
    print(self.leftMotor.angle())