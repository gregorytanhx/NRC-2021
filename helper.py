from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor




def CorrectSpeed(x):
  return (x/100) * 1400 



class Claw:
  def __init__(self, port: Port):
    self.motor = Motor(port)
    self.motor.control.limits(1500)

  
  def run_angle(self, speed, angle, wait = True):
    self.motor.reset_angle(0)
    self.motor.run_angle(CorrectSpeed(speed), angle, wait = wait)
    
  def run_time(self, speed, time, wait = True):
    self.motor.run_time(CorrectSpeed(speed), time, wait = wait)    
  
  def dc(self, dir = 1, speed = 50):
    self.motor.dc(speed * dir)
    
  def hold(self):
    self.motor.hold()
    
  def reset(self):
    self.motor.reset_angle(0)
  
  def measureAngleRange(self, moveTime):
    results = []
    print("Full results:", end=' ')

    while len(results) < 10:

        self.motor.dc(-40)
        wait(moveTime)
        self.motor.hold()
        wait(1000)

        self.motor.reset_angle(0)

        self.motor.dc(40)
        wait(moveTime)
        self.motor.hold()
        wait(1000)

        angle = self.motor.angle()
        if angle != 0:                      # Skips erroneous results (see fixme above).
            results.append(angle)
            print(results[-1], end=' ')

    print("\nAverage:", sum(results) / len(results))

  
class FrontClaw(Claw):
  def __init__(self, port: Port):
    super().__init__(port)
    self.closeDist = -460
    
  def run_target(self, speed, angle, wait = True):
    self.motor.run_target(CorrectSpeed(speed), angle, wait = wait)
  
  def goUp(self, speed = 50, wait = True):
    self.run_target(30, 0, wait = wait)
  
  def goDown(self, speed = 30, wait = True):
    self.run_target(50, 405, wait = wait)
  
  def openUp(self, wait = True):
    self.run_target(100, 865, wait = wait)
  
  def defaultPos(self):
    self.dc()
    wait(1500)
    self.run_target(-50, self.closeDist, wait=False)

    # self.run_target(100 * dir, deg)
    # self.hold()
    # self.dc(dir = dir)


class BackClaw(Claw):
  def __init__(self, port: Port):
    super().__init__(port)
  
  def run_target(self, speed, angle, wait = True, reset = True):
    self.motor.reset_angle(0)
    self.motor.run_target(CorrectSpeed(speed), angle, wait = wait)
    
  def mid(self):
    self.run_target(-50, -180)
    
  def defaultPos(self):
    self.run_time(100, 1000)
    self.run_target(-50, -185)
    wait(100)
    self.run_target(50, 40)

 
class Base:
  def __init__(self, 
               leftMotor: motor, 
               rightMotor: motor, 
               colLeft: ColorSensor, 
               colRight: ColorSensor, 
               frontClaw: motor, 
               backClaw: motor):
    
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor
    self.colLeft = colLeft
    self.colRight = colRight
    self.clock = StopWatch()
    self.frontClaw = frontClaw
    self.backClaw = backClaw
    
  def stop(self):
    self.leftMotor.brake()
    self.rightMotor.brake()
  
  def hold(self):
    self.leftMotor.hold()
    self.rightMotor.hold()
    wait(10)
    
  def move(self, speed, condition):
    while condition():
      self.run(speed, speed)
    
  def reset(self):
    self.leftMotor.reset_angle(0)
    self.rightMotor.reset_angle(0)
  
  def run(self, leftSpeed: float, rightSpeed: float):
    self.leftMotor.run(CorrectSpeed(leftSpeed))
    self.rightMotor.run(CorrectSpeed(rightSpeed))
    

  def run_time(self, speed: float, time: int):
    # time in seconds
    start = self.clock.time()
    while self.clock.time() - start < time:
      self.run(speed, speed)
    self.stop()
    
  def run_target(self, speed, angle, stop = Stop.HOLD):
    self.reset()

    self.leftMotor.run_target(CorrectSpeed(speed), angle, wait=False, then = stop)
    self.rightMotor.run_target(CorrectSpeed(speed), angle, wait=True, then = stop)
