
def scanHouse(house, sensor, GyroStraight, speed, ev3):
  while GyroStraight.leftMotor.angle() < 300:
    detected = True
    Straight.move(speed)
    reading = sensor.read('COLOR') 
    if reading == 2: # blue
      house.append(0)
    elif reading == 3: # green
      house.append(1)
    elif reading == 4: # yellow
      house.append(2)
    else:
      detected = False
      
    # if color detected, move until no color detected
    ev3.screen.print(house)
    if detected:
      while GyroStraight.leftMotor.angle() < 300 and reading != 0:
        reading, _ , _ , _ = sensor.read('rgbw')
        Straight.move(speed)
        ev3.screen.print(house)