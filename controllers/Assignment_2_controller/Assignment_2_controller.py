"""Assignment controller."""
from controller import Robot, DistanceSensor, Motor, Lidar
import math

def stopMotors():
    motorLeft.setVelocity(0)
    motorRight.setVelocity(0)
    
def turn_90(direction):
    motorRight.setPosition(1)
    motorLeft.setPosition(-1)

    #motorRight.setVelocity(-1)
    #motorLeft.setVelocity(1)

robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#setup motors
motorLeft = robot.getMotor('wheel_left')
motorRight = robot.getMotor('wheel_right')

motorLeft.setPosition(float('inf'))
motorRight.setPosition(float('inf'))
motorLeft.setVelocity(0.0)
motorRight.setVelocity(0.0)

#setup sensors
ds = robot.getLidar('lidar')
ds.enable(timestep)
cam = robot.getCamera('camera')
cam.enable(timestep)

start = True
mode = 'straight'
mode_start = robot.getTime()
start_time = robot.getTime()

# Main loop:
while robot.step(timestep) != -1:
    if robot.getTime() - 2 < start_time:
        continue
    
    if start:
        #run motors
        motorLeft.setVelocity(2)
        motorRight.setVelocity(2)
        start = False
        
    # lidar data
    rangeImage = ds.getRangeImageArray()
    
    # lidar debuging
    #front = rangeImage[89][2]
    #left = rangeImage[0][2]
    #right = rangeImage[179][2]
    #front = "{:.2f}".format(front)
    #left = "{:.2f}".format(left)
    #right = "{:.2f}".format(right)
    #print(front + " | " + left + " | " + right + "      Front | Left | Right")
    
    lidar_0 = rangeImage[179][2]   # distance to right 
    lidar_30 = rangeImage[149][2]  # distance to right minus 30 degrees
    front = rangeImage[90][2]
    
    ratio = round(lidar_30 / lidar_0, 2)
    
    # turn left if wall dead ahead
    if front < 0.22:
        if mode != 'left':
            print('Turning left')
            mode = 'left'
            
        motorLeft.setVelocity(0.5)
        motorRight.setVelocity(3)
        mode_start = robot.getTime()
    
    # going parallel to the wall
    if ratio >= 1.1 and ratio <= 1.2 and robot.getTime() - 0.2 > mode_start:
        if mode != 'straight':
            print('Going straight')
            mode = 'straight'
            
        motorLeft.setVelocity(2)
        motorRight.setVelocity(2)
        mode_start = robot.getTime()
    
    # need to turn right    
    elif ratio > 1.2 and robot.getTime() - 0.2 > mode_start:
        if mode != 'right':
            print('Turning right')
            mode = 'right'
            
        motorLeft.setVelocity(3)
        motorRight.setVelocity(0.5)
        mode_start = robot.getTime()
    
    # need to turn left
    elif ratio < 1.1 and robot.getTime() - 0.2 > mode_start:
        if mode != 'left':
            print('Turning left')
            mode = 'left'
            
        motorLeft.setVelocity(0.5)
        motorRight.setVelocity(3)
        mode_start = robot.getTime()