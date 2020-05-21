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
    if robot.getTime() - 0.5 < start_time:
        continue
    
    if start:
        #run motors
        motorLeft.setVelocity(10)
        motorRight.setVelocity(10)
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
    if front < 0.24 and mode != 'wall_ahead':
        mode = 'wall_ahead'    
        motorLeft.setVelocity(1.1)
        motorRight.setVelocity(10)
        continue
    
    # robot is too close to wall AND is going into the wall 
    if lidar_0 < 0.08 and mode != 'collision_avoid' and ratio <= 1.15:
        mode = 'collision_avoid'
        motorRight.setVelocity(10)
        motorLeft.setVelocity(3)
        continue
    
    # robot was too close to the wall and is now turning left, go straight once it's parallel
    if mode == 'collision_avoid' and ratio > 1.15:
        motorRight.setVelocity(10)
        motorLeft.setVelocity(10)
        mode = 'straight'
        continue
    
    # going parallel to the wall (lidar distances and wall make ~(30 60 90) triangle)
    if ratio >= 1.12 and ratio <= 1.17 and mode!= 'straight':
        mode = 'straight'   
        motorLeft.setVelocity(10)
        motorRight.setVelocity(10)
        continue
    
    # need to turn right (30* distance too short)
    if ratio > 1.18 and mode != 'wall_ahead' and mode != 'right':
        mode = 'right'
        motorLeft.setVelocity(10)
        motorRight.setVelocity(3)
        mode_start = robot.getTime()
        continue
    
    # need to turn left (30* distance too long)
    if ratio < 1.12 and mode != 'wall_ahead' and mode !='left' and robot.getTime() - 0.3 > mode_start:
        mode = 'left'
        motorLeft.setVelocity(3)
        motorRight.setVelocity(10)
        mode_start = robot.getTime()
        continue