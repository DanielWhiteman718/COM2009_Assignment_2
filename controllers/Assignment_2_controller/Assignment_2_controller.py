"""Assignment controller."""
from controller import Robot, DistanceSensor, Motor, Lidar
import math
import numpy as np

# calculates the force for turn from distances in front
def wanderer(rangeImage):
    ranges = np.array(rangeImage)[:, 2] * -1
    force = np.sum(angles * ranges * 10)
    
    if rangeImage[90][2] < 0.21:
        motorLeft.setVelocity(10)
        motorRight.setVelocity(-10)
    
    elif force > 120:
        motorLeft.setVelocity(10)
        motorRight.setVelocity(2)
        
    # turn left
    elif force < -120:
        motorLeft.setVelocity(3)
        motorRight.setVelocity(10)
        
    # go straight
    else:
        motorLeft.setVelocity(10)
        motorRight.setVelocity(10)
        
        


def stopMotors():
    motorLeft.setVelocity(0)
    motorRight.setVelocity(0)



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

cam_back = robot.getCamera('camera_back')
cam_back.enable(timestep)

start = True
mode = 'straight'
mode_start = robot.getTime()
start_time = robot.getTime()

angles = np.cos(np.deg2rad(np.linspace(0, 179, 180)))  # that's for free wanderer force calculation

base_color = [0, 0, 0]  # to hold colour of the starting zone


colours = [[128,0,0],[255,0,0],[128,128,0],[255,255,0],[0,128,0],[0,255,0],
        [0,128,128],[0,255,255],[0,0,128],[0,0,255],[128,0,128],[255,0,255]]


# algorithm for the maze part
def Maze(start, mode, mode_start):
    while robot.step(timestep) != -1:
        if robot.getTime() - 0.5 < start_time:
            continue
        
        # after 0.5s start the motors and get colour of starting zone
        if start:
            motorLeft.setVelocity(10)
            motorRight.setVelocity(10)
            global base_color
            base_color = np.array(cam_back.getImageArray())[32][32]    # read the color of the base
            start = False
            
            
        # lidar data
        rangeImage = ds.getRangeImageArray()
        
        lidar_0 = rangeImage[179][2]   # distance to right 
        lidar_30 = rangeImage[149][2]  # distance to right minus 30 degrees
        front = rangeImage[90][2]      # distance to the front
        
        ratio = round(lidar_30 / lidar_0, 2)
        
        # detecting if out of maze
        camImage = np.array(cam.getImageArray())
        colour = np.array(camImage[32][32])
        colourCatch = False
        
        
        if colour[0] == 128:
            if (colour[1] < 100) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] == 128) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] < 100) and (colour[2] == 128):
                colourCatch = True
        elif colour[0] == 255:
            if (colour[1] < 100) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] == 255) and (colour[2] < 100):
                colourCatch = True
            elif (colour[1] < 100) and (colour[2] == 255):
                colourCatch = True
        elif colour[0] < 100:
            if (colour[1] == 128) and (colour[2] < 100):
                colourCatch = True
            if (colour[1] == 255) and (colour[2] < 100):
                colourCatch = True
            if (colour[1] == 128) and (colour[2] == 128):
                colourCatch = True
            if (colour[1] == 255) and (colour[2] == 255):
                colourCatch = True
            if (colour[1] < 100) and (colour[2] == 128):
                colourCatch = True
            if (colour[1] < 100) and (colour[2] == 255):
                colourCatch = True
            
            
        if colourCatch:
            stopMotors()
            print("OUT OF MAZE")
            break
        
        # maze wall following

        # turn left if wall dead ahead
        if front < 0.24 and mode != 'wall_ahead':
            mode = 'wall_ahead'    
            motorLeft.setVelocity(1.1)
            motorRight.setVelocity(10)
            #print("WALL AHEAD")
            continue
        
        # robot is too close to wall AND is going into the wall 
        if lidar_0 < 0.09 and mode != 'collision_avoid' and ratio <= 1.15:
            mode = 'collision_avoid'
            motorRight.setVelocity(10)
            motorLeft.setVelocity(3)
            #print("COLLISION AVOID")
            continue
        
        # robot was too close to the wall and is now turning left, go straight once it's parallel
        if mode == 'collision_avoid' and ratio > 1.15:
            motorRight.setVelocity(10)
            motorLeft.setVelocity(10)
            mode = 'straight'
            #print("COLLISION AVOID CONTINUE")
            continue
        
        # going parallel to the wall (lidar distances and wall make ~(30 60 90) triangle)
        if ratio >= 1.12 and ratio <= 1.17 and mode!= 'straight':
            mode = 'straight'   
            motorLeft.setVelocity(10)
            motorRight.setVelocity(10)
            #print("STRAIGHT")
            continue
        
        # need to turn right (30* distance too short)
        if ratio > 1.17 and mode != 'wall_ahead' and mode != 'right' and robot.getTime() - 0.1 > mode_start:
            mode = 'right'
            motorLeft.setVelocity(10)
            motorRight.setVelocity(3)
            mode_start = robot.getTime()
            #print("RIGHT")
            continue
        
        # need to turn left (30* distance too long)
        if ratio < 1.12 and mode != 'wall_ahead' and mode !='left' and robot.getTime() - 0.3 > mode_start:
            mode = 'left'
            motorLeft.setVelocity(3)
            motorRight.setVelocity(10)
            mode_start = robot.getTime()
            #print("LEFT")
            continue
        
        pass


# when robot is facing the beacon and just needs to close in and stop
def hug_beacon(rangeImage):
    if rangeImage[90][2] < 0.2 or rangeImage[80][2] < 0.21 or rangeImage[100][2] < 0.21 :
        motorLeft.setVelocity(0)
        motorRight.setVelocity(0)
        print("BEACON REACHED")
        
        global mode
        mode = 'finished'


# returns true if given colour is close enough to the base colour
def match_color(rgb):
    distance = np.sum(np.square(rgb - base_color))
    distance = np.sqrt(distance)
    return distance < 100


# algorithm for the beaconing part, var mode is local
def Beaconing():
    print("BEACONING MODE")
    global mode
    while robot.step(timestep) != -1:
  
        # lidar data
        rangeImage = ds.getRangeImageArray()
        # camera data
        camImage = np.array(cam.getImageArray())
        
        if mode == 'free' and match_color(camImage[32][32]):
            motorLeft.setVelocity(10)
            motorRight.setVelocity(10)
            mode = 'hug'
            print("HUG MODE")
        
        if mode == 'hug':
            hug_beacon(rangeImage)
        elif mode == 'free':
            wanderer(rangeImage)
            
        pass

Maze(start, mode, mode_start)
mode = 'free'
Beaconing()