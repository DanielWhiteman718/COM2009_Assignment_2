"""Assignment controller."""
from controller import Robot, DistanceSensor, Motor, Lidar, Camera
import math

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

#run motors
motorLeft.setVelocity(2)
motorRight.setVelocity(2)

while robot.step(timestep) != -1:
    front = ds.getRangeImageArray()[255][2]
    left = ds.getRangeImageArray()[0][2]
    right = ds.getRangeImageArray()[511][2]
    
    #colour list
    colors = [('Maroon',[0.5,0,0]),('Red',[1,0,0]),('Olive',[0.5,0.5,0]),
    ('Yellow',[1,1,0]),('Green',[0,0.5,0]),('Lime',[0,1,0]),('Teal',[0,0.5,0.5]),
    ('Aqua',[0,1,1]),('Navy',[0,0,0.5]),('Blue',[0,0,1]),('Purple',[0.5,0,0.5]),
    ('Fuchsia',[1,0,1])]
    
    
    def getRGB(cam):
      r,g,b = 0,0,0
      image = cam.getImageArray()
      pixel = cam.getWidth()*cam.getHeight()
      # only get one centred pixel
      #r = image[32][32][0]
      #g = image[32][32][1]
      #b = image[32][32][2]
      #print(r, g, b)
      
      for x in range(0,cam.getWidth()):
        for y in range(0,cam.getHeight()):
          r += image[x][y][0]
          g += image[x][y][1]
          b += image[x][y][2]
          rgb = [getColour(r), getColour(g), getColour(b)]
              
    def getColour(c):
      if c > 200 and c < 255:
        return 1
      elif c > 125 and c < 200:
        return 0.5
      else:
        return 0
        
    getRGB(cam)
    
    front = "{:.2f}".format(front)
    left = "{:.2f}".format(left)
    right = "{:.2f}".format(right)
    