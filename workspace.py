from algorithms.sPRM import sPRM
import sys
import numpy as np
from PIL import Image, ImageTk, ImageColor
from io import BytesIO
from tkinter import ttk, Canvas, NW
import os
from configspace import Configspace

class Workspace:

    # -------------------------------------------------------------------------
    def __init__(self, robotImagePath, envImagePath, root):

        self.root = root

        # Load the environment bitmap and convert it to black and white
        # what makes it easier to work with
        self.envImage = Image.open(envImagePath).convert("1")
        self.envArray = np.array(self.envImage)
        self.envPhoto = ImageTk.PhotoImage(self.envImage)

        # Load the robot bitmap and convert it to black and white also
        self.robotImage = Image.open(robotImagePath).convert("1")
        self.robotArray = np.array(self.robotImage)
        self.robotPhoto = ImageTk.PhotoImage(self.robotImage)
        self.robotRadius = round(0.5 * self.robotArray.shape[0])

        self._find_edges() # TODO: Refactor to return the edges and store them in self here
        #self._compute_c_space() # TODO: Refactor to return the c-space and store it in self here
        
        self.label = ttk.Label(root, image = self.envPhoto)

        self.currentPos = (0,0)
        self.isInitialize = False

    # -------------------------------------------------------------------------
    def drawAll (self,xCurrent,yCurrent,xInit=-1,yInit=-1,xGoal=-1,yGoal=-1):
        self.currentPos=xCurrent,yCurrent
        self.imageToDraw = self.envImage.copy()
        if xInit>-1: self.imageToDraw.paste(self.robotImage.copy(),(xInit,yInit))
        if xGoal>-1: self.imageToDraw.paste(self.robotImage.copy(),(xGoal,yGoal))
        self.imageToDraw.paste(self.robotImage.copy(),(self.currentPos[0],self.currentPos[1]))
        self.photoToDraw = ImageTk.PhotoImage(self.imageToDraw)
        self.label.configure(image=self.photoToDraw)
        self.label.image = self.photoToDraw
        self.label.pack(side = "bottom", fill = "both", expand = "yes")

    # -------------------------------------------------------------------------
    def _find_edges(self):
        
        self.robotEdgePixels = []
        
        # This is pretty straight forward for our simple case at hand.
        # A black pixel with both black and white neighbors is defined as an
        # edge pixel.
        # We keep it even simpler by just using 4 neighbors instead of 8, so
        # in round figures (what the robot actually is) we don't get _every_
        # pixel but those that are relevant for collision detection.
        for x in range(self.robotImage.size[0] - 1):
            for y in range(self.robotImage.size[1] - 1):
                pixel = self.robotImage.getpixel((x, y))
                if (not pixel):
                    # Pixel of the robot is black, so we now check if this
                    # is an edge pixel.
                    #
                    # n1 = neighbor top
                    # n2 = neighbor right
                    # n3 = neighbor bottom
                    # n4 = neighbor left
                    n1 = self.robotImage.getpixel((x, y-1))
                    n2 = self.robotImage.getpixel((x+1, y))
                    n3 = self.robotImage.getpixel((x, y+1))
                    n4 = self.robotImage.getpixel((x-1, y))
                    
                    nSum = n1 + n2 + n3 + n4
                    if (nSum > 0 and nSum < (4 * 255)):
                        self.robotEdgePixels.append((x, y))

                    # Possible optimizations:
                    #  - Remove the edges from the robot's "eye" - how?
                    #  - Special cases (although not relevant for the robot):
                    #     - if pixel is black and y = 1 -> edge (top)
                    #     - if pixel is black and y = height -> edge (bottom)
                    #     - if pixel is black and x = 1 -> edge (left)
                    #     - if pixel is black and x = width -> edge (right) 


    # -------------------------------------------------------------------------
    def compute_c_space(self):

        print("--- Computing configuration space...")
        print("     + c-space width: ", self.envArray.shape[1])
        print("     + c-space height: ", self.envArray.shape[0])

        self.config_space = np.empty((self.envArray.shape[0], self.envArray.shape[1]))
        self.config_space.fill(255)

        robo_offset_y = round(0.5 * self.robotArray.shape[0])
        robo_offset_x = round(0.5 * self.robotArray.shape[1])
        
        # TODO: For every width/100 pixels update some progress information on the screen
        for x in range(self.envArray.shape[1]):
            for y in range(self.envArray.shape[0]):
                if (not self.envArray[y, x]):
                    
                    # TODO: For solving motion planning problem with the
                    # Minkowski Sum for non symmetric robots, the robot 
                    # has to be mirrored to the origin
                    # Punktspiegelung um den Ursprung entspricht einer
                    # Rotation um 180 Grad um den Ursprung. 
                    # As origin, here the center of the circle was chosen,
                    # so the mirrored robot is identical to the original robot :)
                    
                    for roboPixel in self.robotEdgePixels: # no NumPy array, so x and y are in "normal" order!
                        cy = int(y - robo_offset_y + roboPixel[1])
                        cx = int(x - robo_offset_x + roboPixel[0])
                        
                        if (cx < 0 or cx >= self.envArray.shape[1] or cy < 0 or cy >= self.envArray.shape[0]):
                            continue
                        
                        self.config_space[cy, cx] = 0
                        
        print("--- FINISHED computing configuration space")

    # -------------------------------------------------------------------------
    def display_c_space(self):
        
        Image.fromarray(self.config_space.astype(np.uint8)).show(title="Configuration space")

    # -------------------------------------------------------------------------
    def compute_path_with_sPRM(self, radius, samples, workspace, configspace):
        
        sprm = sPRM(radius, samples, workspace, configspace)

    # -------------------------------------------------------------------------
    def is_in_collis(self,x,y):
        
        # x and y are the coords of the center pixel of the robot
        #print("--- is_in_collision(" + str(x) + "," + str(y) + ")")
        # Improved collision detection: do not iterate over every pixel of the
        # robot bitmap but only over the edge pixels. The collision detection
        # is the same as above (edge pixel over a black pixel (obstacle) in
        # the environment is a collision).
        for pixel in self.robotEdgePixels:
            
            # skip check for pixels of the robot that are outside of the environment
            # (if the robot is positioned at the very edge)
            if (x - self.robotRadius + pixel[0] > self.envImage.size[0] - 1 or
                y - self.robotRadius + pixel[1] > self.envImage.size[1] - 1):
                continue
            
            if (not self.envImage.getpixel( (x - self.robotRadius + pixel[0], y - self.robotRadius + pixel[1]) )):
                # Pixel of the env image is black, so we hit an obstacle here
                return True

        return False        
