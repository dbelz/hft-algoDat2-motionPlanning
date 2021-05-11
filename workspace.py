from algorithms.RRT import RRT
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
        # TODO: Change to robot_height and robot_width to be more general applicable (not just to a circle robot)
        self.robotRadius = round(0.5 * self.robotArray.shape[0])

        self._find_edges() # TODO: Refactor to return the edges and store them in self here
        
        self.label = ttk.Label(root, image = self.envPhoto)

        self.currentPos = (0,0)
        self.isInitialize = False

    # -------------------------------------------------------------------------
    def drawAll (self, xCurrent, yCurrent, xInit=-1, yInit=-1, xGoal=-1, yGoal=-1):
        # self.currentPos has to hold the center of the robot,
        # not the top left corner of the robot image!
        self.currentPos = xCurrent, yCurrent
        self.imageToDraw = self.envImage.copy()
        if xInit>-1: self.imageToDraw.paste(
            self.robotImage.copy(),
            (xInit - round(0.5 * self.robotImage.width),
             yInit - round(0.5 * self.robotImage.height)))
        if xGoal>-1: self.imageToDraw.paste(
            self.robotImage.copy(),
            (xGoal - round(0.5 * self.robotImage.width),
             yGoal - round(0.5 * self.robotImage.height)))
        self.imageToDraw.paste(
            self.robotImage.copy(),
            (self.currentPos[0] - round(0.5 * self.robotImage.width),
             self.currentPos[1] - round(0.5 * self.robotImage.height)))
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
    def construct_roadmap_with_sPRM(self, radius, samples, workspace, configspace):
        
        self.sprm = sPRM(workspace, configspace)
        self.sprm.distribute_configuration_samples(samples)
        self.sprm.build_neighbor_graph(radius)
        
    # -------------------------------------------------------------------------
    def find_path_with_sprm(self, c_init, c_goal):
        return self.sprm.find_path(c_init, c_goal)
    
    # -------------------------------------------------------------------------
    def find_path_with_rrt(self, workspace, configspace, c_init, c_goal, range, iterations):
        self.rrt = RRT(workspace, configspace, c_init, c_goal, range, iterations)
        return self.rrt.find_path()

  
