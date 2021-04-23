from PIL import Image, ImageTk, ImageColor
from io import BytesIO
from tkinter import ttk, Canvas, NW
import os
from configspace import Configspace
import time

# https://stackoverflow.com/questions/18943387/how-to-analyse-bitmap-image-in-python-using-pil
# http://www.pythonclub.org/_media/modules/pil/pil.pdf

class Workspace:

    # -------------------------------------------------------------------------
    def __init__(self, robotImagePath, envImagePath, root):

        self.root = root

        # Load the environment bitmap and convert it to black and white
        # what makes it easier to work with
        self.envImage = Image.open(envImagePath).convert("1")
        self.envPhoto = ImageTk.PhotoImage(self.envImage)

        # Load the robot bitmap and convert it to black and white also
        self.robotImage = Image.open(robotImagePath).convert("1")
        self.robotPhoto = ImageTk.PhotoImage(self.robotImage)

        self.__findEdges()

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
    def __findEdges(self):
        
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
    def isInCollision(self,x,y):
        # x and y are the coords of the top left pixel of the robot
        #print("--- isInCollision(" + str(x) + "," + str(y) + ")")
        # Improved collision detection: do not iterate over every pixel of the
        # robot bitmap but only over the edge pixels. The collision detection
        # is the same as above (edge pixel over a black pixel (obstacle) in
        # the environment is a collision).
        for pixel in self.robotEdgePixels:
            if (not self.envImage.getpixel( (x + pixel[0], y + pixel[1]) )):
                # Pixel of the env image is black, so we hit an obstacle here
                return True

        return False
