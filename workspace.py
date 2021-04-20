import numpy as np
from PIL import Image, ImageTk, ImageColor
from io import BytesIO
from tkinter import ttk, Canvas, NW
import os
from configspace import Configspace
import time
import cv2

# https://stackoverflow.com/questions/18943387/how-to-analyse-bitmap-image-in-python-using-pil
# http://www.pythonclub.org/_media/modules/pil/pil.pdf

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

        # For the improved version load the robot bitmap using OpenCV and
        # let it find the contours of the robot.
        # (This code is from the official OpenCV tutorial)
        robotCVImg = cv2.imread(robotImagePath)
        gray = cv2.cvtColor(robotCVImg, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Store the pixels of the contour in tuples which saves us some runtime
        # below when iterating over the list on each collission detection.
        # Contour 0 is the frame of the robot bitmap, contour 1 is the robot "circle"
        # and contour 2 is the small dot on the robot.
        self.robotContourPixels = []
        for c in contours[1]:
            self.robotContourPixels.append((int(c[0][0]), int(c[0][1])))

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
    def isInCollision(self,x,y):
        # x and y are the coords of the top left pixel of the robot
        print("--- isInCollision(" + str(x) + "," + str(y) + ")")

        # To compare the runtime performance we run both collission detections
        # after one another, but only the result of the improved CD is used.
        self.__isInCollisionSequential(x, y)
        rv = self.__isInCollisionContour(x, y)

        return rv

    # -------------------------------------------------------------------------
    def __isInCollisionSequential(self, x, y):

        startTimeCheckSequential = time.time()

        # Iterate over each pixel of the robot. Skip it if it is white.
        # A black pixel means that this is a pixel of the robot and therefore
        # we have to check for a possible collission. This is done by mapping
        # the current robot pixel to the corresponding environment pixel. If
        # both pixels are black, we have a collission. If the environment pixel
        # is white, we are collission free (for the given pixel).
        for robotX in range(self.robotImage.size[0]):
            for robotY in range(self.robotImage.size[1]):
                pixel = self.robotImage.getpixel((robotX, robotY))
                if (not pixel):
                    # Pixel of the robot is black, so we have to check for obstacles
                    if (not self.envImage.getpixel((x + robotX, y + robotY))):
                        # Pixel of the env image is also black, so we hit an obstacle here
                        print("Duration of sequential check: ", str((time.time() - startTimeCheckSequential) * 1000), "ms")
                        return True

        print("Duration of sequential check: ", str((time.time() - startTimeCheckSequential) * 1000), "ms")
        return False

    # -------------------------------------------------------------------------
    def __isInCollisionContour(self, x, y):

        startTime = time.time()

        # Improved collision detection: do not iterate over every pixel of the
        # robot bitmap but only over the contour pixels. The collision detection
        # is the same as above (contour pixel over a black pixel (obstacle) in
        # the environment is a collision).
        for pixel in self.robotContourPixels:
            if (not self.envImage.getpixel( (x + pixel[0], y + pixel[1]) )):
                # Pixel of the env image is black, so we hit an obstacle here
                print("Duration of contour check: ", str((time.time() - startTime) * 1000), "ms")
                return True

        print("Duration of contour check: ", str((time.time() - startTime) * 1000), "ms")
        return False
