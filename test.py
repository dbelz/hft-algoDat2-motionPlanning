import sys
import numpy as np
import random
from tkinter import ttk, Canvas, NW
from PIL import Image, ImageTk, ImageColor
from pprint import pprint as pp

#np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True,linewidth=np.nan,threshold=sys.maxsize)

#x = np.arange(0, 250)
#y = np.arange(0, 250)
#arr = np.zeros((y.size, x.size))

# cx = 12.
# cy = 16.
# r = 5.

# # The two lines below could be merged, but I stored the mask
# # for code clarity.
# mask = (x[np.newaxis,:]-cx)**2 + (y[:,np.newaxis]-cy)**2 < r**2
# pp(mask)

# arr[mask] = 1
# pp(arr)

# cx = 17.
# cy = 18.

# mask = (x[np.newaxis,:]-cx)**2 + (y[:,np.newaxis]-cy)**2 < r**2
# pp(mask)

# arr[mask] = 1
# pp(arr)

# roboImage = Image.open("./resources/robot_BW_small.bmp").convert("1")
# roboArray = np.array(roboImage)

# mask = np.ones(shape=arr.shape[0:2], dtype="bool")

# roboX = 50
# roboY = 50

# print(mask.dtype, mask.min(), mask.max())


# arr[mask] = 1

# Image.fromarray(arr.astype(np.uint8)).show(title="Mask test")

x1 = int(random.uniform(0, 1500))
y1 = int(random.uniform(0, 1000))

dist = random.gauss(0, 40)

x2 = int(x1 + dist)
y2 = int(y1 + dist)

print("1({},{}), 2({},{})".format(x1, y1, x2, y2))