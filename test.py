import sys
import numpy as np
from tkinter import ttk, Canvas, NW
from PIL import Image, ImageTk, ImageColor
from pprint import pprint as pp

#np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True,linewidth=np.nan,threshold=sys.maxsize)

x = np.arange(0, 64)
y = np.arange(0, 64)
arr = np.zeros((y.size, x.size))

cx = 12.
cy = 16.
r = 5.

# The two lines below could be merged, but I stored the mask
# for code clarity.
mask = (x[np.newaxis,:]-cx)**2 + (y[:,np.newaxis]-cy)**2 < r**2
pp(mask)

arr[mask] = 1
pp(arr)

cx = 17.
cy = 18.

mask = (x[np.newaxis,:]-cx)**2 + (y[:,np.newaxis]-cy)**2 < r**2
pp(mask)

arr[mask] = 1
pp(arr)

# envImage = Image.open("./resources/robot_BW_small.bmp").convert("1")
# envArray = np.array(envImage)

# arr[envArray] = 1
# pp(arr)v