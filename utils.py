from tkinter import ttk, messagebox
import sys
import numpy as np
import random


# -------------------------------------------------------------------------    
def setBackgroundColor(frame, color):
    style = ttk.Style()     # Create style
    style.configure("A.TFrame", background=color) # Set bg color
    frame.config(style='A.TFrame')    # Apply style to widget


# -------------------------------------------------------------------------    
def show_info(msg, debug=False):
    _show_message_dialog(msg, "INFO", debug)
    
def show_warning(msg, debug=False):
    _show_message_dialog(msg, "WARN", debug)
    
def _show_message_dialog(msg, level="INFO", debug=False):
    if debug: print("INFO: ", msg)
    messagebox.showinfo("Information", msg)


# -------------------------------------------------------------------------    
def calculate_distance(a, b):
    return ((b[0] - a[0])**2 + (b[1] - a[1])**2)**(0.5)


# -------------------------------------------------------------------------    
def encode_config(cfg):
    return "{},{}".format(cfg[0], cfg[1])


# -------------------------------------------------------------------------    
def decode_config(cfg_string):
    cfg = cfg_string.split(',')
    return (int(cfg[0]), int(cfg[1]))


# -------------------------------------------------------------------------    
def find_neighbors(workspace, config, vertex, graph, radius, debug=False, reverse=False):
    for possible_neighbor in vertex:
        dist = calculate_distance(config, possible_neighbor)
        if (dist < radius and dist > 0):
            if debug: print("sample: {}, poss_neighbor: {}, dist: {}".format(config, possible_neighbor, dist))
            
            if (is_edge_valid(workspace, config, possible_neighbor, dist)):
                if debug: print("Possible neighbor found: {}".format(possible_neighbor))
                if (reverse):
                    # For the goal configuration, we have to add the edge in reverse,
                    # otherwise no path can be found.
                    graph.add_edge(encode_config(possible_neighbor), encode_config(config), dist)
                else:
                    graph.add_edge(encode_config(config), encode_config(possible_neighbor), dist)
   
   
# -----------------------------------------------------------------------------
def find_nearest_neighbor(config, vertex):
    
    c_near = None
    min_dist = float("inf")
    
    for possible_neighbor in vertex:
        dist = calculate_distance(config, possible_neighbor)
        if (dist < min_dist and dist > 0):
            c_near = possible_neighbor
    
    return c_near


# -----------------------------------------------------------------------------
def get_cfg_between(c_near, c_rand, range):

    diff = np.array(c_rand) - np.array(c_near)
    length = np.linalg.norm(diff)
    diff = (diff / length) * min (range, length)

    return (int(c_near[0] + diff[0]), int(c_near[1] + diff[1]))


# -------------------------------------------------------------------------
def is_edge_valid(workspace, config, possible_neighbor, dist):
    # TODO: Calculate the needed amount of samples according to algo in lecture notes
    # To keep it simple we check every pixel of the edge.
    #steps = np.linspace(config, possible_neighbor, round(dist/5), endpoint=False)
    steps = np.linspace(config, possible_neighbor, int(dist), endpoint=False)
    for step in steps:
        if (is_in_collision(workspace, int(step[0]), int(step[1]))):
            return False
    return True
            
            
# -------------------------------------------------------------------------
def show_progress(progress):
    print("\r[{0:<50}] {1}%".format('#'*int(progress/2), progress), end="\r", flush=True)


# -------------------------------------------------------------------------
def is_in_collision(workspace, x, y):
    
    # x and y are the coords of the center pixel of the robot
    #print("--- is_in_collision(" + str(x) + "," + str(y) + ")")
    # Improved collision detection: do not iterate over every pixel of the
    # robot bitmap but only over the edge pixels. The collision detection
    # is the same as above (edge pixel over a black pixel (obstacle) in
    # the environment is a collision).
    for pixel in workspace.robotEdgePixels:
        
        # skip check for pixels of the robot that are outside of the environment
        # (if the robot is positioned at the very edge)
        if (x - workspace.robotRadius + pixel[0] > workspace.envImage.size[0] - 1 or
            y - workspace.robotRadius + pixel[1] > workspace.envImage.size[1] - 1):
            continue
        
        if (not workspace.envImage.getpixel( (x - workspace.robotRadius + pixel[0], y - workspace.robotRadius + pixel[1]) )):
            # Pixel of the env image is black, so we hit an obstacle here
            return True

    return False      