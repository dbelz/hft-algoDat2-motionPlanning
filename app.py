import sys
import tkinter 
from tkinter import Label, ttk, RIGHT, Canvas, BOTH, Scale, HORIZONTAL, END, messagebox
from workspace import Workspace 
from configspace import Configspace
from controller import  Controller
from PIL import ImageTk, Image
import os
from utils import setBackgroundColor

"""
--> https://pep8.org/
--> https://docs.python-guide.org/writing/style/
--> https://github.com/hblanks/zen-of-python-by-example/blob/master/pep20_by_example.py


The Zen of Python, by Tim Peters
---------------------------------
Beautiful is better than ugly.
Explicit is better than implicit.
Simple is better than complex.
Complex is better than complicated.
Flat is better than nested.
Sparse is better than dense.
Readability counts.
Special cases aren't special enough to break the rules.
Although practicality beats purity.
Errors should never pass silently.
Unless explicitly silenced.
In the face of ambiguity, refuse the temptation to guess.
There should be one-- and preferably only one --obvious way to do it.
Although that way may not be obvious at first unless you're Dutch.
Now is better than never.
Although never is often better than *right* now.
If the implementation is hard to explain, it's a bad idea.
If the implementation is easy to explain, it may be a good idea.
Namespaces are one honking great idea -- let's do more of those!
"""

def bye_bye(self):
    sys.exit()

def demo():
    root = tkinter.Tk()
    root.title("Motion Planning")
    universal_height = 1000

    root.bind('<Control-q>', bye_bye)

    nb = ttk.Notebook(root)
    page1 = ttk.Frame(nb, width= 1080,height = universal_height)
    page2 = ttk.Frame(nb,width = 1080,height = universal_height)

    nb.add(page1, text='Workspace')
    nb.add(page2, text='Configspace')
    nb.grid(column=0)
 
    workspace = Workspace("./resources/robot_BW_small.bmp", "./resources/Room_BW_small.bmp", page1)
    configspace = Configspace(page2)
    controller = Controller(workspace,configspace)

    workspace.drawAll(workspace.currentPos[0],workspace.currentPos[1])
    
    # -------------------------------------------------------------------------
    def callback(event):
        print ("clicked at", event.x, event.y)
        controller.drawMouseOffSet(event.x, event.y)
        if controller.isInCollision(event.x, event.y): setBackgroundColor(col_stat_lbl,"red")
        else: setBackgroundColor(col_stat_lbl,"green")

    workspace.label.bind("<Button-1>", callback)

    # -------------------------------------------------------------------------
#    def display_c_space():
#        controller.display_c_space()
#    display_c_space_btn = ttk.Button(page1, text = 'Display C-Space', command = display_c_space)
#    display_c_space_btn.pack(side=tkinter.RIGHT)
    

    # -------------------------------------------------------------------------
    # Start and goal configuration
    # -------------------------------------------------------------------------
    
    position_frame = ttk.LabelFrame(page1, text="Position")
    position_frame.pack(side=tkinter.LEFT, padx=2)

    # --- Start configuration
    def set_init():
        controller.setCurrentPosAsInit()
    setInitButton = ttk.Button(position_frame, text = 'Set Init',command = set_init)
    setInitButton.pack(side=tkinter.LEFT)

    # --- Goal configuration
    def set_goal():
        controller.setCurrentPosAsGoal()
        if (not controller.isAllInitialized()):
            messagebox.showerror("Initialization error", "Set an init state first!")
            return
    setGoalButton = ttk.Button(position_frame, text = 'Set Goal',command = set_goal)
    setGoalButton.pack(side=tkinter.LEFT)

    # -------------------------------------------------------------------------
    # sPRM
    # -------------------------------------------------------------------------
    # default values of r=50 and s=10000 seem to be a good idea

    sprm_frame = ttk.LabelFrame(page1, text="sPRM")
    sprm_frame.pack(side=tkinter.LEFT, padx=2)
    
    # --- Number of configuration samples
    config_samples_lbl = ttk.Label(sprm_frame, text="samples")
    config_samples_lbl.pack(side=tkinter.LEFT, padx=5)
    
    config_samples_entry = ttk.Entry(sprm_frame, width=6)
    config_samples_entry.insert(END, "10000")
    config_samples_entry.pack(side=tkinter.LEFT)

    # --- RADIUS    
    radius_entry_lbl = ttk.Label(sprm_frame, text="radius")
    radius_entry_lbl.pack(side=tkinter.LEFT, padx=5)

    radius_entry = ttk.Entry(sprm_frame, width=3)
    radius_entry.insert(END, "50")
    radius_entry.pack(side=tkinter.LEFT)

    # --- Button to create the roadmap
    def construct_roadmap_with_sPRM():
        
        try:
            samples = int(config_samples_entry.get())
            radius = int(radius_entry.get())
        except:
            messagebox.showerror("Input error", "Input for radius or number of samples missing or invalid!")
            return
            
        controller.construct_roadmap_with_sPRM(radius, samples)
        
        messagebox.showinfo("Roadmap constructed", "Roadmap constructed, choose init and goal state for the robot")

    sprm_roadmap_btn = ttk.Button(sprm_frame, text = 'Roadmap', command = construct_roadmap_with_sPRM)
    sprm_roadmap_btn.pack(side=tkinter.LEFT)

    # --- Button to find path
    def find_path_with_sprm():

        controller.find_path_with_sprm()
        
        slider['from_'] = 0
        slider['to_'] = len(configspace.solutionPath)-1

    sprm_path_btn = ttk.Button(sprm_frame, text="Path", command=find_path_with_sprm)
    sprm_path_btn.pack(side=tkinter.LEFT)

    # -------------------------------------------------------------------------
    # RRT
    # -------------------------------------------------------------------------

    rrt_frame = ttk.LabelFrame(page1, text="RRT")
    rrt_frame.pack(side=tkinter.LEFT, padx=2)

    # --- Number of iterations
    iterations_entry_lbl = ttk.Label(rrt_frame, text="iterations")
    iterations_entry_lbl.pack(side=tkinter.LEFT, padx=5)
    
    iterations_entry = ttk.Entry(rrt_frame, width=6)
    iterations_entry.insert(END, "10000")
    iterations_entry.pack(side=tkinter.LEFT)

    # --- Range to c_new    
    range_entry_lbl = ttk.Label(rrt_frame, text="range")
    range_entry_lbl.pack(side=tkinter.LEFT, padx=5)

    range_entry = ttk.Entry(rrt_frame, width=3)
    range_entry.insert(END, "50")
    range_entry.pack(side=tkinter.LEFT)

    # --- Button to find path
    def find_path_with_rrt():

        if (not controller.isAllInitialized()):
            messagebox.showerror("Initialization error", "Set an init and goal state first!")
            return

        try:
            iterations = int(iterations_entry.get())
            range = int(range_entry.get())
        except:
            messagebox.showerror("Input error", "Input for radius or number of samples missing or invalid!")
            return

        controller.find_path_with_rrt(range, iterations)

        slider['from_'] = 0
        slider['to_'] = len(configspace.solutionPath)-1
        
    rrt_btn = ttk.Button(rrt_frame, text='Path', command = find_path_with_rrt)
    rrt_btn.pack(side=tkinter.LEFT)

    # -------------------------------------------------------------------------
    # Slider to move robot on the solution path
    # -------------------------------------------------------------------------
    def moveRobotOnPath(val):
        if controller.isAllInitialized():
            controller.setSolutionPathOnCurrentPos(int(val))
            controller.drawCurrentPos()
            if controller.isInCollision(): setBackgroundColor(col_stat_lbl,"red")
            else: setBackgroundColor(col_stat_lbl,"green")

    slider = Scale(page1, from_=0, to=200, orient=HORIZONTAL, command=moveRobotOnPath)
    slider.config(length=400)
    slider.pack(side=tkinter.LEFT, padx=2)

    # -------------------------------------------------------------------------
    # Collision status
    # -------------------------------------------------------------------------

    col_stat_frame = ttk.LabelFrame(page1, text="Collision")
    col_stat_frame.pack(side=tkinter.LEFT, padx=2)

    col_stat_lbl = Label(col_stat_frame, width=10)
    col_stat_lbl.pack(side=tkinter.LEFT)

    # -------------------------------------------------------------------------
    root.mainloop()


if __name__ == "__main__":
    demo()
