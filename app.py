import tkinter 
from tkinter import ttk, RIGHT, Canvas, BOTH, Scale, HORIZONTAL, END, messagebox
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

def demo():
    root = tkinter.Tk()
    root.title("Motion Planning")
    universal_height = 1000

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
    def callback(event):
        print ("clicked at", event.x, event.y)
        controller.drawMouseOffSet(event.x, event.y)
        if controller.isInCollision(event.x, event.y): setBackgroundColor(page1,"red")
        else: setBackgroundColor(page1,"green")

    workspace.label.bind("<Button-1>", callback)

    def moveRobotOnPath(val):
        if controller.isAllInitialized():
            controller.setSolutionPathOnCurrentPos(int(val))
            controller.drawCurrentPos()
            if controller.isInCollision(): setBackgroundColor(page1,"red")
            else: setBackgroundColor(page1,"green")

    slider = Scale(page1, from_=0, to=200, orient=HORIZONTAL, command=moveRobotOnPath)
    slider.config(length=600)


    
#    def display_c_space():
#        controller.display_c_space()
#    display_c_space_btn = ttk.Button(page1, text = 'Display C-Space', command = display_c_space)
#    display_c_space_btn.pack(side=tkinter.RIGHT)
    
    def set_goal():

        controller.setCurrentPosAsGoal()
        if (not controller.isAllInitialized()):
            messagebox.showerror("Initialization error", "Set an init state first!")
            return
        
        controller.find_path()
        
        slider['from_'] = 0
        slider['to_'] = len(configspace.solutionPath)-1

    setGoalButton = ttk.Button(page1, text = 'Set Goal',command = set_goal)
    setGoalButton.pack(side=tkinter.RIGHT)

    def set_init():
        controller.setCurrentPosAsInit()
    setInitButton = ttk.Button(page1, text = 'Set Init',command = set_init)
    setInitButton.pack(side=tkinter.RIGHT)

    # TODO: Keep in mind: We might need a drop-down to choose the algorithm later
    
    def construct_roadmap_with_sPRM():
        
        try:
            samples = int(config_samples_entry.get())
            radius = int(radius_entry.get())
        except:
            messagebox.showerror("Input error", "Input for radius or number of samples missing or invalid!")
            return
            
        controller.construct_roadmap_with_sPRM(radius, samples)
        
        messagebox.showinfo("Roadmap constructed", "Roadmap is constructed, you can now choose an init a goal state for the robot...")
                
    sprm_btn = ttk.Button(page1, text = 'sPRM', command = construct_roadmap_with_sPRM)
    sprm_btn.pack(side=tkinter.RIGHT)
    
    # default values of r=50 and s=10000 seem to be a good idea
    config_samples_entry = ttk.Entry(page1, width=6)
    config_samples_entry.insert(END, "10000")
    config_samples_entry.pack(side=tkinter.RIGHT)
    
    config_samples_lbl = ttk.Label(page1, text="samples")
    config_samples_lbl.pack(side=tkinter.RIGHT, padx=5)
    
    radius_entry = ttk.Entry(page1, width=3)
    radius_entry.insert(END, "50")
    radius_entry.pack(side=tkinter.RIGHT)
    
    radius_entry_lbl = ttk.Label(page1, text="radius")
    radius_entry_lbl.pack(side=tkinter.RIGHT, padx=5)

    slider.pack()

    root.mainloop()


if __name__ == "__main__":
    demo()
