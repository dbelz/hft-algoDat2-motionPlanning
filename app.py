import tkinter 
from tkinter import ttk, RIGHT, Canvas, BOTH, Scale, HORIZONTAL
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

    def display_c_space():
        controller.display_c_space()
    setDrawCSpaceButton = ttk.Button(page1, text = 'Display C-Space', command = display_c_space)
    setDrawCSpaceButton.pack(side=tkinter.RIGHT)
    
    def set_goal():
        controller.setCurrentPosAsGoal()
        slider['from_'] = 0
        slider['to_'] = len(configspace.solutionPath)-1

    setGoalButton = ttk.Button(page1, text = 'Set Goal',command = set_goal)
    setGoalButton.pack(side=tkinter.RIGHT)

    def set_init():
        controller.setCurrentPosAsInit()
    setInitButton = ttk.Button(page1, text = 'Set Init',command = set_init)
    setInitButton.pack(side=tkinter.RIGHT)


    slider.pack()

    root.mainloop()


if __name__ == "__main__":
    demo()
