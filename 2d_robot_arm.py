'''
Using tkinter to create the interactive GUI as it is easy to operate & distribute, does not
require OpenGL libgll backend drivers or such, and is easier to customize  
'''
import tkinter as tk
import math
from arm import RobotArmGUI

# Constants for the grid
'''
We'll form a grid exactly as given in the diagram. For this, the radius is 300.
Let's plot grid lines from -300 to 300 on x axis, hence x axis has length 600
Y axis has length 300 (0 -> 300)
'''
WIDTH, HEIGHT = 600, 300 
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT #This helps us plot the coordinate frame origin exactly at bottom of grid and in the centre 
L1, L2, L3 = 150, 100, 50  #Lengths of robot links
RADIUS = 300

# Creating window and canvas, the object which is used to control the GUI
root = tk.Tk()
root.title("2D Robot Arm Simulation and Analysis")
canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white") #Setting colours as per illustration in PDF
canvas.pack()

''' 
Instantiating object of class RobotArmGUI, which has functions 
for manipulating the robot arm within the GUI 
'''
gui = RobotArmGUI(canvas,CENTER_X,CENTER_Y,WIDTH,HEIGHT,RADIUS,L1,L2,L3)
# Bind mouse event
canvas.bind("<Button-1>", gui.on_click)
# Initial draw
gui.draw_grid()
gui.draw_circle()
gui.draw_arm((0, 0, 0))
root.mainloop()


