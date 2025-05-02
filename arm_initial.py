import os
import math

class RobotArmGUI:
    '''
    A class for creating and manipulating (via mouse clicks) a 2D Robot Arm simulation. 
    Requires constructor arguments, in order, as :
    1.The canvas object used to initialise tkinter
    2. X and Y coordinates of center of GUI (for frame conversion from world frame to GUI frame)
    3. WIDTH and HEIGHT of GUI window
    4. RADIUS of circle
    5. Robot link lengths. Currently only supports a 3 link robot.
    '''
    def __init__(self,canvas,CENTER_X,CENTER_Y,WIDTH,HEIGHT,RADIUS,L1,L2,L3):
        self.CENTER_X = CENTER_X
        self.CENTER_Y = CENTER_Y
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.RADIUS = RADIUS
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.canvas = canvas
        
    def to_screen(self,x, y):
        '''
        The origin on the grid is present at the bottom of the window and at the centre of the horizontal axis.
        This origin must not be confused with the base frame of robot, which is at point A. 
        We must convert all point locations with respect to point A for correct calculation.
        Also, the tkinter window records points on click from top left corner as origin. 
        This function converts those coordinates to frame with origin at point A (robot base frame)
        '''
        return self.CENTER_X + x, self.CENTER_Y - y

    #Drawing circle and grid according to corresponding dimensions provided in constructor
    def draw_grid(self):
        #We define robot's 'base frame' as having origin as point A (illustrated in report).
        for x in range(-self.WIDTH // 2 , self.WIDTH // 2 + 1, 20): #Draws gridlines across entire x axis
            sx1, sy1 = self.to_screen(x, -self.HEIGHT)
            sx2, sy2 = self.to_screen(x, self.HEIGHT)
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill="#cccccc") #Using color code for gray grid-lines
        for y in range(-self.HEIGHT, self.HEIGHT + 1, 20): #Draws gridlines across entire y axis
            sx1, sy1 = self.to_screen(-self.WIDTH // 2, y)
            sx2, sy2 = self.to_screen(self.WIDTH // 2, y)
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill="#cccccc")

    #Drawing a circle
    def draw_circle(self):
        '''
        This will draw a semi-circular arc which cover the entire X-axis. 
        '''
        x0, y0 = self.to_screen(-self.RADIUS, 0)
        x1, y1 = self.to_screen(self.RADIUS, 0)
        self.canvas.create_oval(x0, y0 - self.RADIUS, x1, y1 + self.RADIUS, outline="blue") #Blue circle

    def draw_arm(self,angles):
        '''
        Let's define three angles. theta1 is the angle between segment AB and X-axis. theta2 is the angle between frame at point B and segment 
        BC. theta3 is the angle between frame at point C and segment CD.
        In this case, given these angles and link lengths, drawing the arm requires forward kinematics, which tells us the exact coordinates
        of each frame with respect to the previous frame, allowing us to draw arm segments.
        We can calculate forward kinematics using regular coordinates and conver to screen coordinates.
        '''
        theta1, theta2, theta3 = angles #Form a vector of angles
        x0, y0 = 0, 0 #Base Frame location
        x1 = x0 + self.L1 * math.cos(theta1) # x1,y1 = L1cos(theta1),L1sin(theta1)
        y1 = y0 + self.L1 * math.sin(theta1) 
        x2 = x1 + self.L2 * math.cos(theta1 + theta2) #Point C coordinates wrt base frame
        y2 = y1 + self.L2 * math.sin(theta1 + theta2) 
        x3 = x2 + self.L3 * math.cos(theta1 + theta2 + theta3) #Using forward kinematics to again find point D wrt base frame
        y3 = y2 + self.L3 * math.sin(theta1 + theta2 + theta3)

        self.canvas.create_line(*self.to_screen(x0, y0), *self.to_screen(x1, y1), width=4, fill="green") #unpack the points and convert to screen coordinates before drawing segments
        self.canvas.create_line(*self.to_screen(x1, y1), *self.to_screen(x2, y2), width=4, fill="green") 
        self.canvas.create_line(*self.to_screen(x2, y2), *self.to_screen(x3, y3), width=4, fill="green")

    def solve_ik(self,target_x, target_y):
        '''
        Given coordinate location of end effector D, we need to find the corresponding angles vector.
        Here, we consider location of end effector only (its frame origin coordinates) and not orientation of frame.
        which puts D in that location. This is inverse kinematics. Before solving for it, we eliminate some impossible solutions.
        Robot manipulator reachability is restricted by total link length. Here this is L1+L2+L3 = 300 (circle inner region is workspace).
        Also eliminate cases where configuration has singular Jacobian or includes collision of links by checking angles obtained at each step.
        '''
        dx, dy = target_x, target_y
        dist = math.hypot(dx, dy) #Calculate query point distance
        if dist > (self.L1 + self.L2 + self.L3): #Check reachability
            print("Unreachable. Terminating")
            return None
        
        # We will convert the problem to simple 2 link manipulator by finding coordinates of point C and using that to determine theta1 and theta2 first
        tx = dx - self.L3 * (dx / dist)
        ty = dy - self.L3 * (dy / dist)
        d = math.hypot(tx, ty)
        
        # Using cosine rule
        cos_angle2 = (d ** 2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        if abs(cos_angle2) > 1: #Check if angle has been correctly obtained
            return None
        angle2 = math.acos(cos_angle2)   #theta2 is cos inverse of the angle we obtained via cosine rule
        k1 = self.L1 + self.L2 * math.cos(angle2)
        k2 = self.L2 * math.sin(angle2)
        angle1 = math.atan2(ty, tx) - math.atan2(k2, k1)
        #Solving for theta3 using theta2 and theta1
        angle3 = math.atan2(dy - (self.L1 * math.sin(angle1) + self.L2 * math.sin(angle1 + angle2)),
                            dx - (self.L1 * math.cos(angle1) + self.L2 * math.cos(angle1 + angle2))) - (angle1 + angle2)
        return angle1, angle2, angle3

    # Redraw the arm on clicking
    def on_click(self,event):
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_circle()
        world_x = event.x - self.CENTER_X #Converting to regular coordinates
        world_y = self.CENTER_Y - event.y
        if math.hypot(world_x, world_y) <= self.RADIUS: #Check if desired point is within robot workspace
            angles = self.solve_ik(world_x, world_y)
            if angles: #Check if all angles in inverse kinematics are feasible or not
                self.draw_arm(angles)