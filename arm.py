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
        x0, y0 = self.to_screen(-self.RADIUS, 0)
        x1, y1 = self.to_screen(self.RADIUS, 0)
        self.canvas.create_oval(x0, y0 - self.RADIUS, x1, y1 + self.RADIUS, outline="blue") #Blue circle

    def draw_arm(self,angles):
        theta1, theta2, theta3 = angles
        x0, y0 = 0, 0
        x1 = x0 + self.L1 * math.cos(theta1)
        y1 = y0 + self.L1 * math.sin(theta1)
        x2 = x1 + self.L2 * math.cos(theta1 + theta2)
        y2 = y1 + self.L2 * math.sin(theta1 + theta2)
        x3 = x2 + self.L3 * math.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.L3 * math.sin(theta1 + theta2 + theta3)

        self.canvas.create_line(*self.to_screen(x0, y0), *self.to_screen(x1, y1), width=4, fill="green")
        self.canvas.create_line(*self.to_screen(x1, y1), *self.to_screen(x2, y2), width=4, fill="green")
        self.canvas.create_line(*self.to_screen(x2, y2), *self.to_screen(x3, y3), width=4, fill="green")

    def solve_ik(self,target_x, target_y):
        dx, dy = target_x, target_y
        dist = math.hypot(dx, dy)
        if dist > (self.L1 + self.L2 + self.L3):
            return None
        tx = dx - self.L3 * (dx / dist)
        ty = dy - self.L3 * (dy / dist)
        d = math.hypot(tx, ty)

        cos_angle2 = (d ** 2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        if abs(cos_angle2) > 1:
            return None
        angle2 = math.acos(cos_angle2)
        k1 = self.L1 + self.L2 * math.cos(angle2)
        k2 = self.L2 * math.sin(angle2)
        angle1 = math.atan2(ty, tx) - math.atan2(k2, k1)
        angle3 = math.atan2(dy - (self.L1 * math.sin(angle1) + self.L2 * math.sin(angle1 + angle2)),
                            dx - (self.L1 * math.cos(angle1) + self.L2 * math.cos(angle1 + angle2))) - (angle1 + angle2)
        return angle1, angle2, angle3

    # Redraw everything on click
    def on_click(self,event):
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_circle()
        world_x = event.x - self.CENTER_X
        world_y = self.CENTER_Y - event.y
        if math.hypot(world_x, world_y) <= self.RADIUS:
            angles = self.solve_ik(world_x, world_y)
            if angles:
                self.draw_arm(angles)