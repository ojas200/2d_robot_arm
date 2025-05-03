import os
import math
import numpy as np
import time

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
        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 0
        self.current_angles = np.zeros(3)
        self.current_position = np.zeros(2)
        
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
        self.current_position = np.array([x3,y3])

    def solve_ik(self,target_x, target_y):
        '''
        Given coordinate location of end effector D, we need to find the corresponding angles vector.
        Here, we consider location of end effector only (its frame origin coordinates) and not orientation of frame.
        which puts D in that location. This is inverse kinematics. Before solving for it, we eliminate some impossible solutions.
        Robot manipulator reachability is restricted by total link length. Here this is L1+L2+L3 = 300 (circle inner region is workspace).
        Also eliminate cases where configuration has singular Jacobian or includes collision of links by checking angles obtained at each step.

        Updated: now solve_ik returns all solutions, so that we can minimize max angle change at each time step.
        '''
        solutions = [] #Let us update solve_ik to return all possible solutions
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
        
        #Now, we have multiple solutions for angle2. Which is +- angle2
        ang2_options = [math.acos(cos_angle2),-math.acos(cos_angle2)]  

        for angle2 in ang2_options: 
            k1 = self.L1 + self.L2 * math.cos(angle2)
            k2 = self.L2 * math.sin(angle2)
            angle1 = math.atan2(ty, tx) - math.atan2(k2, k1)
            #Solving for theta3 using theta2 and theta1
            angle3 = math.atan2(dy - (self.L1 * math.sin(angle1) + self.L2 * math.sin(angle1 + angle2)),
                                dx - (self.L1 * math.cos(angle1) + self.L2 * math.cos(angle1 + angle2))) - (angle1 + angle2)
            solutions.append((angle1,angle2,angle3))

        return solutions
    
    def forward_kinematics(self,soln):
        '''
        Updated this function to take in a set of joint angle solutions in radians.
        Use this function to find world coordinate locations of each point
        Returns x,y tuple in order for B,C,D
        '''
        angle1, angle2, angle3 = soln
        #Coordinates of base frame (origin -> A)
        x0, y0 = 0, 0
        #Coordinates of point B
        x1 = x0 + self.L1 * math.cos(angle1)
        y1 = y0 + self.L1 * math.sin(angle1)
        #Coordinates of point C
        x2 = x1 + self.L2 * math.cos(angle1 + angle2)
        y2 = y1 + self.L2 * math.sin(angle1 + angle2)
        #Coordinates of point D
        x3 = x2 + self.L3 * math.cos(angle1 + angle2 + angle3)
        y3 = y2 + self.L3 * math.sin(angle1 + angle2 + angle3)
        return x1,y1,x2,y2,x3,y3
    
    def within_grid(self, x1, y1, x2, y2, x3, y3):
        '''
        Checks if all the points B, C and D lie within our figure. We use the output
        of forward kinematics to check if the point's coordinates lie in a specific range (grid dimensions).
        '''
        def in_bounds(x, y):
            return -self.WIDTH // 2 <= x <= self.WIDTH // 2 and 0 <= y <= self.HEIGHT
        return all(in_bounds(*pt) for pt in [(x1, y1), (x2, y2), (x3, y3)])

    def on_click_dp(self, event):
        '''
        The main action loop. Defines the actions post receiving a new click.
        Modified this to include dynamic programming solution minimum max delta(angles) over all paths.
        '''
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_circle()

        #Converting click coordinates to base frame coordinates
        world_x = event.x - self.CENTER_X
        world_y = self.CENTER_Y - event.y

        #Checking reachability before calculating inverse kinematics
        if math.hypot(world_x, world_y) <= self.RADIUS:
            
            destination = np.array([world_x, world_y])
            points = np.linspace(self.current_position, destination, num=21, endpoint=True)

            print("\nTable:\n")
            print(f"{'Dot#':>4} | {'B pos':>18} | {'A-B ∡':>7} | {'ΔA-B ∡':>7} | "
                f"{'C pos':>18} | {'B-C ∡':>7} | {'ΔB-C ∡':>7} | "
                f"{'D pos':>18} | {'C-D ∡':>7} | {'ΔC-D ∡':>7} | {'Max Δ ∡':>7}")

            #At each point, we will store the best angles (which we use to draw the arm) for calculation of delta (angle)
            prev_angles = None

            #Store all solutions for each of the 21 points
            all_soln = []
            #Save corresponding points in case a point is at singularity
            valid_pts = [] 
            for i, pt in enumerate(points):
                soln = self.solve_ik(pt[0], pt[1])
                if not soln:
                    print(f"Point {i:>3} lies at singular configuration")
                    return

                formatted = []
                for sol in soln:
                    angle1, angle2, angle3 = np.degrees(sol)
                    theta = np.array([90 - angle1, -angle2, -angle3])
                    formatted.append(theta)
                
                valid_pts.append(pt)
                all_soln.append(formatted)

            #Store all solutions    
            n = len(all_soln)

            #Dynamic Programming to minimize max delta over all paths
            dp = [{} for _ in range(n)]  # dp[i][j] = (max_delta_so_far, prev_choice_index)

            for j, theta in enumerate(all_soln[0]):
                dp[0][j] = (0, -1)  # No delta for the first one

            for i in range(1, n):
                for j, curr_theta in enumerate(all_soln[i]):
                    min_entry = (float('inf'), None)
                    for k, prev_theta in enumerate(all_soln[i - 1]):
                        delta = np.abs(curr_theta - prev_theta)
                        max_delta = max(np.max(delta), dp[i - 1][k][0])
                        if max_delta < min_entry[0]:
                            min_entry = (max_delta, k)
                    dp[i][j] = min_entry

            # Step 3: Backtrack to find optimal path
            min_last_delta = float('inf')
            last_index = None
            for j, (max_delta, _) in dp[-1].items():
                if max_delta < min_last_delta:
                    min_last_delta = max_delta
                    last_index = j

            best_path = [last_index]
            for i in reversed(range(1, n)):
                last_index = dp[i][last_index][1]
                best_path.append(last_index)
            best_path.reverse()


            for i, pt in enumerate(valid_pts): 
                #We find the theta which minimizes max delta over entire path
                #Converting to degrees and storing with correct format as required
                theta = all_soln[i][best_path[i]]        
                angle1, angle2, angle3 = np.array([90-theta[0], -theta[1], -theta[2]])

                # Forward kinematics for points B, C, D
                x1,y1,x2,y2,x3,y3 = self.forward_kinematics(np.radians([angle1,angle2,angle3]))

                if not self.within_grid(x1,y1,x2,y2,x3,y3):
                    print("In this solution, one or more joints lie outside grid. Skipping")
                    continue

                #For the best solution in this case, find delta and arrange angles in the table to satisfy given pre-condition
                delta_angles = np.abs(theta - prev_angles) if prev_angles is not None else np.zeros(3)
                max_delta = np.max(delta_angles)

                #Saving for next step
                prev_angles = theta

                #Print table
                print(f"{i+1:>4} | ({x1:6.1f}, {y1:6.1f}) | {theta[0]:7.2f} | {delta_angles[0]:7.2f} | "
                    f"({x2:6.1f}, {y2:6.1f}) | {theta[1]:7.2f} | {delta_angles[1]:7.2f} | "
                    f"({x3:6.1f}, {y3:6.1f}) | {theta[2]:7.2f} | {delta_angles[2]:7.2f} | {max_delta:7.2f}")
                
                #Clear canvas to display arm position afresh
                self.canvas.delete("all")
                self.draw_grid()
                self.draw_circle()
                self.draw_arm((angle1,angle2,angle3))
                
                #By default, tkinter will not show the figure at each intermittent step. We want to see all intermediate arm steps.
                #Hence, force canvas update.
                self.canvas.update()
                time.sleep(0.25)
        
        #Check if arm has actually reached the point
        print("Distance between destination and final end effector position:", math.sqrt((x3-world_x)**2 + (y3-world_y)**2))
        self.current_position = destination

    def on_click(self, event):
        '''
        The main action loop. Defines the actions post receiving a new click.
        '''
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_circle()

        #Converting click coordinates to base frame coordinates
        world_x = event.x - self.CENTER_X
        world_y = self.CENTER_Y - event.y

        if math.hypot(world_x, world_y) <= self.RADIUS:
            print("Not reachable. Re-input")
            #Checking reachability before calculating inverse kinematics
            destination = np.array([world_x, world_y])
            points = np.linspace(self.current_position, destination, num=21, endpoint=True)

            print("\nTable:\n")
            print(f"{'Dot#':>4} | {'B pos':>18} | {'A-B ∡':>7} | {'ΔA-B ∡':>7} | "
                f"{'C pos':>18} | {'B-C ∡':>7} | {'ΔB-C ∡':>7} | "
                f"{'D pos':>18} | {'C-D ∡':>7} | {'ΔC-D ∡':>7} | {'Max Δ ∡':>7}")

            #At each point, we will store the best angles (which we use to draw the arm) for calculation of delta (angle)
            prev_angles = None

            #Store all solutions for each of the 21 points
            for i, pt in enumerate(points):
                soln = self.solve_ik(pt[0], pt[1])
                if not soln:
                    print(f"Point {i:>3} lies at singular configuration")
                    continue
                
                best_soln = []
                min_max_delta = float('inf')

                for sol in soln: 
                    #We will calculate the best solution by selecting the solution which minimizes max delta at each time step
                    #Converting to degrees and storing with correct format as required        
                    angle1, angle2, angle3 = np.degrees(sol)
                    theta = np.array([90-angle1, -angle2, -angle3])

                    delta_angles = np.abs(theta - prev_angles) if prev_angles is not None else np.zeros(3)
                    max_delta = np.max(delta_angles)
                    if max_delta < min_max_delta:
                        best_soln = sol
                        min_max_delta = max_delta

                # Forward kinematics for points B, C, D
                x1,y1,x2,y2,x3,y3 = self.forward_kinematics(best_soln)
                if not self.within_grid(x1,y1,x2,y2,x3,y3):
                    print("In this solution, one or more joints lie outside grid. Skipping")
                    continue

                #For the best solution in this case, find delta and arrange angles in the table to satisfy given pre-conditions
                angle1, angle2, angle3 = np.degrees(best_soln)
                theta = np.array([90-angle1, -angle2, -angle3])
                delta_angles = np.abs(theta - prev_angles) if prev_angles is not None else np.zeros(3)
                max_delta = np.max(delta_angles)

                #Saving for next step
                prev_angles = theta

                #Print table
                print(f"{i+1:>4} | ({x1:6.1f}, {y1:6.1f}) | {theta[0]:7.2f} | {delta_angles[0]:7.2f} | "
                    f"({x2:6.1f}, {y2:6.1f}) | {theta[1]:7.2f} | {delta_angles[1]:7.2f} | "
                    f"({x3:6.1f}, {y3:6.1f}) | {theta[2]:7.2f} | {delta_angles[2]:7.2f} | {max_delta:7.2f}")
                
                #Clear canvas to display arm position afresh
                self.canvas.delete("all")
                self.draw_grid()
                self.draw_circle()
                self.draw_arm(best_soln)
                
                #By default, tkinter will not show the figure at each intermittent step. We want to see all intermediate arm steps.
                #Hence, force canvas update.
                self.canvas.update()
                time.sleep(0.25)
            
            #Check if arm has actually reached the point
            print("Distance between destination and final end effector position:", math.sqrt((x3-world_x)**2 + (y3-world_y)**2))
            self.current_position = destination