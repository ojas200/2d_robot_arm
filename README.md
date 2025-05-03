# 2D-Robot-Arm Manipulation & Analysis
Simple implementation of Python-GUI based 2D robot arm for analysis of robot arm workspace and reachability

### Installation
The required dependencies can be installed, if not previously present in the system by using<br>
```
pip install tkinter
pip install python-math
```

### Implementation<br>
This repository includes a Python script called `arm.py` and a test script called `2d_robot_arm.py`.<br>
`arm.py` contains a class called `RobotArmGUI` which includes functionality for an interactive GUI based on `tkinter` to simulate and analyze a 2D Robot Arm with three links. The `RobotArmGUI` script contains following functions:

1. `to_screen`: Converts GUI Coordinates to world coordinates for the robot's calculation
2. `draw_grid`: Draws the grid as per requirements.
3. `draw_circle`: Draws the circle as per requirements on the grid.
4. `draw_arm`: Draws the robot arm given input joint angles.
5. `solve_ik`: Solves the inverse kinematics solution for some given input end-effector configuration.
6. `forward_kinematics`: Solves forward kinematics to determine end-effector position given input joint angles
7. `within_grid`: Checks if end-effector is within grid limits.
8. `on_click_dp`: A solver for dynamic programming based approach to find a unique joint space combination that minimizes sharp jumps during traversal.
9. `on_click`: Regular inverse kinematics solver that choose joint angles at each step to minimize maximum angle jump from previous time step.


### Usage
To open the GUI, install requirements first. Next, clone repository to local machine (preferably Linux-based OS but Windows will also work):<br>
`git clone https://github.com/ojas200/2d_robot_arm.git`<br>

To run on terminal, run `python 2d_robot_arm.py` or use a code editor such as VSCode or such to run the script `2d_robot_arm.py`.<br>
Currently, we are using `on_click` for implementation of trajectory generation. `on_click_dp` is still a work in progress.
