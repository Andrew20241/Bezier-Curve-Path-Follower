"""Libraries need for Code"""
from vex import * 
import math 

"""Defines the objects for the Controller and Brain for VEX Robot"""
controller_1 = Controller(ControllerType.PRIMARY)
brain = Brain() 

#############################        Pneumatics       #####################################

class better_pneumatic:
    """This code is aimed toward making the Pneumatics more easier to implement without worrying about states.

    Methods: Constructor and Toggle
    """
    def __init__(self, Pneumatic_obj):
        self.p = Pneumatic_obj
        
    def toggle(self):
        """Toggles the pneumatics to the opposite state in which it is in
        
        Args: None

        Returns: None
        """
        if self.p.value() == True: 
            self.p.close() 
        elif self.p.value() == False: 
            self.p.open() 

        wait(0.1, SECONDS)

#############################        Generation       #####################################

class Point:
    """Defines a Point in x and y coordinate space."""

    def __init__(self, x, y):
        self.x = x
        self.y = y

class Curve_Generator: 
    """Generates Bezier Curve based on given point."""
    def __init__(self):
        self = self

    def distance_formula(self, x1, x2, y1, y2): 
        """Distance forumla Method calculator.
        
        Args: 
            x1: first point X coordinate
            x2: second point X coordinate
            y1: first point Y coordinate
            y2: second point Y coordinate

        Returns:
            double: distance between points
        """
        return math.sqrt(((x2 - x1)**2) + ((y2 - y1)**2))

    def generator(self, point): 
        """Given a list of points, it will generate a Bezier Curve.

        Args: 
            point: list of points from user input

        Returns:
            list: list of new points that are the defined path of the Bezier Curve
        """
        P1 = point[0]
        P2 = point[1]
        P3 = point[2]
        P4 = point[3]
    
        Ppath = []

        for t in range(0, 20):
            z = 0.05 * t

            x_path = (((1 - z)**3) * P1.x) + (3 * ((1 - z)**2) * (z) * P2.x) + (3 * (1 - z) * ((z)**2) * P3.x) + ((z**3) * P4.x)
            y_path = (((1 - z)**3) * P1.y) + (3 * ((1 - z)**2) * (z) * P2.y) + (3 * (1 - z) * ((z)**2) * P3.y) + ((z**3) * P4.y)

            first_derivative_x = ((3 * (1 - z)**2) * (P2.x - P1.x)) + (6 * (1-z) * z * (P3.x - P2.x)) + (3 * (z**2) * (P4.x - P3.x))
            first_derivative_y = ((3 * (1 - z)**2) * (P2.y - P1.y)) + (6 * (1-z) * z * (P3.y - P2.y)) + (3 * (z**2) * (P4.y - P3.y))

            second_derivative_x = (6 * (1-z) * (P3.x - (2 * P2.x) + P1.x)) + (6 * z * (P4.x - (2 * P3.x) + P2.y))
            second_derivative_y = (6 * (1-z) * (P3.y - (2 * P2.y) + P1.y)) + (6 * z * (P4.y - (2 * P3.y) + P2.y))

            speed = math.sqrt((first_derivative_x**2) + (first_derivative_y**2))

            curverature = ((first_derivative_x * second_derivative_y) - (first_derivative_y * second_derivative_x)) / (speed**3)

            Ppath.append([x_path, y_path, speed, curverature])

        return Ppath

#############################        Drivetrain       #####################################

class better_drivetrain: 
    """Drivetrain class that contains methods to move the robot."""

    def __init__(self, left, right, inert, gen, gear_ratio, wheel_p):
        self.left = left
        self.right = right                
    
        self.inert = inert

        self.gen = gen 

        self.gear_ratio = gear_ratio

        self.wheel_p = wheel_p

    def motor_setting(self, bool_reset, type):
        """Sets the motor type
        
        Args:
            bool_reset: reset encoder
            type: setting the motor type
        """
        if bool_reset == True: 
            for i in self.left:
                i.reset_position()
            for i in self.right: 
                i.reset_position()  

        if type == 'COAST':
            for i in self.left:
                i.set_stopping(COAST)
            for i in self.right: 
                i.set_stopping(COAST)
        else: 
            for i in self.left:
                i.set_stopping(BRAKE)
            for i in self.right: 
                i.set_stopping(BRAKE)

    def move_rpm(self, left_val, right_val): 
        """Moving the robot using RPM values.

        Args: 
            left_val: RPM for left side of drivetrain
            right_val: RPM for right side of drivetrain
        """
        for i in self.left: 
            i.spin(FORWARD, left_val, RPM)
        for i in self.right: 
            i.spin(FORWARD, right_val, RPM)

    def move_pct(self, left_val, right_val): 
        """Moving the robot using Percent values.

        Args: 
            left_val: PCT for left side of drivetrain
            right_val: PCT for right side of drivetrain
        """
        for i in self.left: 
            i.spin(FORWARD, left_val, PERCENT)
        for i in self.right: 
            i.spin(FORWARD, right_val, PERCENT)


    def control(self, forward, turn, t): 
        """Control value to allow for greater control in high and low speeds.
        
        Args: 
            forward: throttle value
            turn: turn value
            t: constant for how much control
        """
        forward = (math.exp(((abs(forward) - 127)*t)/1000) * forward)/127
        turn = (math.exp(((abs(turn) - 127)*t)/1000) * turn)/127
        
        self.move_pct((forward + turn), (forward - turn))   

    def user_control(self): 
        """User controls."""
        while True: 
            self.control(controller_1.axis2.value(), controller_1.axis4.value(), 15)

            wait(0.1, SECONDS) 

            controller_1.screen.clear_row(1)    
            controller_1.screen.set_cursor(1, 1)

    def drive(self, org_path, own_scale_factor, own_curverature): 
        """Follows the path given by input.

        Args: 
            org_path: original by user
            own_scale_factor: how much to speed up the velocities
            own_curverature: how much curve should the robot drive
        """
        track_width = 0.875   
        max_velocity_rpm = 200

        path_data = self.gen.generator(org_path)

        left_velocity = []
        right_velocity = []

        for i in range(0, len(path_data)-1): 
            if abs(path_data[i][3]) < 1e-5 or own_curverature: 
                curverature = 0
                left_velocity.append(-1000)
                right_velocity.append(-1000)
            else: 
                curverature = path_data[i][3]
                omega = path_data[i][2] * curverature 

                left_velocity.append(path_data[i][2] - (track_width / 2) * omega)
                right_velocity.append(path_data[i][2] + (track_width / 2) * omega) 

        largest_left = max(left_velocity)
        largest_right = max(right_velocity)
        largest_avg = (largest_left + largest_right)/2
        scale_factor = max_velocity_rpm / largest_avg * own_scale_factor

        for i in range(0, len(left_velocity)): 
            if left_velocity[i] == -1000: 
                left_velocity[i] = max_velocity_rpm
            else: 
                left_velocity[i] = left_velocity[i] * scale_factor
                right_velocity[i] = right_velocity[i] * scale_factor
            if right_velocity[i] == -1000: 
                right_velocity[i] = max_velocity_rpm
            else: 
                left_velocity[i] = left_velocity[i] * scale_factor
                right_velocity[i] = right_velocity[i] * scale_factor

            if left_velocity[i] > max_velocity_rpm: 
                left_velocity[i] = max_velocity_rpm
            elif right_velocity[i] > max_velocity_rpm: 
                right_velocity[i] = max_velocity_rpm

            self.move_rpm(left_velocity[i], right_velocity[i])

            wait(0.1, SECONDS)

            controller_1.screen.clear_screen() 
            controller_1.screen.set_cursor(1, 0)
            controller_1.screen.print("left rpm: {0:.2f}".format(left_velocity[i]))
            controller_1.screen.set_cursor(2, 0)
            controller_1.screen.print("right rpm: {0:.2f}".format(left_velocity[i]))
            controller_1.screen.set_cursor(3, 0)
            controller_1.screen.print("left_ft: {0:.2f}".format(path_data[i][2]))


            wait(0.1, SECONDS)

        self.move_pct(0, 0)

        controller_1.screen.clear_screen()

    def turn(self, angle):
        """Accurate turning method for drivetrain/

        Args: 
            angle: amount of degree wanting to turn relative to the robot current heading
        """
        while True: 
            minAngle = angle - self.inert.rotation() 

            if minAngle > 0: 
                self.move_pct(70, -70)
            else: 
                self.move_pct(-70, 70)

            if minAngle > 180: 
                minAngle = angle - 360 - self.inert.rotation()
        
            if minAngle < -180: 
                minAngle = angle + (360 - self.inert.rotation())

            if self.inert.rotation() - 5 == angle: 
                break 

#############################        Declaration       #####################################

left_temp_drive = [
    Motor(Ports.PORT4, GearSetting.RATIO_18_1, False), 
]

right_temp_drive = [
    Motor(Ports.PORT20, GearSetting.RATIO_18_1, True), 
]

inertial = Inertial(Ports.PORT6)

cgen = Curve_Generator()

drivetrain = better_drivetrain(left_temp_drive, right_temp_drive, inertial, cgen, 1, 2.75) 

#############################        Code       #####################################

drivetrain.motor_setting(True, "COAST")

inertial.calibrate() 

wait(2, SECONDS)

skill = [Point(2, 0), Point(2, 1), Point(2, 2), Point(2, 3)]

#drivetrain.drive(skill, 1, True)
drivetrain.turn(90)

drivetrain.user_control()
