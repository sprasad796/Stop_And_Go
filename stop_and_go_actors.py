####################################################
# Uber, Inc. (c) 2019
# Description : Description of all the actors interacting with the Car
####################################################
import math

import pygame
import stop_and_go_globals as sg
from stop_and_go_data_type import CarState, CarTurn, HeadingDirection

####################################################


class Stop_Area(object):
    def __init__(self, x, y, width, length, window):
        """ Intersection area on the frame which is for 4-way traffic. It is
            an area/junction of the readway where two or more roads cross or meet.
            Args:
                x(float)              : X-position of the stop area where it starts in x-coordinate
                y(float)              : Y-position of the stop area where it starts in x-coordinate
                width(float)          : Width of the stop area where it ends in x-direction
                length(float)         : Length of the stop area where it ends in y-direction
                window(pygame window) : Current frame
        """
        self.x = x  # X-position of the stop area
        self.y = y  # Y-position of the stop area
        self.width = width  # Width of the stop area
        self.length = length  # Length of the stop area
        self.window = window  # Current window
        self.corners = {}  # Corner points

        self.fill_corner_points()

    ####################################################

    def fill_corner_points(self):
        """ Fill with the 4 corners of the boundary points of intersection
            area to draw it
        """
        self.corners[1] = [0, self.y, self.y + self.width]
        self.corners[2] = [0, self.x + self.length, self.x]
        self.corners[3] = [0, self.y + self.width, self.y]
        self.corners[4] = [0, self.x, self.x + self.length]

    ####################################################
    def render(self, no_car, color=sg.WHITE):
        """ Draw the intersection area.
            Args:
                no_car(bool)          : If the current frame doesn't has car draw with green color else white
                color(list)           : Default is white color to draw the intersection
        """
        if no_car:
            pygame.draw.rect(self.window, sg.GREEN, (self.x, self.y, self.width, self.length))
        else:
            pygame.draw.rect(self.window, sg.WHITE, (self.x, self.y, self.width, self.length))


####################################################################


class Stop_Line(object):
    def __init__(self, start_x, start_y, end_x, end_y, line_width, color=sg.RED):
        """ Stop line properties, where all the cars stop on the path before
            the intersection. To draw the stop lines
            Args:
                start_x(float)         : X-position of the stop line where it starts in x-coordinate
                start_y(float)         : Y-position of the stop line where it starts in y-coordinate
                end_x(float)           : X position of the stop line where it ends in the x-coordinate
                end_y(float)           : Y position of the stop line where it ends in the y-coordinate
                line_width(pixels)     : Stop line width in pixels
                color(list)            : Default color is red
        """
        self.start = (start_x, start_y)
        self.stop = (end_x, end_y)
        self.color = color  # Line color
        self.line_width = line_width  # Line width

    ####################################################

    def draw(self, window):
        """ Draw the stop lines.
            Args:
                window(pygame window)        : Current frame
        """
        pygame.draw.line(window, sg.BLACK, self.start, self.stop, self.line_width)
        pygame.draw.line(window, sg.RED, self.start, self.stop, self.line_width)


###################################################################################################


class Turn_Status(object):
    def __init__(self, car_seq, car_turn, car_width, car_length):
        """ Maintain the turn status properties of the cars. This gets the center of curvature ,
            and center points of the car about to take turn. It is applied during turning of the car.
            Args:
                car_seq(int)        : Car sequence number
                car_turn(int)       : Car turn { left, right, no }
                car_width(int)      : Car width in pixels
                car_length(int)     : Car length in pixels
        """
        self.turning = False  # Start Turning from intersection
        self.turned = False  # Turned from the intersection
        self.turn_center_x = 0.0  # x position of the center of the turn
        self.turn_center_y = 0.0  # y position of the center of the turn
        self.radius = 0.0  # Radius of curvature during turn of the car
        self.car_seq = car_seq  # Car's sequence number
        self.car_turn = car_turn  # Car's turn type
        self.car_width = car_width  # Car's width
        self.car_length = car_length  # car's length

    #############################################################################

    def get_turn_circle_center(self, path_list_y, center_x, center_y, seq, turn, window):
        """ Get the center of curvature of the car's at stop line while turning
            Args:
                path_list_y(float)      : Y coordinate of the path to calculate the radius of the curvature
                center_x(float)         : X coordinate of the center around car is about to take turn
                center_y(float)         : Y coordinate of the center around car is about to take turn
                seq(int)                : Sequence of the car
                turn(int)               : Turn type ( left, right , no)
                window(pygame window)   : Current frame
        """
        if (seq == sg.CAR_SEQ_2) or (seq == sg.CAR_SEQ_4):
            intersection_y = path_list_y
            self.radius = abs(center_y - intersection_y)
        if (seq == sg.CAR_SEQ_1) or (seq == sg.CAR_SEQ_3):
            intersection_x = path_list_y
            self.radius = abs(center_x - intersection_x)
        if ((seq == sg.CAR_SEQ_4) and (turn == CarTurn.RIGHT.value)) or (
            (seq == sg.CAR_SEQ_2) and (turn == CarTurn.LEFT.value)
        ):
            self.turn_center_x = center_x + self.radius
            self.turn_center_y = center_y
        elif ((seq == sg.CAR_SEQ_4) and (turn == CarTurn.LEFT.value)) or (
            (seq == sg.CAR_SEQ_2) and (turn == CarTurn.RIGHT.value)
        ):
            self.turn_center_x = center_x - self.radius
            self.turn_center_y = center_y
        elif ((seq == sg.CAR_SEQ_1) and (turn == CarTurn.RIGHT.value)) or (
            (seq == sg.CAR_SEQ_3) and (turn == CarTurn.LEFT.value)
        ):
            self.turn_center_x = center_x
            self.turn_center_y = center_y + self.radius
        elif ((seq == sg.CAR_SEQ_1) and (turn == CarTurn.LEFT.value)) or (
            (seq == sg.CAR_SEQ_3) and (turn == CarTurn.RIGHT.value)
        ):
            self.turn_center_x = center_x
            self.turn_center_y = center_y - self.radius

        self.turning = True


####################################################


class Pose(object):
    def __init__(self, x, y, heading_angle):
        """ Car's Pose information e.g current position, heading angle
            Args:
                x(float)           : Currnet position of the car in x coordinate
                y(float)           : Currnet position of the car in y coordinate
                heading_angle(rad) : Heading angle in radian
        """
        self.x = x
        self.y = y
        self.heading_angle = heading_angle


####################################################


class DynamicState(object):
    def __init__(self):
        """ Car's dynamic state information e.g speed, acceleartion """
        self.speed = 0  # Current speed of the car
        self.accl = 0  # Current acceleration of the car
        self.set_speed = False


####################################################


class PhysicalProperties(object):
    def __init__(self, width, length, seq):
        """ Car's current Physical State information.
            Args:
                width(int)       : Width of the car
                length(int)      : length of the car
                seq(int)         : Sequence number of the car
        """
        self.points = []  # All the points to draw the car's polygon
        self.width = width  # Width of the car
        self.length = length  # Length of the car


####################################################


class SimState(object):
    def __init__(self, seq):
        """ Car's simulation State information.
            Args:
                seq(int)          : Sequence number of the car
        """
        self.cur_state = CarState.CRUISE.value  # Sim State maintained by the car
        self.dp = 0  # Delta increment in x or y direction
        self.t_dp = 0  # Total displacement during truning state
        self.seq = seq  # Sequence number of the car
        self.end = False  # Car has completed the current simulation
        self.lock = False  # Car has locked the stop area zone to make  movement in stop area
        self.overlap = False  # Car has overlapped the middle stop area
        self.stop_car = False  # Car should stopped at stop zone or not


####################################################


class Car(object):
    def __init__(self, x, y, width, length, seq, sim, heading_angle, window):
        """ Car's information. This contains the information about the car
            at particular time.
            Args :
                x(float)              : Car's x position
                y(float)              : Car's y position
                width(int)            : Car's width
                length(int)           : Car's length
                seq(int)              : Car's sequence number
                sim(object)           : Car' simulation object which contains simulation data for
                                        the car instance
                heading_angle(rad)    : Car's current heading angle
                window(pygame window) : Current frame
        """
        self.pose = Pose(x, y, heading_angle)
        self.prev_pose = Pose(0, 0, 0)
        self.initial_pose = Pose(x, y, 0)
        self.dynamic_state = DynamicState()
        self.physical_properties = PhysicalProperties(width, length, seq)
        self.sim_state = SimState(seq)
        self.window = window  # Current window
        self.stop_timer = 0  # Total time spend by the car at stop zone
        self.sim = sim  # Simulation data for this car
        self.time_index = 0  # Total time travel by car in window and it should emulate the sim. This
        self.stop_time_index = 0  # Total time spent by the car at stop sign due to other cars
        self.Turn_Status = Turn_Status(
            self.sim_state.seq, self.sim.turn, self.physical_properties.width, self.physical_properties.length
        )  # Instantiate the Turn_Status
        self.car_turn_path = {}  # Store the car seq key and path turn no as { no_turn, left_turn, right_turn }
        self.populate_car_turn_path_no()  # Populate the car_turn_path

    ####################################################

    def populate_car_turn_path_no(self):
        """ Mapping of the car's turn information based on the type of turn
            Car seq_no (key) , List[no_path_seq_no, left_turn_path_seq_no, right_turn_path_seq_no]
        """
        self.car_turn_path[sg.CAR_SEQ_1] = [
            (sg.PATH_SEQ_0, HeadingDirection.EAST.value),
            (sg.PATH_SEQ_3, HeadingDirection.NORTH.value),
            (sg.PATH_SEQ_1, HeadingDirection.SOUTH.value),
        ]

        self.car_turn_path[sg.CAR_SEQ_2] = [
            (sg.PATH_SEQ_1, HeadingDirection.SOUTH.value),
            (sg.PATH_SEQ_0, HeadingDirection.EAST.value),
            (sg.PATH_SEQ_2, HeadingDirection.WEST.value),
        ]

        self.car_turn_path[sg.CAR_SEQ_3] = [
            (sg.PATH_SEQ_2, HeadingDirection.WEST.value),
            (sg.PATH_SEQ_1, HeadingDirection.SOUTH.value),
            (sg.PATH_SEQ_3, HeadingDirection.NORTH.value),
        ]

        self.car_turn_path[sg.CAR_SEQ_4] = [
            (sg.PATH_SEQ_3, HeadingDirection.NORTH.value),
            (sg.PATH_SEQ_2, HeadingDirection.WEST.value),
            (sg.PATH_SEQ_0, HeadingDirection.EAST.value),
        ]

    ####################################################

    def check_outside_boundary(self):
        """ Check the car's boudnary outside of the frame.
            If Yes set it True else False
        """
        # During turn length and width gets swap, so need to offset it.
        self.sim_state.end = (
            True
            if (
                self.pose.x + self.physical_properties.length + sg.CAR_BOUNDARY_OFFSET < 0
                or self.pose.x >= sg.WINDOW_WIDTH_PIXELS
                or self.pose.y >= sg.WINDOW_LENGTH_PIXELS
                or self.pose.y + self.physical_properties.length < 0
            )
            else False
        )

    ####################################################

    def get_car_point_list(self, create=True):
        """ Get the car boundary points to draw on frame.
            Args:
                create(bool)       : If create is True , create the car's boudnary points
                                     based on the car's current position
            Returns:
                list               : Returns the car's boundary points
        """
        # During turning state
        if create:
            self.physical_properties.points = [
                [self.pose.x, self.pose.y],
                [self.pose.x + self.physical_properties.width, self.pose.y],
                [self.pose.x + self.physical_properties.width, self.pose.y + self.physical_properties.length],
                [self.pose.x, self.pose.y + self.physical_properties.length],
            ]

        return self.physical_properties.points

    ####################################################

    def rotate_point_origin(self, angle):
        """ Rotate the car around it's center and
            return the car's new boundary points to draw on farme.
            Args:
                angle(radian)  : Angle about which car is taking turn.
            Returns:
                list           : Returns the new car's center and new boundary points
                                 after the rotation.
        """
        new_points = []
        (o_x, o_y) = self.get_car_center()
        points = self.get_car_point_list()

        # Anticlockwise Rotation
        if (
            (self.sim_state.seq == sg.CAR_SEQ_4)
            or (self.sim_state.seq == sg.CAR_SEQ_1 and self.sim.turn == CarTurn.RIGHT.value)
            or (self.sim_state.seq == sg.CAR_SEQ_3 and self.sim.turn == CarTurn.RIGHT.value)
        ):
            for point in points:
                x1 = o_x + math.cos(angle) * (point[0] - o_x) + math.sin(angle) * (point[1] - o_y)
                y1 = o_y - math.sin(angle) * (point[0] - o_x) + math.cos(angle) * (point[1] - o_y)
                new_points.append([x1, y1])

        # Clockwise Rotation
        elif (
            (self.sim_state.seq == sg.CAR_SEQ_2)
            or (self.sim_state.seq == sg.CAR_SEQ_1 and self.sim.turn == CarTurn.LEFT.value)
            or (self.sim_state.seq == sg.CAR_SEQ_3 and self.sim.turn == CarTurn.LEFT.value)
        ):
            for point in points:
                x1 = o_x + math.cos(angle) * (point[0] - o_x) - math.sin(angle) * (point[1] - o_y)
                y1 = o_y + math.sin(angle) * (point[0] - o_x) + math.cos(angle) * (point[1] - o_y)

                new_points.append([x1, y1])

        return new_points

    ####################################################

    def get_car_center(self):
        """ Returns the center of the car
            Returns:
                tuple          : Returns the x and y coordinate of the center of the
        """
        return (self.pose.x + self.physical_properties.width / 2, self.pose.y + self.physical_properties.length / 2)

    ####################################################

    def get_turning_angle(self, ang_rad):
        """ Get the turning angle of car around center of the curvature.
            Args:
                ang_rad(radian)       : Based on the car's displacement, get the actual angle of rotation
            Returns:
                radian                : Returns the turning angle of the car from the
                                        center of curvature
        """
        if ((self.sim.turn == CarTurn.RIGHT.value) and (self.sim_state.seq == sg.CAR_SEQ_4)) or (
            (self.sim_state.seq == sg.CAR_SEQ_2) and (self.sim.turn == CarTurn.LEFT.value)
        ):
            cur_ang = math.pi - ang_rad
        elif ((self.sim.turn == CarTurn.LEFT.value) and (self.sim_state.seq == sg.CAR_SEQ_4)) or (
            (self.sim_state.seq == sg.CAR_SEQ_2) and (self.sim.turn == CarTurn.RIGHT.value)
        ):
            cur_ang = ang_rad
        elif (self.sim_state.seq == sg.CAR_SEQ_1) or (self.sim_state.seq == sg.CAR_SEQ_3):
            cur_ang = math.pi / 2 - ang_rad

        return cur_ang

    ####################################################

    def get_delta_increments(self, cur_ang):
        """ Delta increment of the car while turning.
            Args:
                cur_ang(radian)     : Car's displacement in x-y direction based on the car's turn
                                      around the angle.
            Returns:
                float               : Returns the car's displacement in x direction
                float               : Returns the car's displacement in y direction
        """
        # Calculate delta increment about the center of AV
        if (self.sim_state.seq == sg.CAR_SEQ_4) or (
            self.sim_state.seq == sg.CAR_SEQ_1 and self.sim.turn == CarTurn.RIGHT.value
        ):
            dx = self.Turn_Status.radius * math.cos(cur_ang)
            dy = -self.Turn_Status.radius * math.sin(cur_ang)
        elif (self.sim_state.seq == sg.CAR_SEQ_2) or (
            self.sim_state.seq == sg.CAR_SEQ_1 and self.sim.turn == CarTurn.LEFT.value
        ):
            dx = self.Turn_Status.radius * math.cos(cur_ang)
            dy = self.Turn_Status.radius * math.sin(cur_ang)
        elif (self.sim_state.seq == sg.CAR_SEQ_3) and (self.sim.turn == CarTurn.RIGHT.value):
            dx = -self.Turn_Status.radius * math.cos(cur_ang)
            dy = self.Turn_Status.radius * math.sin(cur_ang)
        elif (self.sim_state.seq == sg.CAR_SEQ_3) and (self.sim.turn == CarTurn.LEFT.value):
            dx = -self.Turn_Status.radius * math.cos(cur_ang)
            dy = -self.Turn_Status.radius * math.sin(cur_ang)

        return dx, dy

    ####################################################

    def set_turning_car_points(self, path_list, Stop_Area):
        """ Get the new boundary points while turning to draw it on frame
            Args:
                path_list(list)   : Get the path's constant x,y value. Car will always use this values
                                    at the end of the turn
                Stop_area(object) : Get the previous x, y positions of the car based on the stop area
        """
        path_const_xy = path_list[self.car_turn_path[self.sim_state.seq][self.sim.turn][0]].const_xy

        cur_ang = 0

        self.sim_state.t_dp += self.sim_state.dp

        # Angle made by car based on its total displacement along the path trajectory
        av_ang_rad = self.sim_state.t_dp / self.Turn_Status.radius
        cur_ang = self.get_turning_angle(av_ang_rad)

        dx, dy = self.get_delta_increments(cur_ang)

        # Turned around the center of object. Exact initial points of the object (x, y).
        self.pose.x = self.Turn_Status.turn_center_x + dx - self.physical_properties.width / 2
        self.pose.y = self.Turn_Status.turn_center_y + dy - self.physical_properties.length / 2

        # When to stop turning : Each AV is making 90 degree turn
        if av_ang_rad > math.pi / 2:
            # Turning state is over
            self.Turn_Status.turning = False
            # Start the Turned state
            self.Turn_Status.turned = True
            # Set the heading angle PI/2
            self.pose.heading_angle = math.pi / 2
            # Get the final x and y
            if (self.sim_state.seq == sg.CAR_SEQ_4) or (self.sim_state.seq == sg.CAR_SEQ_2):
                self.pose.x = self.prev_pose.x
                self.pose.y = path_const_xy - self.physical_properties.width / 2
            else:
                self.pose.x = path_const_xy - self.physical_properties.length / 2
                self.pose.y = self.prev_pose.y

            # Switch length and width of the car , as AV made 90 degree turn
            tmp = self.physical_properties.length
            self.physical_properties.length = self.physical_properties.width
            self.physical_properties.width = tmp

            # Update the heading direction
            self.pose.heading_direction = self.car_turn_path[self.sim_state.seq][self.sim.turn][1]

            return

        if (self.sim_state.seq == sg.CAR_SEQ_2) or (self.sim_state.seq == sg.CAR_SEQ_4):
            self.prev_pose.y = self.pose.y
        if (self.sim_state.seq == sg.CAR_SEQ_1) or (self.sim_state.seq == sg.CAR_SEQ_3):
            self.prev_pose.x = self.pose.x

        # Angle made by object with x-axis
        if (self.sim_state.seq == sg.CAR_SEQ_1) or (self.sim_state.seq == sg.CAR_SEQ_3):
            cur_ang = math.pi / 2 + cur_ang

        if sg.DEBUG == sg.DEBUG_LEVEL_1:
            print(" av_ang_rad = and cur_ang = , seq ", av_ang_rad, cur_ang, self.sim_state.seq)

        self.pose.heading_angle = av_ang_rad

        new_points = self.rotate_point_origin(cur_ang)

        self.update_prev_xy(Stop_Area)

        self.physical_properties.points = new_points

    ####################################################

    def update_prev_xy(self, Stop_Area):
        """ Keep updating the previous X & Y positions.
            Args:
                Stop_Area(object) : Based on Stop area position, set the car's previous x, y
        """
        # Left Turns
        if self.sim.turn == CarTurn.LEFT.value and self.sim_state.seq == sg.CAR_SEQ_1:
            self.prev_pose.y = (
                Stop_Area.corners[self.sim_state.seq][1] - self.physical_properties.width - sg.STOP_SAFE_OFFSET
            )
        if self.sim.turn == CarTurn.LEFT.value and self.sim_state.seq == sg.CAR_SEQ_2:
            self.prev_pose.x = Stop_Area.corners[self.sim_state.seq][1] + sg.STOP_SAFE_OFFSET
        if self.sim.turn == CarTurn.LEFT.value and self.sim_state.seq == sg.CAR_SEQ_3:
            self.prev_pose.y = Stop_Area.corners[self.sim_state.seq][1] + sg.STOP_SAFE_OFFSET
        if self.sim.turn == CarTurn.LEFT.value and self.sim_state.seq == sg.CAR_SEQ_4:
            self.prev_pose.x = (
                Stop_Area.corners[self.sim_state.seq][1] - self.physical_properties.length - sg.STOP_SAFE_OFFSET
            )

        # Right Turns
        if self.sim.turn == CarTurn.RIGHT.value and self.sim_state.seq == sg.CAR_SEQ_1:
            self.prev_pose.y = Stop_Area.corners[self.sim_state.seq][2] + sg.STOP_SAFE_OFFSET
        if self.sim.turn == CarTurn.RIGHT.value and self.sim_state.seq == sg.CAR_SEQ_2:
            self.prev_pose.x = (
                Stop_Area.corners[self.sim_state.seq][2] - self.physical_properties.length - sg.STOP_SAFE_OFFSET
            )
        if self.sim.turn == CarTurn.RIGHT.value and self.sim_state.seq == sg.CAR_SEQ_3:
            self.prev_pose.y = (
                Stop_Area.corners[self.sim_state.seq][2] - self.physical_properties.width - sg.STOP_SAFE_OFFSET
            )
        if self.sim.turn == CarTurn.RIGHT.value and self.sim_state.seq == sg.CAR_SEQ_4:
            self.prev_pose.x = Stop_Area.corners[self.sim_state.seq][2] + sg.STOP_SAFE_OFFSET

    ####################################################

    def render_car(self, collision=False, color=sg.BLUE):
        """ Draw the car objects
            Args:
                collision(bool)    : If car has overlapped with the intersection
                color(list)        : Default color to draw the car is blue
        """
        if collision:
            pygame.draw.rect(
                self.window,
                sg.RED,
                (self.pose.x, self.pose.y, self.physical_properties.width, self.physical_properties.length),
            )
        else:
            if self.Turn_Status.turning:
                points = self.get_car_point_list(Create=False)
                pygame.draw.polygon(self.window, color, points, 0)
            else:
                points = self.get_car_point_list()
                pygame.draw.polygon(self.window, color, points, 0)

    ####################################################

    def update(self, dx, dy):
        """ Incremental movement in x & y-direction of the car
            Args:
                dx(float)   : Displacement of car in x-direction
                dy(float)   : Displacement of car in y-direction
        """
        self.pose.x += dx
        self.pose.y += dy

    ####################################################

    def no_turn_update(self):
        """ Incremental movement of the car in case of no turns """
        if self.sim_state.seq == sg.CAR_SEQ_1:
            dx = self.sim_state.dp
            dy = 0
        elif self.sim_state.seq == sg.CAR_SEQ_2:
            dx = 0
            dy = self.sim_state.dp
        elif self.sim_state.seq == sg.CAR_SEQ_3:
            dx = -self.sim_state.dp
            dy = 0
        elif self.sim_state.seq == sg.CAR_SEQ_4:
            dx = 0
            dy = -self.sim_state.dp

        self.update(dx, dy)

    ####################################################

    def after_turn_update(self):
        """ Incremental movement of the car in case of turns """
        if self.sim_state.seq == sg.CAR_SEQ_1:
            if self.sim.turn == CarTurn.RIGHT.value:
                dx = 0
                dy = self.sim_state.dp
            elif self.sim.turn == CarTurn.LEFT.value:
                dx = 0
                dy = -self.sim_state.dp
        elif self.sim_state.seq == sg.CAR_SEQ_2:
            if self.sim.turn == CarTurn.RIGHT.value:
                dx = -self.sim_state.dp
                dy = 0
            elif self.sim.turn == CarTurn.LEFT.value:
                dx = self.sim_state.dp
                dy = 0
        elif self.sim_state.seq == sg.CAR_SEQ_3:
            if self.sim.turn == CarTurn.RIGHT.value:
                dx = 0
                dy = -self.sim_state.dp
            elif self.sim.turn == CarTurn.LEFT.value:
                dx = 0
                dy = self.sim_state.dp
        elif self.sim_state.seq == sg.CAR_SEQ_4:
            if self.sim.turn == CarTurn.RIGHT.value:
                dx = self.sim_state.dp
                dy = 0
            elif self.sim.turn == CarTurn.LEFT.value:
                dx = -self.sim_state.dp
                dy = 0

        self.update(dx, dy)

    ####################################################

    def reset(self):
        """ Reset the car's Args """
        self.pose.x = self.initial_pose.x
        self.pose.y = self.initial_pose.y
        self.stop_timer = 0
        self.stop_time_index = 0
        self.sim_state.end = False
        self.sim_state.overlap = False
        self.dynamic_state.set_speed = False
        self.time_index = 0
        self.sim_state.t_dp = 0

    ####################################################

    def update_car_position(self, path_list, Stop_Area):
        """ Update the movement of the car throughout the frame.
            Args:
                path_list(list)    : List of path objects
                Stop_Area(object)  : Intersection object
        """
        if not self.sim_state.stop_car:
            # Going through the Constant speed/CRUISE Before zone
            round_time_index = round(self.time_index, 2)
            if round_time_index in self.sim.sim_motion_state_dict:
                if self.dynamic_state.set_speed:
                    self.sim_state.dp = self.dynamic_state.speed * sg.TIME_INCREMENT_STEP
                else:
                    self.sim_state.dp = self.sim.sim_motion_state_dict[round_time_index][2]

            # If car has reached end, no need to update the x-y positions
            if self.sim_state.end:
                return

            # Update the car movement in case of turn
            if self.sim.turn == CarTurn.LEFT.value or self.sim.turn == CarTurn.RIGHT.value:
                if self.Turn_Status.turning:
                    self.set_turning_car_points(path_list, Stop_Area)
                elif self.Turn_Status.turned:
                    self.after_turn_update()
                else:
                    self.no_turn_update()

            # Update the car movement in case of no turn
            else:
                self.no_turn_update()
        # Calculate the radius at stop line
        else:
            if self.sim.turn == CarTurn.LEFT.value or self.sim.turn == CarTurn.RIGHT.value:
                center_x, center_y = self.get_car_center()
                self.Turn_Status.get_turn_circle_center(
                    path_list[self.car_turn_path[self.sim_state.seq][self.sim.turn][0]].const_xy,
                    center_x,
                    center_y,
                    self.sim_state.seq,
                    self.sim.turn,
                    self.window,
                )


####################################################


class Path(object):
    def __init__(self, start_x, start_y, stop_x, stop_y, seq, const_xy):
        """ 4-way path and its properties
            Args:
                start_x(float)        : Start x coordinate to draw the path
                start_y(float)        : Start y coordinate to draw the path
                stop_x(float)         : End x coordinate to draw the path
                stop_y(float)         : End y coordinate to draw the path
                seq(int)              : Sequence number of the path
                const_xy(float)       : Path's constant value in either x or y direction
        """
        self.start = (start_x, start_y)
        self.stop = (stop_x, stop_y)
        self.path_seq = seq  # Sequence number of the current path
        self.const_xy = const_xy  # Constant position of path in x or y direction

    ####################################################

    def render_path(self, window, is_green=False):
        """ Utility to draw the path
            Args:
                window(pygame window)    : To draw the object on the frame
                is_green(bool)           : Default color is GREEN
        """
        colour = sg.GREEN if is_green else sg.WHITE

        pygame.draw.line(window, colour, self.start, self.stop, sg.PATH_LINE_WIDTH)


############################################################################################
