####################################################
# Uber, Inc. (c) 2019
# Description : Regenerated data after the simulation
####################################################
import json

import stop_and_go_globals as sg
from stop_and_go_check_collision import CollisionCheck
from stop_and_go_data_type import CarState

#####################################################################################


class JsonFileManager(object):
    """ Json file manager to write the json file with the traffic, sdv information """

    def __init__(self, filename):
        """ Initialize the JsonFileManager object with the filename
            Args:
                filename(string)       : Name of the json file
        """
        self.filename = filename

    def write_json(self, content_list):
        """ Write the generated reference car, traffic cars, stop line information at
        given time to the json file
            Args:
                content_list(list)     : List of information to be written to the json
                                         file in the append mode
        """
        with open(self.filename, "a+") as json_file:
            json_string = json.dumps(content_list, indent=4, sort_keys=True)
            json_file.write(json_string)
            json_file.write("\n")


#####################################################################


class Vehicle_State(object):
    """ Maintain the vehicle states to have tarffic and sdv information """

    def __init__(self):
        self.width_p = 0.0
        self.length_p = 0.0
        self.loc_x_p = 0.0
        self.loc_y_p = 0.0
        self.loc_main_x_p = 0.0
        self.loc_main_y_p = 0.0
        self.center_x_p = 0.0
        self.center_y_p = 0.0
        self.center_main_x_p = 0.0
        self.center_main_y_p = 0.0
        self.rot_rad = 0.0
        self.speed_pps = 0.0
        self.acc_ppss = 0.0
        self.heading_rad = 0.0
        self.rot_heading_rad = 0.0
        self.cur_state = CarState.CRUISE.value
        self.cur_turn = "no"
        self.boundary = []


#####################################################################


class Stop_Line_State(object):
    """ Maintain the center of the stop line information """

    def __init__(self):
        self.center_x_p = 0.0
        self.center_y_p = 0.0


class Road_State(object):
    """ Maintain the road states for stop line information """

    def __init__(self):
        self.stop_line_states = []
        for i in range(4):
            self.stop_line_states.append(Stop_Line_State())


#####################################################################


class Save_Sim_Flow_Data(object):
    """ Save the simulation data after car has gone through the intersection """

    def __init__(self, car_nums):
        # Append number of items into list
        self.sim_data_dict_list = [{} for _ in range(car_nums)]
        self.car_nums = car_nums
        self.collision_check = CollisionCheck()

    #####################################################################

    def populate_road_states(self, stop_line_list, road_state):
        """ Populate the road states with the stop line information
            Args:
                stop_line_list (list): Contains x,y position of the 4 stop line
                road_state(object)   : Road state object to have the stop line information
            Returns:
                road_state(object)   : Returns the center of all 4 stop lines
        """
        for i in range(sg.NUMBER_OF_PATHS):
            road_state.stop_line_states[i].center_x_p = (stop_line_list[i].start[0] + stop_line_list[i].stop[0]) / 2
            road_state.stop_line_states[i].center_y_p = (stop_line_list[i].start[1] + stop_line_list[i].stop[1]) / 2

        return road_state

    #####################################################################

    def populate_vehicle_states(self, vehicle_state, car, center_x, center_y, boundary_points):
        """ Populate the vehicle states information for given frame
            Args:
                vehicle_state(object) : Vehicle_State object to write into json file
                car(object)           : Car object to get the car's information
                center_x(float)       : x coordinate of the center position of the car
                center_y(float)       : y coordinate of the center position of the car
                boundary_points(list) : Boundary points of the car
        """
        # Populate Vehicle States
        vehicle_state.loc_main_x_p = car.pose.x
        vehicle_state.loc_main_y_p = car.pose.y
        vehicle_state.loc_x_p = car.pose.x
        vehicle_state.loc_y_p = car.pose.y
        vehicle_state.center_x_p = center_x
        vehicle_state.center_y_p = center_y
        vehicle_state.center_main_x_p = center_x
        vehicle_state.center_main_y_p = center_y
        vehicle_state.speed_pps = car.dynamic_state.speed
        vehicle_state.acc_ppss = car.dynamic_state.accl
        vehicle_state.heading_rad = car.pose.heading_angle
        vehicle_state.rot_heading_rad = car.pose.heading_angle
        vehicle_state.boundary = boundary_points
        vehicle_state.width_p = car.physical_properties.width
        vehicle_state.length_p = car.physical_properties.length
        vehicle_state.cur_state = car.sim_state.cur_state
        vehicle_state.cur_turn = car.sim.turn_no[car.sim.turn]

    #####################################################################

    def populate_sim_data_car_seq(self, car_list, stop_line_list, frame_state):
        """ Populate the sim data for frame's information after
            car has passed through the intersection
            Args:
                car_list(list)      : car_list contains 4 car objects
                stop_line_list(list): stop_line_list contains 4 stop line objects
                frame_state(object) : frame_state contains information about the frame
            Returns:
                bool                : Return true if it has enough data points to start
                                      drawing on frame
        """
        no_data_required = True
        frame_divisor = 10.0

        for car in car_list:
            # Get the last key from the sim_data_dict_list
            cur_time_step = round((car.time_index + car.stop_time_index), 1)
            vehicle_state = Vehicle_State()
            road_states = Road_State()

            # Start storing after the start frame time
            if cur_time_step < (frame_state.start_frame / frame_divisor):
                no_data_required = False
                if sg.DEBUG == sg.DEBUG_LEVEL_1:
                    print(
                        " Start Frame is less than the expected start time for car_seq # at time ",
                        frame_state.start_frame,
                        car.sim_state.seq,
                        cur_time_step,
                    )
                return no_data_required

            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(
                    " At current time index ",
                    cur_time_step,
                    " & stop_time_index ",
                    car.stop_time_index,
                    " for car seq # ",
                    car.sim_state.seq,
                    " x pos ",
                    round(car.pose.x, 1),
                    " y pos ",
                    round(car.pose.y, 1),
                    " cur_state ",
                    car.sim_state.cur_state,
                    "speed_pps = ",
                    car.dynamic_state.speed,
                    "acc_ppss ",
                    car.dynamic_state.accl,
                    "end frame ",
                    frame_state.end_frame / frame_divisor,
                )

            # Check for if end frame is crossing sim time
            # Only store the data when cur_time_step is greater than the start time for that simulation

            # Get the center of the car
            (center_x, center_y) = car.get_car_center()

            # Get the boundary points of the car
            if car.Turn_Status.turning:
                boundary_points = car.get_car_point_list(False)
                if len(boundary_points) == 0:
                    boundary_points = car.get_car_point_list()
            else:
                boundary_points = car.get_car_point_list()

            # Populate Vehicle States
            self.populate_vehicle_states(vehicle_state, car, center_x, center_y, boundary_points)

            # Populate Road states
            road_state = self.populate_road_states(stop_line_list, road_states)

            self.sim_data_dict_list[car.sim_state.seq - 1][cur_time_step] = [cur_time_step, vehicle_state, road_state]

            # All cars should have end frame
            if cur_time_step <= (frame_state.end_frame / frame_divisor):
                no_data_required = False

        # Check the car's distances from it's leading vehicle
        if sg.ENABLE_VEHICLE_COLLISION_CHECK:
            self.collision_check.check_leading_vehicle_distance(car_list)

        return no_data_required


############################################################################################################
