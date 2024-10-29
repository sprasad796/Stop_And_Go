#####################################################################
# Uber, Inc. (c) 2019
# Description : Simulator's parameters before starting the main loop.
#               Values are generated based on the Gaussain Distribution
#####################################################################
# import matplotlib.pyplot as plt
import numpy as np
import stop_and_go_globals as sg
import yaml
from stop_and_go_data_type import CarState, CarTurn

#####################################################################


class Sim(object):
    """ To generate the simulation data based on the reading config yml file
        It follows the document
        https://docs.google.com/spreadsheets/d/1A7xSu2Pn7Yu6H77T10eblTQKMHqrSmF4IwswlY3JsqU/edit#gid=0
        It creates simulation data in 5 phases.
        Phase-1 : Cruise Before  ( Car starts with constant velocity and travels upto dist_before_stop_m
        Phase-2 : Decel ( Car starts decelerating to manage to come to the stop at stop line )
        Phase-3 : Stopped ( Car is stopped at stop line for configuration time )
        Phase-4 : Accel ( Car stars accelerating from the stop line to reach upto constant velocity
        Phase-5 : Cruise After ( Car cruises with constant velocity after )
    """

    def __init__(self, config_manager, seq_no, time_stopped):
        """ Initializes the Sim class with
            Args:
                config_manager(object)   : Config_manager object
                seq_no(int)              : Set the sequence number of the car object
                time_stopped             : Stop time for the car at the intersection
        """
        self.dist_before_stop_m = 0.0
        self.dist_cruise_before_m = 0.0
        self.dist_after_stop_m = 0.0
        self.accl_after_stop_mpss = 0.0
        self.decel_before_stop_mpss = 0.0
        self.speed_before_stop_mps = 0.0
        self.speed_after_stop_mps = 0.0
        self.time_stopped_s = time_stopped
        self.sim_time_increment_s = float(config_manager.config_params["car"]["sim_time_increment_s"])
        self.sim_motion_state_dict = {}
        self.current_dict_time_s = 0.0
        self.current_dict_dist = 0.0
        self.resolution = float(config_manager.config_params["car"]["resolution_pixel_meter"])
        self.seq_no = seq_no
        self.v = []
        self.t = []
        self.config_manager = config_manager
        self.turn = CarTurn.NO.value
        self.turn_no = {}
        self.cruise_after_time = 0.0
        self.last_key = 0

        self.populate_turn_no()
        self.generate_simulation_data()

    #####################################################################

    def populate_turn_no(self):
        self.turn_no[CarTurn.NO.value] = "no"
        self.turn_no[CarTurn.LEFT.value] = "left"
        self.turn_no[CarTurn.RIGHT.value] = "right"

    #####################################################################

    def generate_simulation_data(self):
        """ Generate the simulation data based on different phases in sim """
        print("Generating the simulation Data for AV# ", self.seq_no)

        # Make sure total_dist_after_const_speed is less than the dist_before_stop
        total_dist_after_const_speed = self.dist_before_stop_m + 1

        while total_dist_after_const_speed > self.dist_before_stop_m:
            # First Generate the Simulation parameters
            self.generate_sim_param()

            total_dist_after_const_speed = -(self.speed_before_stop_mps * self.speed_before_stop_mps) / (
                2 * self.decel_before_stop_mpss
            )

            self.dist_cruise_before_m = self.dist_before_stop_m - total_dist_after_const_speed

        self.update_cruise_before()
        self.update_decel_before_stop()
        self.update_stopped_time()
        self.update_accel_after_stop()
        self.update_cruise_after()
        self.update_past_sim()

    ##############################################################################

    def plot_av_position_time(self):
        """ Plot the position time graph to have an idea of phases """
        # x-axis Value
        x = sorted(self.sim_motion_state_dict.keys())

        # Corresponding Y-axis value
        y = []
        x1 = []
        prev_y = 0.0
        for x_index in x:
            if x_index > 20:
                break
            x1.append(x_index)
            cur_y = self.sim_motion_state_dict[x_index][2] + prev_y
            prev_y += self.sim_motion_state_dict[x_index][2]
            y.append(cur_y)

        print(" Print data points ")
        print(" y = ", y)
        print(" x1 = ", x1)
        # Get the figure
        # fig = plt.figure(figsize=(8, 6))

        # Plotting the points
        # plt.plot(x1, y)

        # Naming the x-axis
        # plt.xlabel('Time (sec.)')

        # Naming the y-axis
        # plt.ylabel('Position (pixel)')

        # Title of the graph
        # plt.title(' Position (pixels) Vs Time (sec.)')

        # Save the figure
        # image_name = "Position_time_plot" + str(self.seq_no) + ".jpg"
        # fig.savefig(image_name)

    ##########################################################################

    def update_cruise_before(self):
        """ Populate the map for the cruise before phase """
        cruise_time = self.dist_cruise_before_m / self.speed_before_stop_mps
        self.current_dict_time_s = 0
        self.current_dict_time_s += cruise_time
        prev_dist = 0.0

        for time_step in np.arange(0.0, self.current_dict_time_s, self.sim_time_increment_s):
            round_time_step = round(time_step, 1)
            cur_dist = self.speed_before_stop_mps * time_step

            self.current_dict_dist += cur_dist - prev_dist

            # Update the dctionary for the motion state
            self.sim_motion_state_dict[round_time_step] = [
                round_time_step,
                CarState.CRUISE.value,
                round((cur_dist - prev_dist) * self.resolution, 4),
                round(self.current_dict_dist * self.resolution, 4),
                round(self.speed_before_stop_mps, 4),
                0,
            ]

            # Update prev_dist
            prev_dist = cur_dist

            # For velocity plot
            self.v.append(self.speed_before_stop_mps)
            self.t.append(round_time_step)
            self.last_time_key = time_step

    ##########################################################################

    def update_decel_before_stop(self):
        """ Populate the map for the deceleration before the stop line phase """
        # Total deceleration time
        decel_time = -(self.speed_before_stop_mps / self.decel_before_stop_mpss)

        last_time_key = round(sorted(self.sim_motion_state_dict.keys())[-1], 2)

        # Update the total decel time
        self.current_dict_time_s += decel_time

        prev_dist = 0.0

        for time_step_after_cruise in np.arange(
            last_time_key + self.sim_time_increment_s, self.current_dict_time_s, self.sim_time_increment_s
        ):

            time_step = time_step_after_cruise - last_time_key
            cur_dist = (
                self.speed_before_stop_mps * time_step + 0.5 * self.decel_before_stop_mpss * time_step * time_step
            )
            self.current_dict_dist += cur_dist - prev_dist

            # For velocity time plot
            v1 = self.speed_before_stop_mps + self.decel_before_stop_mpss * time_step
            if v1 < 0:
                v1 = 0
            self.v.append(v1)
            self.t.append(round(time_step_after_cruise, 2))

            # Update the dctionary for the motion state
            round_time_step = round(time_step_after_cruise, 2)
            self.sim_motion_state_dict[round_time_step] = [
                round_time_step,
                CarState.DECEL.value,
                round((cur_dist - prev_dist) * self.resolution, 4),
                round(self.current_dict_dist * self.resolution, 4),
                round(v1, 4),
                round(self.decel_before_stop_mpss, 4),
            ]

            prev_dist = cur_dist
            self.last_time_key = time_step_after_cruise

    ##########################################################################

    def update_stopped_time(self):
        """ Populate the map for the stopped time phase """
        last_time_key = round(sorted(self.sim_motion_state_dict.keys())[-1], 2)
        self.current_dict_time_s += self.time_stopped_s
        total_dist = self.sim_motion_state_dict[last_time_key][3]

        for time_step in np.arange(
            last_time_key + self.sim_time_increment_s, self.current_dict_time_s, self.sim_time_increment_s
        ):
            self.v.append(0)
            self.t.append(round(time_step, 2))

            # Update the dctionary for the motion state
            round_time_step = round(time_step, 2)
            self.sim_motion_state_dict[round_time_step] = [
                round_time_step,
                CarState.STOP.value,
                0.0,
                total_dist,
                0.0,
                0.0,
            ]
            self.last_time_key = time_step

    ##########################################################################

    def update_accel_after_stop(self):
        """ Populate the map for the accleration after the stop line phase """
        accl_time = self.speed_after_stop_mps / self.accl_after_stop_mpss
        last_time_key = round(sorted(self.sim_motion_state_dict.keys())[-1], 2)
        self.current_dict_time_s += accl_time - self.sim_time_increment_s
        prev_dist = 0.0
        for time_step_after_stop in np.arange(
            last_time_key + self.sim_time_increment_s, self.current_dict_time_s, self.sim_time_increment_s
        ):

            time_step = time_step_after_stop - last_time_key
            cur_dist_from_stop = 0.5 * self.accl_after_stop_mpss * time_step * time_step
            self.current_dict_dist += cur_dist_from_stop - prev_dist
            prev_dist = cur_dist_from_stop

            vel = self.accl_after_stop_mpss * time_step
            self.t.append(time_step_after_stop)
            self.v.append(vel)

            # Update the dctionary for the motion state
            round_time_step = round(time_step_after_stop, 2)
            self.sim_motion_state_dict[round_time_step] = [
                round_time_step,
                CarState.ACCEL.value,
                round((cur_dist_from_stop - prev_dist) * self.resolution, 4),
                round(self.current_dict_dist * self.resolution, 4),
                round(vel, 4),
                round(self.accl_after_stop_mpss, 4),
            ]

            self.last_time_key = time_step_after_stop

    ##########################################################################

    def update_cruise_after(self):
        """ Populate the map for the velocity aftee the accleartion phase """
        last_time_key = round(sorted(self.sim_motion_state_dict.keys())[-1], 2)
        cruise_after_dist = self.dist_after_stop_m + self.dist_before_stop_m - self.current_dict_dist
        cruise_after_time = cruise_after_dist / self.speed_after_stop_mps
        self.cruise_after_time = cruise_after_time

        self.current_dict_time_s += cruise_after_time - self.sim_time_increment_s

        for time_step in np.arange(
            last_time_key + self.sim_time_increment_s, self.current_dict_time_s, self.sim_time_increment_s
        ):

            round_time_step = round(time_step, 2)
            cur_dist = self.speed_after_stop_mps * self.sim_time_increment_s
            self.current_dict_dist += cur_dist

            self.v.append(self.speed_after_stop_mps)
            self.t.append(round_time_step)

            # Update the dctionary for the motion state
            self.sim_motion_state_dict[round_time_step] = [
                round_time_step,
                CarState.CRUISE_A.value,
                round(cur_dist * self.resolution, 4),
                round(self.current_dict_dist * self.resolution, 4),
                round(self.speed_after_stop_mps, 4),
                0.0,
            ]

            self.last_time_key = time_step

    ##################################################################################

    def update_past_sim(self):
        """ Populate the map for the addiition data points when av goes out of frame """
        last_time_key = round(sorted(self.sim_motion_state_dict.keys())[-1], 2)

        while self.current_dict_dist * self.resolution <= sg.WINDOW_WIDTH_PIXELS:
            round_time_step = round(last_time_key + self.sim_time_increment_s, 2)
            cur_dist = self.speed_after_stop_mps * self.sim_time_increment_s
            self.current_dict_dist += cur_dist
            last_time_key += self.sim_time_increment_s

            self.v.append(self.speed_after_stop_mps)
            self.t.append(round_time_step)

            # Update the dctionary for the motion state
            self.sim_motion_state_dict[round_time_step] = [
                round_time_step,
                CarState.PAST_SIM.value,
                round(cur_dist * self.resolution, 4),
                round(self.current_dict_dist * self.resolution, 4),
                round(self.speed_after_stop_mps, 4),
                0.0,
            ]

            self.last_time_key = round_time_step

    ##################################################################################

    def plot_vel_time(self):
        """ Utility function to plot velcoity time graph """
        # fig = plt.figure(figsize=(8, 6))

        # Plotting the points
        # plt.plot(self.t, self.v)

        # Naming the x-axis
        # plt.xlabel('time ')

        # Naming the y-axis
        # plt.ylabel('Velocity ')

        # Title of the graph
        # plt.title(' Velocity Vs Time')

        # Save the figure
        # image_name = "vel_time_plot" + str(self.seq_no) + ".jpg"
        # fig.savefig(image_name)

    ##################################################################################

    def dist_travel_at_sec(self, n):
        """ Utility function to get the distance at time
            Args:
                n(float)     : Given time index
            Retunrs:
                object       : sim object at given time
        """
        return self.sim_motion_state_dict[n][3]

    ##################################################################################

    def print_dict(self):
        """ Utility function to print the dictionary """
        # x = sorted(self.sim_dict.keys())
        x = sorted(self.sim_motion_state_dict.keys())

        # Corresponding Y-axis value
        y = []
        x1 = []
        prev_y = 0.0
        for x_index in x:
            x1.append(x_index)
            prev_y = self.sim_motion_state_dict[x_index][2] + prev_y
            print(
                " Time = , Current distance = , Total Distance =", x_index, self.sim_motion_state_dict[x_index], prev_y
            )
            y.append(prev_y)

    ##################################################################################

    def generate_sim_param(self):
        """ Generate the simulation parameter after reading from the config """
        min_accl_after_stop = 0.5
        max_accl_after_stop = 10
        min_deccl_before_stop1 = -1
        max_deccl_after_stop2 = -4
        stop_speed_threshold = 1
        turn_low = 0
        turn_high = 3
        turn_step = 1

        car = "car" + str(self.seq_no)
        self.dist_before_stop_m = float(self.config_manager.config_params[car]["dist_before_stop_m"])
        self.dist_after_stop_m = float(self.config_manager.config_params[car]["dist_after_stop_m"])
        accl_after_stop_mean = float(self.config_manager.config_params[car]["accl_after_stop_mean_mpss"])
        accl_after_stop_dev = float(self.config_manager.config_params[car]["accl_after_stop_dev_mpss"])
        decl_before_stop_mean = float(self.config_manager.config_params[car]["decl_before_stop_mean_mpss"])
        decl_before_stop_dev = float(self.config_manager.config_params[car]["decl_before_stop_dev_mpss"])
        speed_before_stop_mean = float(self.config_manager.config_params[car]["speed_before_stop_mean_mps"])
        speed_before_stop_dev = float(self.config_manager.config_params[car]["speed_before_stop_dev_mps"])
        speed_after_stop_mean = float(self.config_manager.config_params[car]["speed_after_stop_mean_mps"])
        speed_after_stop_dev = float(self.config_manager.config_params[car]["speed_after_stop_dev_mps"])

        # accl_after_stop using Gaussian Distribution
        while (self.accl_after_stop_mpss <= min_accl_after_stop) or (self.accl_after_stop_mpss >= max_accl_after_stop):
            self.accl_after_stop_mpss = np.random.normal(accl_after_stop_mean, accl_after_stop_dev, 1)[0]
            print(" accl_after_stop = ", self.accl_after_stop_mpss)

        # decl_before_stop using Gaussian Distribution
        self.decel_before_stop_mpss = 0
        while (
            (self.decel_before_stop_mpss >= 0)
            or (self.decel_before_stop_mpss > min_deccl_before_stop1)
            or (self.decel_before_stop_mpss < max_deccl_after_stop2)
        ):
            self.decel_before_stop_mpss = np.random.normal(decl_before_stop_mean, decl_before_stop_dev, 1)[0]
            print(" decl_before_stop = ", self.decel_before_stop_mpss)

        # speed_before_stop using Gaussian Distribution
        self.speed_before_stop_mps = 0
        while self.speed_before_stop_mps <= stop_speed_threshold:
            self.speed_before_stop_mps = np.random.normal(speed_before_stop_mean, speed_before_stop_dev, 1)[0]
            print(" speed_before_stop = ", self.speed_before_stop_mps)

        while self.speed_after_stop_mps <= stop_speed_threshold:
            self.speed_after_stop_mps = np.random.normal(speed_after_stop_mean, speed_after_stop_dev, 1)[0]
            print(" speed_after_stop = ", self.speed_after_stop_mps)

        # Generate the random value of turn ( 0 = No Turn, 1 = Left Turn, 2 = Right Turn )
        turn_num = int(np.random.randint(turn_low, turn_high, turn_step))
        if turn_num == 0:
            self.turn = CarTurn.NO.value
        elif turn_num == 1:
            self.turn = CarTurn.LEFT.value
        else:
            self.turn = CarTurn.RIGHT.value

        print(" turn       = ", self.turn_no[turn_num])


##################################################################################


class Config_manager(object):
    """ Configuration manager to read the configuration file stop_and_go_config.yml """

    def __init__(self):
        self.config_params = {}
        self.read_yaml()

    def print_config_params(self):
        # Display the key-value pair
        for k1, v1 in self.config_params.items():
            print("main_key k1 = ", k1)
            if type(v1) is dict:
                for k2, v2 in v1.iteritems():
                    print(" sub_key k2 = and value = ", k2, v2)
            else:
                print(" value = ", v1)

    def push(self, k, v):
        sub_params = {}
        if type(v) is dict:
            for v_k, fv_v in v.items():
                sub_params[v_k] = fv_v
            self.config_params[k] = sub_params
        else:
            self.config_params[k] = v

    def read_yaml(self):

        with open(sg.CONFIG_FILE, "r") as ymlfile:
            cfg = yaml.safe_load(ymlfile)

        for k, v in cfg.items():
            self.push(k, cfg[k])


###################################################################################
