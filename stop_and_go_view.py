# Uber, Inc. (c) 2019
# Description  : It has camera and Frame State information
####################################################
import numpy as np
import stop_and_go_globals as sg

#####################################################################


class Camera(object):
    """ To get the information about the position of the camera wrt reference car """

    def __init__(self, car_list):
        self.car_list = car_list
        self.set_camera_pos_x = 0.0
        self.set_camera_pos_y = 0.0
        self.set_camera_ang = 0.0

    ####################################################

    def get_camera_cur_pos(self, cur_time, ref_time, save_sim_flow_data):
        """ Get the camera position in inverted Y coordinate frame
            Args:
                cur_time(float)                : Current simulation time
                ref_time(float)                : Reference time wrt reference car's reference time
                save_sim_flow_data(dictionary) : Contains the simulation data for each movements on the frame
            Returns:
                tuple                          : Returns the camera positions (x, y & camera angle)
        """
        self.get_camera_positions(ref_time, save_sim_flow_data)
        return (self.set_camera_pos_x, self.set_camera_pos_y, self.set_camera_ang)

    ####################################################

    def set_camera_pos(self, ref_time, save_sim_flow_data):
        """ Set the camera positions wrt to the reference car's reference time and
            the configuration mode. Set the camera position once.
            Args:
                ref_time(float)                : Reference time wrt reference car's reference time
                save_sim_flow_data(dictionary) : Contains the simulation data for each movements on the frame
        """
        # Get the reference x and y position of the camera wrt at 5 sec position
        if not sg.SET_CAMERA_ONCE:
            # Get the camera psoition from the new saved data base
            camera_pos_x = int(save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][ref_time][1].center_x_p)
            camera_pos_y = int(save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][ref_time][1].center_y_p)
            self.set_camera_ang = save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][ref_time][
                1
            ].heading_rad

            self.set_camera_pos_x = camera_pos_x
            self.set_camera_pos_y = camera_pos_y

            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(
                    " before rotation camera positions heading angle ",
                    self.set_camera_pos_x,
                    self.set_camera_pos_y,
                    self.set_camera_ang,
                )

            # Set the camera position one for one simulation
            sg.SET_CAMERA_ONCE = True

    ####################################################

    def get_camera_positions(self, ref_time, save_sim_flow_data):
        """ Get the camera positions wrt to the reference time and
            the configuration mode.
            Args:
                ref_time(float)                : Reference time wrt reference car's reference time
                save_sim_flow_data(dictionary) : Contains the simulation data for each movements on the frame
        """
        # Get the reference x and y position of camera
        ref_mid_x, ref_mid_y = self.car_list[sg.REFERENCE_CAR_SEQ - 1].get_car_center()

        # To check camera configurable position is present
        if hasattr(sg, "CAMERA_POS_X") and hasattr(sg, "CAMERA_POS_Y"):
            self.set_camera_pos_x = sg.CAMERA_POS_X
            self.set_camera_pos_y = sg.CAMERA_POS_Y
        else:
            # To check the camera is in fixed frame mode
            if hasattr(sg, "FIXED_FRAME_CAMERA_VIEW"):
                # Get the reference x and y position of the camera wrt at 5 sec position
                self.set_camera_pos(ref_time, save_sim_flow_data)
            else:
                self.set_camera_pos_x = ref_mid_x
                self.set_camera_pos_y = ref_mid_y


####################################################


class Frame_State(object):
    def __init__(self, frame_span, frame_window, ref_frame, last_frame, start_frame, start_frame_dev):
        """ Maintain the frame states
            Args:
                frame_span(int)       : Number of frames per simulation
                frame_window(int)     : Shift the start frame number after each simulation
                ref_frame(int)        : Reference frame number based on reference time of the car
                last_frame(int)       : Last frame number
                start_frame(int)      : Start frame number
        """
        self.initial_start_frame = start_frame
        self.initial_start_frame_dev = start_frame_dev
        self.initial_frame_span = frame_span
        self.initial_frame_window = frame_window
        self.initial_ref_frame = ref_frame
        self.initial_last_frame = last_frame

        self.start_frame = 0
        self.end_frame = 0
        self.ref_frame = 0
        self.frame_span = 0
        self.frame_window = 0
        self.last_frame = 0
        self.reset_frames()

    ####################################################
    def reset_frames(self):
        """ Reset the frame's parameter after the end of the simulation """
        # if frames are greater than the end_frame , reset to the beginning
        self.start_frame = int(
            np.random.uniform(
                self.initial_start_frame - self.initial_start_frame_dev,
                self.initial_start_frame + self.initial_start_frame_dev,
                1,
            )[0]
        )
        self.ref_frame = self.start_frame + self.initial_ref_frame
        self.end_frame = self.start_frame + self.initial_frame_span
        self.frame_span = self.start_frame + self.initial_frame_span
        self.frame_window = self.start_frame + self.initial_frame_window
        self.last_frame = self.initial_last_frame

        if sg.DEBUG == sg.DEBUG_LEVEL_1:
            print(" start_frame = ref_frame = and end_frame = ", self.start_frame, self.ref_frame, self.end_frame)


####################################################
