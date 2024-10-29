#####################################################################
# Uber, Inc. (c) 2019
# Description: This is the main game loop
#####################################################################
import math

import stop_and_go_globals as sg
from stop_and_go_data_type import CarTurn
from stop_and_go_rotate_image import RotateImage
from stop_and_go_subimage import SubImage

####################################################################


class CoordinateTransform(object):
    """ Transform the coordinate from main frame image pixels to
        camera view image pixels
    """

    def __init__(self, exp_no, sub_seq_no, camera, car_list, path_list, frame_state, save_sim_flow_data):
        self.exp_no = exp_no
        self.sub_seq_no = sub_seq_no
        self.camera = camera
        self.frame_state = frame_state
        self.car_list = car_list
        self.path_list = path_list
        self.save_sim_flow_data = save_sim_flow_data

    ####################################################################

    def get_translation_metrics(self):
        """ Get the translation metrics from main frame to sub frame.
            Returns:
                tuple: transformation metrics from 512X512 to 256X256
        """
        ref_time = round(float(self.frame_state.ref_frame * sg.TIME_INCREMENT_STEP), 1)
        ref_main_x = self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][ref_time][1].center_main_x_p
        ref_main_y = self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][ref_time][1].center_main_y_p
        translation_from_main_to_sub = (sg.SUB_IMAGE_WIDTH / 2 - ref_main_x, sg.SUB_IMAGE_LENGTH / 2 - ref_main_y)

        if sg.DEBUG == sg.DEBUG_LEVEL_2:
            print(
                " Translation matrix at Ref time is ",
                translation_from_main_to_sub[0],
                translation_from_main_to_sub[1],
                ref_time,
            )

        return translation_from_main_to_sub

    ####################################################################

    def save_sub_positions(self, frame):
        """ Save transformed camera view positions of the sdv, traffic and paths
            Args:
                frame (int) : Current frame number
            Returns:
                tuple: transformation metrics from 512X512 to 256X256
        """
        cur_time = round(frame * sg.TIME_INCREMENT_STEP, 1)

        # Update the heading angle
        self.update_heading_angle(cur_time)
        # Get the translation matrics here
        trans_from_main_to_sub = self.get_translation_metrics()

        self.save_sub_cars_pos(cur_time, trans_from_main_to_sub)
        self.save_sub_paths_pos(cur_time, trans_from_main_to_sub)

        return trans_from_main_to_sub

    ####################################################################
    def get_updated_heading_angle(self, car_seq_no, sign_cur_angle, car_turn, cur_angle):
        pos_cur_ang = 1
        neg_cur_ang = 0
        pi_over_2 = math.pi / 2

        heading_angle_dict = {
            sg.CAR_SEQ_1: {
                pos_cur_ang: {
                    CarTurn.LEFT.value: -cur_angle,
                    CarTurn.RIGHT.value: cur_angle,
                    CarTurn.NO.value: cur_angle,
                },
                neg_cur_ang: {
                    CarTurn.LEFT.value: cur_angle,
                    CarTurn.RIGHT.value: cur_angle,
                    CarTurn.NO.value: cur_angle,
                },
            },
            sg.CAR_SEQ_2: {
                pos_cur_ang: {
                    CarTurn.LEFT.value: pi_over_2 - cur_angle,
                    CarTurn.RIGHT.value: pi_over_2 + cur_angle,
                    CarTurn.NO.value: cur_angle,
                },
                neg_cur_ang: {
                    CarTurn.LEFT.value: pi_over_2,
                    CarTurn.RIGHT.value: pi_over_2,
                    CarTurn.NO.value: pi_over_2,
                },
            },
            sg.CAR_SEQ_3: {
                pos_cur_ang: {
                    CarTurn.LEFT.value: math.pi - cur_angle,
                    CarTurn.RIGHT.value: -math.pi + cur_angle,
                    CarTurn.NO.value: cur_angle,
                },
                neg_cur_ang: {CarTurn.LEFT.value: math.pi, CarTurn.RIGHT.value: math.pi, CarTurn.NO.value: math.pi},
            },
            sg.CAR_SEQ_4: {
                pos_cur_ang: {
                    CarTurn.LEFT.value: -pi_over_2 - cur_angle,
                    CarTurn.RIGHT.value: -pi_over_2 + cur_angle,
                    CarTurn.NO.value: cur_angle,
                },
                neg_cur_ang: {
                    CarTurn.LEFT.value: -pi_over_2,
                    CarTurn.RIGHT.value: -pi_over_2,
                    CarTurn.NO.value: -pi_over_2,
                },
            },
        }

        return heading_angle_dict[car_seq_no][sign_cur_angle][car_turn]

    ####################################################################
    def update_heading_angle(self, cur_time):
        """ Update the heading angle at current time. it has range [-pi, pi]
            Args:
                cur_time (float) : Current simulation time
        """
        for car in self.car_list:
            cur_ang = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad

            sign_cur_angle = 1 if (cur_ang > 0) else 0

            updated_cur_ang = self.get_updated_heading_angle(car.sim_state.seq, sign_cur_angle, car.sim.turn, cur_ang)

            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][
                1
            ].rot_heading_rad = updated_cur_ang

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " Heading angle for car seq # ",
                    self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad,
                    car.sim_state.seq,
                )

    ####################################################################

    def save_sub_cars_pos(self, cur_time, transform_main_to_sub):
        """ Save the camera view car's positions
            Args:
                cur_time (float)              : Current time
                transform_main_to_sub (tuple) : Translation metrics from main to camera view frame
        """
        for car in self.car_list:
            cur_main_x_0 = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_main_x_p
            cur_main_y_0 = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_main_y_p

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " car # ",
                    car.seq,
                    " has main . x position ",
                    cur_main_x_0,
                    " and y position ",
                    cur_main_y_0,
                    "at time step ",
                    cur_time,
                )

            cur_main_mid_x_0 = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][
                1
            ].center_main_x_p
            cur_main_mid_y_0 = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][
                1
            ].center_main_y_p

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " car # ",
                    car.sim_state.seq,
                    " has main . mid x position ",
                    cur_main_mid_x_0,
                    " and mid y position ",
                    cur_main_mid_y_0,
                    "at time step ",
                    cur_time,
                )

            cur_sub_x_0 = cur_main_x_0 + transform_main_to_sub[0]
            cur_sub_y_0 = cur_main_y_0 + transform_main_to_sub[1]

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " car # ",
                    car.sim_state.seq,
                    " has sub . x position ",
                    cur_sub_x_0,
                    " and y position ",
                    cur_sub_y_0,
                    " at time step ",
                    cur_time,
                )

            cur_sub_center_x = cur_main_mid_x_0 + transform_main_to_sub[0]
            cur_sub_center_y = cur_main_mid_y_0 + transform_main_to_sub[1]

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " car # ",
                    car.sim_state.seq,
                    " has sub . mid x position ",
                    cur_sub_center_x,
                    " and mid y position ",
                    cur_sub_center_y,
                    " at time step ",
                    cur_time,
                )

            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_x_p = cur_sub_x_0
            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_y_p = cur_sub_y_0

            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].center_x_p = cur_sub_center_x
            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].center_y_p = cur_sub_center_y

    ####################################################################

    def save_sub_paths_pos(self, cur_time, transform_main_to_sub):
        """ Save the camera view path's positions
            Args:
                cur_time(float)              : Current time
                transform_main_to_sub(tuple) : Translation metrics from main to camera view frame
        """
        for i in range(sg.NUMBER_OF_PATHS):
            self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2].stop_line_states[
                i
            ].center_x_p += transform_main_to_sub[0]

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " sub path x position s",
                    self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2]
                    .stop_line_states[i]
                    .center_x_p,
                )

            self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2].stop_line_states[
                i
            ].center_y_p += transform_main_to_sub[1]

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " sub path y position s",
                    self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2]
                    .stop_line_states[i]
                    .center_y_p,
                )

    ####################################################################

    def update_from_main_to_sub(self, window, frame, image_name):
        """ Update the rotated positions in case of turn
            Args:
                window (pygame window) : Current window
                frame (int)            : Current Frame number
                image_name (string)    : Image name
            Returns:
                sub_window_image(numpy): Current 256X256 window image
                sub_mask_img(numpy)    : Current 256X256 masked image
                ref_car_end(bool)      : True if Car has recahed the end
        """
        subimg = SubImage()

        cur_time = round(frame * sg.TIME_INCREMENT_STEP, 1)

        frame_division = 10.0

        # Get the reference x and y position of camera
        camera_pos_x, camera_pos_y, heading_ang_rad = self.camera.get_camera_cur_pos(
            round(float(frame * sg.TIME_INCREMENT_STEP), 1),
            (self.frame_state.ref_frame / frame_division),
            self.save_sim_flow_data,
        )

        if sg.DEBUG == sg.DEBUG_LEVEL_2:
            print(
                " The position of camera at mid frame ",
                camera_pos_x,
                camera_pos_y,
                " and heading angle in radian ",
                heading_ang_rad,
            )

        rotate_img_trns = RotateImage(self.car_list, self.path_list, self.save_sim_flow_data, heading_ang_rad)

        # Images without rotation
        sub_window_image, ref_car_end = subimg.create_subimage(window, camera_pos_x, camera_pos_y)
        sub_mask_img = subimg.create_sub_mask_image(sub_window_image)
        sub_window_image, sub_mask_img = rotate_img_trns.get_rotated_images(cur_time, sub_window_image, sub_mask_img)

        # Save once at the end of drawing traffic
        if image_name == sg.TRAFFIC_IMAGE_KEYWORD:
            trans_from_main_to_sub = self.save_sub_positions(frame)
            # Get the rotated images, if reference car is taking a turn
            rotate_img_trns.save_rotated_positions(cur_time, trans_from_main_to_sub)

        return sub_window_image, sub_mask_img, ref_car_end

    #####################################################################
