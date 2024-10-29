####################################################################
# Uber, Inc. (c) 2019
# Description: This is the main game loop
#####################################################################
import math

import cv2
import numpy as np
import stop_and_go_globals as sg
from stop_and_go_data_type import CarTurn

##############################################################


class RotateImage(object):
    """ RotateImage class to rotate the image at reference time """

    def __init__(self, car_list, path_list, save_sim_flow_data, heading_ang_rad):
        """ Initialize the RotateImage object
            Args:
                car_list(list)           : List of car objects
                path_list(list)          : List of path objects
                save_sim_flow_data(dict) : Map of the simulation data information
                heading_ang_rad(rad)     : Car's heading angle in radian
        """
        self.car_list = car_list
        self.path_list = path_list
        self.save_sim_flow_data = save_sim_flow_data
        self.heading_ang_rad = heading_ang_rad

    ##############################################################

    def get_rotated_images(self, cur_time, sub_window_image, sub_mask_img):
        """ Save the rotated positions of the cars and paths
            Args:
                cur_time(float)         : Current time
                sub_window_image(numpy) : Frame with path, car, intersection objects
                sub_mask_img(numpy)     : Masked image of size 256 X 256
            Returns:
                numpy                   : Returns the rotated window image
                numpy                   : Returns the rotated masked image
        """
        # Rotation of images
        pi_angle = 180

        heading_ang_degree = self.heading_ang_rad * pi_angle / math.pi

        if self.car_list[sg.REFERENCE_CAR_SEQ - 1].sim.turn != CarTurn.NO.value:
            if self.car_list[sg.REFERENCE_CAR_SEQ - 1].sim.turn == CarTurn.LEFT.value:
                heading_ang_degree = -heading_ang_degree

            # Get the rotated subimage
            rotated_subimage = self.get_rotated_image(sub_window_image, heading_ang_degree)
            # Get the rotated sub mask image
            rotated_sub_mask_image = self.get_rotated_image(sub_mask_img, heading_ang_degree)

            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(" heading_angle_radian, heading_ang_degree = ", self.heading_ang_rad, heading_ang_degree)

            # Update the sub_mask_img in case of rotation
            sub_window_image = rotated_subimage
            sub_mask_img = rotated_sub_mask_image

        return sub_window_image, sub_mask_img

    ##############################################################

    def save_rotated_positions(self, cur_time, trans_from_main_to_sub):
        """ Save the rotated positions of the cars and paths
            Args:
                cur_time(float)               : Current time
                trans_from_main_to_sub(tuple) : Transformation metrics from 512X512 to 256X256
        """
        # Rotation of images
        if self.car_list[sg.REFERENCE_CAR_SEQ - 1].sim.turn != CarTurn.NO.value:
            # Get the new rotated points, to make reference car to point to the positive x direction
            self.save_rotated_cars_positions(cur_time, trans_from_main_to_sub)
            # Get the new rotated point, to make the reference car to point ot the positive x direction
            self.save_rotated_paths_positions(cur_time, trans_from_main_to_sub)
            # Update the rotated angle
            self.update_rotated_heading_angle(cur_time)

    ##############################################################

    def get_rotated_image(self, sub_window_image, theta):
        """ Get the rotated image
            Args:
                sub_window_image(numpy) : Image with current path, intersection & car objects
                theta (radian)          : Rotate image with the angle
            Returns:
                 numpy                  : Returns the rotated image
        """
        rows, cols = sub_window_image.shape[:2]

        # Rotation around the center in case of 1st av as reference
        mapping = cv2.getRotationMatrix2D((cols / 2, rows / 2), -theta, 1)

        return cv2.warpAffine(
            sub_window_image,
            mapping,
            (sg.SUB_IMAGE_WIDTH, sg.SUB_IMAGE_LENGTH),
            flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP,
            borderValue=0,
        )

    ##############################################################

    def update_rotated_heading_angle(self, cur_time):
        """ Update the rotated heading angle to make it in range of [-pi, pi]
            Args:
                cur_time(float) : Current time
        """
        for car in self.car_list:
            if self.car_list[sg.REFERENCE_CAR_SEQ - 1].sim.turn == CarTurn.LEFT.value:
                self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][
                    1
                ].rot_heading_rad += self.heading_ang_rad
            else:
                self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][
                    1
                ].rot_heading_rad -= self.heading_ang_rad

            # Possible case when car is at 3.14 and further rotation pushed it into 3.14+theta

            # We need to maintain values between [-pi, pi]

            if self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad > math.pi:
                self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad = (
                    -2 * math.pi
                    + self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad
                )
            if (
                self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad
                < -math.pi
            ):
                self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad = (
                    2 * math.pi
                    + self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad
                )

            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " Rotated heading angle for car.sim_state.seq # ",
                    self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].rot_heading_rad,
                    car.sim_state.seq,
                )

    ############################################################

    def save_rotated_cars_positions(self, cur_time, translation_from_main_to_sub):
        """ Save the rotated car's positions & center into map save_sim_flow_data
            Args:
                cur_time(float) : Current time
                translation_from_main_to_sub(tuple) : Transformation metrics from 512X512 coordinates to 256X256
        """
        theta_rad = self.heading_ang_rad

        for car in self.car_list:
            cur_sub_x = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_x_p
            cur_sub_y = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_y_p

            cur_sub_mid_x = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].center_x_p
            cur_sub_mid_y = self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].center_y_p
            # Rotating in clockwise direction
            if self.car_list[sg.REFERENCE_CAR_SEQ - 1].sim.turn == CarTurn.RIGHT.value:
                theta = -theta_rad
            else:
                theta = theta_rad

            cur_rotated_sub_x, cur_rotated_sub_y = self.get_point_new_location(cur_sub_x, cur_sub_y, theta)
            cur_rotated_sub_mid_x, cur_rotated_sub_mid_y = self.get_point_new_location(
                cur_sub_mid_x, cur_sub_mid_y, theta
            )

            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_x_p = cur_rotated_sub_x
            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][1].loc_y_p = cur_rotated_sub_y

            # For mid point of the car #################
            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(" Generated roated Points for the mid point ")

            # Rotating in clockwise direction
            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][
                1
            ].center_x_p = cur_rotated_sub_mid_x
            self.save_sim_flow_data.sim_data_dict_list[car.sim_state.seq - 1][cur_time][
                1
            ].center_y_p = cur_rotated_sub_mid_y

    ##############################################################

    def save_rotated_paths_positions(self, cur_time, translation_from_main_to_sub):
        """ Save the rotated path's positions into map save_sim_flow_data
            Args:
                cur_time(float) : Current Time
                translation_from_main_to_sub(tuple) : Transformation metrics from 512X512 coordinates to 256X256
        """
        theta_rad = self.heading_ang_rad

        # for car in self.car_list:
        for i in range(sg.NUMBER_OF_PATHS):
            cur_stop_sub_x = (
                self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2]
                .stop_line_states[i]
                .center_x_p
            )
            cur_stop_sub_y = (
                self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2]
                .stop_line_states[i]
                .center_y_p
            )
            if self.car_list[sg.REFERENCE_CAR_SEQ - 1].sim.turn == CarTurn.RIGHT.value:
                cur_rotated_stop_sub_x, cur_rotated_stop_sub_y = self.get_point_new_location(
                    cur_stop_sub_x, cur_stop_sub_y, -theta_rad
                )
            else:
                cur_rotated_stop_sub_x, cur_rotated_stop_sub_y = self.get_point_new_location(
                    cur_stop_sub_x, cur_stop_sub_y, theta_rad
                )

            # Saving new rotated data points
            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(
                    " Rotated path's sub positions at angle ", cur_rotated_stop_sub_x, cur_rotated_stop_sub_y, theta_rad
                )

            self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2].stop_line_states[
                i
            ].center_x_p = cur_rotated_stop_sub_x
            self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2].stop_line_states[
                i
            ].center_y_p = cur_rotated_stop_sub_y

    ##############################################################

    def get_point_new_location(self, x, y, theta):
        """ Get the new point location after rotation by an angle
            Args:
                x(float)      : x-coordinate of the point
                y(flaot)      : y-coordinate of the point
                theta(radian) : Rotate the point around the angle
            Returns:
                float         : New rotated x coordinate
                float         : New rotated y coordinate
        """
        sub_img_by_2 = sg.SUB_IMAGE_WIDTH / 2

        x_new = (x - sub_img_by_2) * np.cos(theta) - (y - sub_img_by_2) * np.sin(theta) + sub_img_by_2
        y_new = (x - sub_img_by_2) * np.sin(theta) + (y - sub_img_by_2) * np.cos(theta) + sub_img_by_2

        return x_new, y_new

    #####################################################################
