###################################################################
# Uber, Inc. (c) 2019
# Description: This is the main game loop
#####################################################################
import cv2
import numpy as np
import pygame
import stop_and_go_globals as sg

#####################################################################


class SubImage(object):
    #####################################################################

    def update_subimage_outside(self, sub_end, mask_end):
        """ Update the subimage creation if it is outside the frame.
            Returns:
                Returns updated sub_end and mask_end pixels to get the actual and masked subimages
        """
        mask_offset = sub_end - sg.WINDOW_WIDTH_PIXELS
        mask_end = mask_end - mask_offset - 1
        sub_end = sg.WINDOW_WIDTH_PIXELS

        return sub_end, mask_end

    #####################################################################

    def update_subimage_inside(self, sub_start, mask_start):
        """ Update the subimage creation if it's start is outside the frame.
            Returns:
                Returns updated sub_start and mask_start to get the masked and actual subimages
        """
        mask_start = 0 - sub_start
        sub_start = 0

        return sub_start, mask_start

    #####################################################################

    def image_end_status(self, ref_mid_x, ref_mid_y):
        """ Check if the subimage is outside the frame
            Returns:
                Returns True if reference car has recahed the end
        """
        # Referenced car has reached the end
        image_end = (
            (ref_mid_x < 0)
            or (ref_mid_x > sg.WINDOW_WIDTH_PIXELS)
            or (ref_mid_y < 0)
            or (ref_mid_y > sg.WINDOW_LENGTH_PIXELS)
        )

        return image_end

    #####################################################################

    def visualize_subimage(self, sub_window1):
        """ Testing utility to visualize the image
            Args:
                sub_window1(numpy) : Given sub image as numpy array
        """
        # Put the reference inthe middle
        sub_window1[:, sg.SUB_IMAGE_WIDTH / 2] = (0, 0, 255)
        sub_window1[sg.SUB_IMAGE_WIDTH / 2, :] = (0, 0, 255)

        # Draw the boundary for visualization
        sub_window1[0:2, :] = (0, 0, 255)
        sub_window1[sg.SUB_IMAGE_WIDTH - 1 : -1, :] = (0, 0, 255)
        sub_window1[:, 0:2] = (0, 0, 255)
        sub_window1[:, sg.SUB_IMAGE_WIDTH - 1 : -1] = (0, 0, 255)

    ####################################################################

    def convert_surface_3darray(self, window):
        """ Convert pygame surface to the 3-D array
            Args:
                window(pygame surface) : Current pygame frame
            Returns:
                Returns the window as 3-D array
        """
        window_arr = pygame.surfarray.array3d(window)
        # Swap height width
        window_arr = window_arr.swapaxes(0, 1)

        # Convert to RGB
        window_arr = window_arr[:, :, ::-1]
        return window_arr

    #####################################################################

    def create_subimage(self, window, ref_mid_x, ref_mid_y):
        """ Create subimage from the main window frame.
            Args:
                window(pygame window) : Current frame
                ref_mid_x(float)      : x position of the reference coordinate
                ref_mid_y(float)      : y position of the reference coordinate
            Returns:
                numpy                 : Generated sub image of size 256X256 pixels
                bool                  : Reference car has recahed end before creating sub image
        """
        ref_car_end = False
        sub_image_outside_main_view = False
        gray = self.convert_surface_3darray(window)

        sub_window1 = np.zeros(shape=(sg.SUB_IMAGE_WIDTH, sg.SUB_IMAGE_LENGTH, 3), dtype=np.uint8)

        # Actual array indexes of the camera view
        sub_start_i = ref_mid_y - sg.SUB_IMAGE_WIDTH / 2
        sub_end_i = ref_mid_y + sg.SUB_IMAGE_WIDTH / 2
        sub_start_j = ref_mid_x - sg.SUB_IMAGE_LENGTH / 2
        sub_end_j = ref_mid_x + sg.SUB_IMAGE_LENGTH / 2

        image_end = self.image_end_status(ref_mid_x, ref_mid_y)
        # If reached the end restart the experiment
        if image_end:
            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(" Image end is reached ")
            ref_car_end = True
            return sub_window1, ref_car_end

        # Initialize mask variables
        mask_start_i = 0
        mask_end_i = sg.SUB_IMAGE_LENGTH
        mask_start_j = 0
        mask_end_j = sg.SUB_IMAGE_WIDTH

        # If sub-window image is out of main window in horizontal direction
        # Never start with gray images
        if sub_end_j > sg.WINDOW_WIDTH_PIXELS:
            sub_image_outside_main_view = True
            sub_end_j, mask_end_j = self.update_subimage_outside(sub_end_j, mask_end_j)

        # If sub-window image is less than the main window in horizontal direction
        if sub_start_j < 0:
            sub_image_outside_main_view = True
            sub_start_j, mask_start_j = self.update_subimage_inside(sub_start_j, mask_start_j)

        # If sub-window image is out of the main window in vertical direction
        if sub_end_i > sg.WINDOW_LENGTH_PIXELS:
            sub_image_outside_main_view = True
            sub_end_i, mask_end_i = self.update_subimage_outside(sub_end_i, mask_end_i)

        # If sub-window image is less than the main window in vertical direction
        if sub_start_i < 0:
            sub_image_outside_main_view = True
            sub_start_i, mask_start_i = self.update_subimage_inside(sub_start_i, mask_start_i)

        if sub_image_outside_main_view:
            sub_window1[int(mask_start_i) : int(mask_end_i) + 1, int(mask_start_j) : int(mask_end_j) + 1] = gray[
                int(sub_start_i) : int(sub_end_i), int(sub_start_j) : int(sub_end_j)
            ]

            # TESTING
            if sg.TESTING:
                self.visualize_subimage(sub_window1)

            return sub_window1, ref_car_end

        # Put the reference line in the middle
        # TESTING
        if sg.TESTING:
            gray[:, int(ref_mid_x)] = (0, 0, sg.WHITE_PIXEL)
            gray[int(ref_mid_y), :] = (0, 0, sg.WHITE_PIXEL)

        # Populate the image in case of general scenario
        return gray[int(sub_start_i) : int(sub_end_i), int(sub_start_j) : int(sub_end_j)], ref_car_end

    #####################################################################

    def create_sub_mask_image(self, mask_image):
        """ Create masked image
            Args:
                mask_image(numpy)     : Masked image of Sub image of size 256X256 pixels.
            Returns:
                numpy                 : Returns the masked image of size 256X256 pixels.
        """
        mask_image = cv2.cvtColor(mask_image, cv2.COLOR_BGR2GRAY)

        # Keep the order of pixels to generate mask image
        mask_image[mask_image > sg.SUB_IMAGE_UPPER_MASK] = 0
        mask_image[mask_image < sg.SUB_IMAGE_LOWER_MASK] = 0
        mask_image[(mask_image > sg.SUB_IMAGE_LOWER_MASK) & (mask_image < sg.SUB_IMAGE_UPPER_MASK)] = sg.WHITE_PIXEL

        return mask_image

    #####################################################################
