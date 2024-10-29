####################################################
# Uber, Inc. (c) 2019
####################################################
import cv2
import numpy as np
import pygame
import stop_and_go_globals as sg
from stop_and_go_actors import Car, Path, Stop_Area, Stop_Line
from stop_and_go_cord_transform import CoordinateTransform
from stop_and_go_data import JsonFileManager
from stop_and_go_data_generation import DatasetGenerator
from stop_and_go_data_type import CarTurn
from stop_and_go_sim import Config_manager, Sim

####################################################

jsonManager = JsonFileManager(sg.OUPUT_JSON_FILENAME)

######################################################################


class Drawer(object):
    """ Draw utility to draw sdv, traffic and paths """

    def __init__(
        self,
        exp_no,
        car_list,
        path_list,
        stop_line_list,
        Sprite_mid,
        camera,
        frame_state,
        dataset_storage_choice,
        save_sim_flow_data,
    ):

        self.exp_no = exp_no
        self.car_list = car_list
        self.path_list = path_list
        self.stop_line_list = stop_line_list
        self.Sprite_mid = Sprite_mid
        self.camera = camera
        self.frame_state = frame_state
        self.save_sim_flow_data = save_sim_flow_data
        self.dataset_generator = DatasetGenerator(save_sim_flow_data, dataset_storage_choice)

    ######################################################################

    def draw_sdv(self, window, sub_seq_no, frame, no_car=False):
        """ Draw utility to draw sdv and draw camera view subimage of the sdv.
            Args:
                window(pygame window) : Current Frame
                sub_seq_no(int)       : Current Image number of the given iteration number
                frame(int)            : Current frame number
                no_car(bool)          : Flag to indicate if it is car's lane
        """
        self.draw_sdv_on_frame(window, frame)

        # Create camera view image
        if sg.GENERATE_SUBIMAGE:
            self.draw_camera_view_subimages(window, sub_seq_no, frame, no_car, sg.REFERENCE_IMAGE_KEYWORD)

    ######################################################################

    def draw_sdv_on_frame(self, window, frame):
        """ Get the boundary points of the cars from save_sim_flow_data to draw.
            Args:
                window(pygame window): Current Frame
                frame(int)           : Current frame number
        """
        # Draw only sdv
        color = sg.BLUE

        cur_time = round(frame * sg.TIME_INCREMENT_STEP, 1)

        boundary_points = self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][1].boundary

        pygame.draw.polygon(window, color, boundary_points, 0)

    ######################################################################

    def draw_traffic(self, window, sub_seq_no, frame, no_car=False):
        """ Draw utility to draw the traffic other than the reference car (SDV).
            Args:
                window(pygame window) : Current Frame
                sub_seq_no(int)       : Current Image number of the given iteration number
                frame(int)            : Current frame number
                no_car(bool)          : Flag to indicate if it is car's lane
        """
        image_name = sg.TRAFFIC_IMAGE_KEYWORD

        self.draw_traffic_on_frame(window, frame)
        # Create camera view image
        if sg.GENERATE_SUBIMAGE:
            self.draw_camera_view_subimages(window, sub_seq_no, frame, no_car, image_name)

    ######################################################################

    def draw_traffic_on_frame(self, window, frame):
        """ Get the boundary points for the traffic car to draw it.
            Args:
                window(pygame window): Current Frame
                frame(int)           : Current frame number
        """
        # Draw cars other than sdv
        color = sg.BLUE

        cur_time = round(frame * sg.TIME_INCREMENT_STEP, 1)

        for num_car in range(self.save_sim_flow_data.car_nums):

            if num_car != (sg.REFERENCE_CAR_SEQ - 1):

                boundary_points = self.save_sim_flow_data.sim_data_dict_list[num_car][cur_time][1].boundary

                pygame.draw.polygon(window, color, boundary_points, 0)

    #################################################################################

    def draw_cars_on_frame(self, window, frame):
        """ Get the boundary points for all the cars ( sdv + traffic ) to draw it.
            Args:
                window(pygame window) : Current Frame
                frame(int)            : Current frame number
        """
        # Draw cars
        color = sg.BLUE

        cur_time = round(frame * sg.TIME_INCREMENT_STEP, 1)

        for num_car in range(self.save_sim_flow_data.car_nums):

            boundary_points = self.save_sim_flow_data.sim_data_dict_list[num_car][cur_time][1].boundary

            pygame.draw.polygon(window, color, boundary_points, 0)

    #####################################################################

    def draw_path_on_frame(self, window, is_green=False):
        """ Draw all the paths.
            Args :
                window(pygame window) : Current Frame
                is_green(bool)        : Flag to indicate the color of the path is green
        """
        for path_seq in range(len(self.path_list)):
            self.path_list[path_seq].render_path(window, is_green)

    #####################################################################

    def draw_stop_lines(self, window):
        """ Draw all the stop lines.
            Args :
                window(pygame window) : Current Frame
        """
        for index in range(len(self.stop_line_list)):
            self.stop_line_list[index].draw(window)

    #####################################################################

    def draw_window(self, window, sub_seq_no, frame, no_car=False):
        """ Draw camera view subimage for path, cars and the intersection area.
            Args:
                window(pygame window) : Current Frame
                sub_seq_no(int)       : Current Image number of the given iteration number
                frame(int)            : Current frame number
                no_car(bool)          : Flag to indicate if there is car
        """
        self.draw_path_on_frame(window, no_car)

        # To draw the middle stop area
        if sg.SPRITE_AREA:
            self.Sprite_mid.render(no_car)

        self.draw_stop_lines(window)
        if not no_car:
            self.draw_cars_on_frame(window, frame)

        # Create camera view subimage
        self.draw_camera_view_subimages(window, sub_seq_no, frame, no_car, image_name="_mask_")

    #####################################################################

    def draw_all_traffic(self, window, sub_seq_no, dataset_storage_choice, tetrys_content_list):
        """ if DISPLAY_TRAFFIC is true draw the window else create images for sdv and traffic.
            Args:
                window(pygame window) : Current Frame
                sub_seq_no(int)       : Current Image number of the given iteration number
            Returns:
                bool                  : Flag to reset the experiment number
        """
        reset_frames_exp = True

        for frame in range(self.frame_state.start_frame, self.frame_state.end_frame):
            if sg.DISPLAY_TRAFFIC:
                self.draw_window(window, sub_seq_no, frame)
            else:
                self.draw_sdv(window, sub_seq_no, frame)
                # Reset the window
                window.fill((0, 0, 0))

                self.draw_traffic(window, sub_seq_no, frame)

                cur_time = round(frame * sg.TIME_INCREMENT_STEP, 1)
                reset_frames_exp = self.check_valid_stop_lines(cur_time)
                if reset_frames_exp:
                    frame_idx = frame - self.frame_state.start_frame
                    self.dataset_generator.write_file_content(
                        self.exp_no, sub_seq_no, frame_idx, cur_time, self.car_list, tetrys_content_list
                    )
                else:
                    return reset_frames_exp

            window.fill((0, 0, 0))
            pygame.display.flip()
            sub_seq_no += 1

            # At the end Draw the local map
            self.draw_window(window, sub_seq_no, frame, True)

        return reset_frames_exp

    ######################################################################

    def check_valid_stop_lines(self, cur_time):
        """ Check the stop lines coordinates.
            Args:
                cur_time(float): Current simulation time
            Returns:
                bool           : Returns reset_frames_exp as True to reset this experiment
        """
        for i in range(sg.NUMBER_OF_PATHS):
            center_x_p = (
                self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2]
                .stop_line_states[i]
                .center_x_p
            )
            center_y_p = (
                self.save_sim_flow_data.sim_data_dict_list[sg.REFERENCE_CAR_SEQ - 1][cur_time][2]
                .stop_line_states[i]
                .center_y_p
            )
            if (
                (center_x_p > sg.STOP_LINE_GENERATE_MAX)
                or (center_y_p > sg.STOP_LINE_GENERATE_MAX)
                or (center_x_p < sg.STOP_LINE_GENERATE_MIN)
                or (center_y_p < sg.STOP_LINE_GENERATE_MIN)
            ):
                return False

        return True

    #####################################################################

    def draw_camera_view_subimages(self, window, sub_seq_no, frame, no_car, image_name):
        """ Draw the camera view subimages for the SDV ( Reference car ) and tarffic.
            Args:
                window(pygame window) : Current Frame
                sub_seq_no(int)       : Current Image number of the given iteration number
                frame(int)            : Current frame number
                no_car(bool)          : Flag to indicate to start drawing the lanes on frame
                image_name(string)    : Image name
        """
        # Prepare string for experiment number
        exp1 = str(self.exp_no)
        exp2 = exp1.zfill(5)
        # Prepare string for iteration number
        str1 = str(sub_seq_no - self.frame_state.start_frame)
        str2 = str1.zfill(6)

        # Update the sub positions
        cord_trns = CoordinateTransform(
            self.exp_no,
            sub_seq_no,
            self.camera,
            self.car_list,
            self.path_list,
            self.frame_state,
            self.save_sim_flow_data,
        )

        sub_window_image, sub_mask_img, ref_car_end = cord_trns.update_from_main_to_sub(window, frame, image_name)

        sub_image_name = sg.IMAGE_BASE_DIR + "/" + "stop_" + exp2 + "_sub_" + str2 + ".jpg"

        # Binary image name
        sub_mask_img_name = sg.IMAGE_BASE_DIR + "/" + "stop_" + exp2 + image_name + str2 + ".jpg"

        if not ref_car_end:
            if (sub_seq_no >= self.frame_state.start_frame) and (sub_seq_no <= self.frame_state.end_frame):
                if not no_car:
                    if sg.DISPLAY_TRAFFIC:
                        cv2.imwrite(sub_image_name, sub_window_image)
                        cv2.imwrite(sub_mask_img_name, sub_mask_img)
                    else:
                        # Create camera view masked image
                        cv2.imwrite(sub_mask_img_name, sub_mask_img)

        if no_car:
            # Creating the local map image
            local_map_img_name = sg.IMAGE_BASE_DIR + "/" + "stop_" + exp2 + "_LANES_000000" + ".jpg"
            sub_window_image[:, :, 0] = 0
            cv2.imwrite(local_map_img_name, sub_window_image)


#####################################################################


class Generator(object):
    """ GENERATE frame's objects for the  simulation """

    def generate_paths(self):
        """ Generate the 4 path objetcs later to draw it on the frame.
            Returns:
                list : Returns the generated paths list
        """
        path_list = []

        x1 = sg.WINDOW_WIDTH_PIXELS / 2
        y1 = sg.WINDOW_LENGTH_PIXELS / 2
        initial_x = 0
        initial_y = y1 + sg.LANE_BUFFER_PIXELS / 2

        path1 = Path(initial_x, initial_y, sg.WINDOW_WIDTH_PIXELS, initial_y, 0, initial_y)

        path_list.append(path1)

        initial_x = x1 - sg.LANE_BUFFER_PIXELS / 2
        initial_y = 0

        path2 = Path(initial_x, initial_y, initial_x, sg.WINDOW_LENGTH_PIXELS, 1, initial_x)
        path_list.append(path2)

        initial_x = sg.WINDOW_WIDTH_PIXELS
        initial_y = y1 - sg.LANE_BUFFER_PIXELS / 2

        path3 = Path(initial_x, initial_y, 0, initial_y, 2, initial_y)

        path_list.append(path3)

        initial_x = x1 + sg.LANE_BUFFER_PIXELS / 2
        initial_y = sg.WINDOW_LENGTH_PIXELS

        path4 = Path(initial_x, initial_y, initial_x, 0, 3, initial_x)

        path_list.append(path4)

        return path_list

    ####################################################################

    def generate_stop_lines(self, Path_list):
        """ Generate the 4 stop lines later to draw it on the frame.
            Args:
                Path_list(list) : Contains the path objects
            Returns:
                list : Returns the generated stop_lines list
        """
        x1 = sg.WINDOW_WIDTH_PIXELS / 2
        y1 = sg.WINDOW_LENGTH_PIXELS / 2

        # First stop line in forward direction
        stop_line1_y = Path_list[sg.PATH_SEQ_0].start[1]
        stop_line1 = Stop_Line(
            x1 - sg.STOP_LINE_LEN_OFFSET,
            stop_line1_y - sg.STOP_LINE_HOR_PIXELS / 2,
            x1 - sg.STOP_LINE_LEN_OFFSET,
            stop_line1_y + sg.STOP_LINE_HOR_PIXELS / 2,
            sg.STOP_LINE_WIDTH,
        )

        # Second Stop line in downward vertical left direction
        stop_line2_x = Path_list[sg.PATH_SEQ_1].start[0]
        stop_line2 = Stop_Line(
            stop_line2_x - sg.STOP_LINE_HOR_PIXELS / 2,
            y1 - sg.STOP_LINE_LEN_OFFSET,
            stop_line2_x + sg.STOP_LINE_HOR_PIXELS / 2,
            y1 - sg.STOP_LINE_LEN_OFFSET,
            sg.STOP_LINE_WIDTH,
        )

        # Third stop line in downward vertical right direction
        stop_line3_y = Path_list[sg.PATH_SEQ_2].start[1]
        stop_line3 = Stop_Line(
            x1 + sg.STOP_LINE_LEN_OFFSET,
            stop_line3_y - sg.STOP_LINE_HOR_PIXELS / 2,
            x1 + sg.STOP_LINE_LEN_OFFSET,
            stop_line3_y + sg.STOP_LINE_HOR_PIXELS / 2,
            sg.STOP_LINE_WIDTH,
        )

        # Fourth stop line in backward direction
        stop_line4_x = Path_list[sg.PATH_SEQ_3].start[0]
        stop_line4 = Stop_Line(
            stop_line4_x - sg.STOP_LINE_HOR_PIXELS / 2,
            y1 + sg.STOP_LINE_LEN_OFFSET,
            stop_line4_x + sg.STOP_LINE_HOR_PIXELS / 2,
            y1 + sg.STOP_LINE_LEN_OFFSET,
            sg.STOP_LINE_WIDTH,
        )

        stop_line_lists = [stop_line1, stop_line2, stop_line3, stop_line4]

        return stop_line_lists

    ####################################################################

    def generate_car_instances(self, seq, path_list, sim, window):
        """ Generate the car instances to draw it on the frame.
            Args:
                seq(int)              : Sequence number of the car instance
                path_list(list)       : List of paths contain path objects
                sim(object)           : Sim class object contains the simulation info for car instance
                window(pygame window) : Current frame
            Returns:
                object                : Returns the generated car instance
        """
        initial_x = 0
        initial_y = 0

        # Car1 moving along x-direction
        if seq == sg.CAR_SEQ_1:
            initial_x = 0
            initial_y = path_list[seq - 1].start[1] - sg.CAR_WIDTH_PIXELS / 2

        # Car2
        elif seq == sg.CAR_SEQ_2:
            initial_x = path_list[seq - 1].start[0] - sg.CAR_WIDTH_PIXELS / 2
            initial_y = 0

        # Car3
        elif seq == sg.CAR_SEQ_3:
            initial_x = sg.WINDOW_WIDTH_PIXELS - sg.CAR_LENGTH_PIXELS
            initial_y = path_list[seq - 1].start[1] - sg.CAR_WIDTH_PIXELS / 2

        # Car4
        elif seq == sg.CAR_SEQ_4:
            initial_x = path_list[seq - 1].start[0] - sg.CAR_WIDTH_PIXELS / 2
            initial_y = sg.WINDOW_LENGTH_PIXELS - sg.CAR_LENGTH_PIXELS

        if (seq == sg.CAR_SEQ_1) or (seq == sg.CAR_SEQ_3):
            car = Car(initial_x, initial_y, sg.CAR_LENGTH_PIXELS, sg.CAR_WIDTH_PIXELS, seq, sim, 0, window)

            # Update the turns in case of testing
            if sg.TESTING:
                if seq == sg.CAR_SEQ_3:
                    car.sim.turn = CarTurn.LEFT.value
                if seq == sg.CAR_SEQ_1:
                    car.sim.turn = CarTurn.RIGHT.value
        elif (seq == sg.CAR_SEQ_2) or (seq == sg.CAR_SEQ_4):
            car = Car(initial_x, initial_y, sg.CAR_WIDTH_PIXELS, sg.CAR_LENGTH_PIXELS, seq, sim, 0, window)

            # Update the turns in case of testing
            if sg.TESTING:
                if seq == sg.CAR_SEQ_4:
                    car.sim.turn = CarTurn.LEFT.value
                if seq == sg.CAR_SEQ_2:
                    car.sim.turn = CarTurn.RIGHT.value
        return car

    ####################################################################

    def generate_objects(self, window):
        """ Generate the all the frame's objects to draw it on the frame.
            Args:
                window(pygame window) : Current frame
            Returns:
                 list                 : car_list contains the car objects
                 list                 : path_list contains the path objects
                 list                 : stop_line_list contains the stop line objects
                 object               : Sprite_mid contains the middle intersection object
        """
        min_num_car = 3  # This is equivalent of generating 3
        max_num_car = 5  # This is equivalent of generating 5

        config_manager = Config_manager()
        # Generate the num_cars by random
        num_cars = np.random.randint(min_num_car, max_num_car, 1)[0]

        num_cars = 4

        if sg.TESTING:
            num_cars = sg.NUM_CARS_FOR_TESTING

        # Get the time stopped value at stop sign
        time_stopped = np.random.normal(sg.MEAN_STOPPED_TIME, sg.STD_STOPPED_TIME, 1)[0]
        while time_stopped <= 0:
            time_stopped = np.random.normal(sg.MEAN_STOPPED_TIME, sg.STD_STOPPED_TIME, 1)[0]

        print(" time stopped = ", time_stopped)
        # Create the car_list
        x1 = sg.WINDOW_WIDTH_PIXELS / 2
        y1 = sg.WINDOW_LENGTH_PIXELS / 2
        Car_list = []

        # Generate path list
        Path_list = self.generate_paths()

        # Create Stop Lines instances
        Stop_line_list = self.generate_stop_lines(Path_list)

        for seq in range(1, num_cars + 1):
            sim = Sim(config_manager, seq, time_stopped)

            # Car1 moving along x-direction
            car = self.generate_car_instances(seq, Path_list, sim, window)
            Car_list.append(car)

        # Draw middle traffic rectangle
        Sprite_mid = Stop_Area(
            x1 - sg.SPRITE_MID_LEN_OFFSET,
            y1 - sg.SPRITE_MID_LEN_OFFSET,
            2 * sg.SPRITE_MID_LEN_OFFSET,
            2 * sg.SPRITE_MID_LEN_OFFSET,
            window,
        )

        return Car_list, Path_list, Stop_line_list, Sprite_mid


############################################################################################################
