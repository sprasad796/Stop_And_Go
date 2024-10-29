#####################################################################
# Uber, Inc. (c) 2019
# Description: This is the main game loop
#####################################################################
import os

import pygame
import stop_and_go_globals as sg
######### optional to find the cv2 module in venv ###################
import sys
sys.path.insert(0, "/Users/keshakumar/Stop_And_Go//path/to/venv/lib/python3.9/site-packages")

# from pygame.locals import *
from stop_and_go_data import Save_Sim_Flow_Data
from stop_and_go_data_type import OptionChoice
from stop_and_go_draw import Generator
from stop_and_go_main_loop import start_game
from stop_and_go_view import Camera, Frame_State

####################################################################
pygame.init()
pygame.display.set_caption("Stop and Go")
os.environ["SDL_VIDEODRIVER"] = "dummy"
#####################################################################


def check_params():
    """ This is to check the validity of the global parameters
    """
    # Check the validity of frames
    if sg.DATASET_SPAN_FRAMES < sg.DATASET_REF_FRAMES:
        print(" Invalid number of span frames ", sg.DATASET_SPAN_FRAMES)
        print(" Span frames can't be less than the reference frame ")
        exit(0)


#####################################################################


def main():
    """ This is the main which initializes
        frame_state      : To maintain the state of each frames to track
                           the start, span, end number of each frames
        config_manager   : To get the information from the stop_and_go_config.yml
                           for the car's kinematics info e.g velocity, accl
        generate         : To generate the frame objects stop_line, paths , cars & intersection
        start_game       : Start the pygame with all the objects and start the
                           car movement through the intersection
    """
    # Please enter choice for storing the datasets
    choice_str1 = " Choices are :\n \
                          (1) Store into json file \n \
                          (2) Store into tetrys table \n"
    choice_str2 = " Please enter the choice to save the datasets : "
    choice_str = choice_str1 + choice_str2

    dataset_storage_choice = int(input(choice_str))
    # dataset_storage_choice = 1

    if dataset_storage_choice > OptionChoice.TETRYS_OPTION:
        print(" This is an invalid choice ", dataset_storage_choice, " Quitting")
        pygame.quit()
        exit(0)

    # Initialize the window
    window = pygame.display.set_mode((sg.WINDOW_WIDTH_PIXELS, sg.WINDOW_LENGTH_PIXELS))

    # Initialize the Frame State
    frame_state = Frame_State(
        sg.DATASET_SPAN_FRAMES,
        sg.DATASET_MOVING_WINDOW,
        sg.DATASET_REF_FRAMES,
        sg.DATASET_TOTAL_FRAMES,
        sg.DATASET_START_FRAMES,
        sg.DATASET_START_FRAME_DEV,
    )

    # Get the generate object
    generator = Generator()
    # Generate the instances of the frame objects
    Car_list, Path_list, Stop_line_list, Sprite_mid = generator.generate_objects(window)

    # Initialize the camera view
    camera = Camera(Car_list)

    # Initialize the save_sim_flow_data
    save_sim_flow_data = Save_Sim_Flow_Data(len(Car_list))

    # Initialize the content list
    tetrys_content_list = []
    # Get the StartGame object
    start_game(
        Car_list,
        Path_list,
        Stop_line_list,
        Sprite_mid,
        window,
        camera,
        frame_state,
        dataset_storage_choice,
        save_sim_flow_data,
        tetrys_content_list,
    )

    print(" ****************** Quiting **************** ")


#####################################################################
if __name__ == "__main__":
    check_params()
    main()
#####################################################################
