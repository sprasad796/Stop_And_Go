#####################################################################
# Uber, Inc. (c) 2019
# Description: This is the main game loop
#####################################################################
import os
import time

import pygame
import stop_and_go_globals as sg
from stop_and_go_data import Save_Sim_Flow_Data
from stop_and_go_data_type import CarState
from stop_and_go_draw import Drawer, Generator
from stop_and_go_intersection_rules import Intersection_Rule
from stop_and_go_view import Camera

####################################################################


def check_image_dir():
    """ Check if the directory exist else create it. """
    try:
        os.stat(sg.IMAGE_BASE_DIR)
    except BaseException:
        os.mkdir(sg.IMAGE_BASE_DIR)


####################################################################


def validate_last_time_key(last_frame_time, car_list):
    """ Check if all the cars have enough simulated time as last frame's time.
        If all the cars have enough time to reach the end of simulation return true
        else return false.
        Args:
            last_frame_time(float): Last timestamp of the frame
            car_list(list)        : Car_list has 4 car's objects
        Returns:
            bool                  : Returns true if all the cars have valid number of frames
    """
    valid_frame_state = True

    # Check the reference car has some phase into cruise after
    for car in car_list:
        # Car may reach end or go outside the boundary in the last second
        last_sim_time_key = round(round(sorted(car.sim.sim_motion_state_dict.keys())[-1], 1) - 2, 1)
        if (car.sim_state.seq == sg.CAR_SEQ_1) and (
            (car.sim.sim_motion_state_dict[last_sim_time_key][1] != CarState.CRUISE_A.value)
            and (car.sim.sim_motion_state_dict[last_sim_time_key][1] != CarState.PAST_SIM.value)
        ):

            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(
                    " Car # ",
                    car.sim_state.seq,
                    " has motion state ",
                    car.sim.sim_motion_state_dict[last_sim_time_key][1],
                )

            valid_frame_state = False
            return valid_frame_state

        if last_sim_time_key < round(last_frame_time, 1):
            valid_frame_state = False
            return valid_frame_state

    return valid_frame_state


###########################################################################################


def start_game(
    car_list,
    path_list,
    stop_line_list,
    sprite_mid,
    window,
    camera,
    frame_state,
    dataset_storage_choice,
    save_sim_flow_data,
    tetrys_content_list,
):
    """ Start the simulation. Check the car's through the intersection. If all the cars
        have reached the end , reset the car's parametrs and restart the simulation.
        if there is a valid frame for each car, draw sdv, traffic on the frame.
        Args:
            car_list(list)                 : car_list has 4 car's objects
            path_list(list)                : path_list has 4 path's objects
            stop_line_list(list)           : stop_line_list  has 4 stop line's objects
            sprite_mid(object)             : sprite_mid contains the intersection information
            window(pygame window)          : Current Frame
            camera(object)                 : Camera information wrt reference car
            frame_state(object)            : Frame_state contains the frame information
            spark(SparkSession)            : Spark session to insert entries
            dataset_storage_choice(int)    : Store the dataset into json or tetrys table
            save_sim_flow_data(dictionary) : Map to store the simulation data for each car with key as time
            tetrys_content_list(list)      : Store each iteration row data for tetrys
    """
    gameLoop = True
    exp_status = False
    reset_frames_exp = True
    frame_division = 10.0
    exp_no = sg.START_EXPERIMENT_NUMBER
    sub_seq_no = sg.DATASET_START_FRAMES
    intersection_rule = Intersection_Rule()

    # Create/check for the existence of directory to put the images in
    check_image_dir()

    # Main loop
    start_time = time.time()
    while gameLoop:
        if exp_no == sg.TOTAL_DATA_POINTS:
            print("--- %s seconds ---" % (time.time() - start_time))
            pygame.quit()
            exit(0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("unexpected quit --- %s seconds ---" % (time.time() - start_time))
                pygame.quit()
                exit(0)

        # Reset the screen to blank white each time
        window.fill(sg.BLACK)

        # Update the car's positiona nd timer at the intersection
        intersection_rule.update_cars_through_intersection(car_list, sprite_mid)

        # Update the car's movement and check if it is out of frame or not
        for car in car_list:
            car.update_car_position(path_list, sprite_mid)
            car.check_outside_boundary()

        # If each car reaches the end
        end_status = all(car.sim_state.end for car in car_list)

        # All the cars has completed their move. We need to restart the moves or
        # need to restart with referenced image in camera view.
        # In camera view if car1 has reached the end then only reset
        if end_status or exp_status:
            if sg.DEBUG == sg.DEBUG_LEVEL_2:
                print(" end status & exp status = ", end_status, exp_status)
            for car in car_list:
                car.reset()

            # Increment the experiment no
            sg.SET_CAMERA_ONCE = False

            # Reset the camera view frame
            frame_state.reset_frames()
            if reset_frames_exp:
                exp_no += 1

            # Reset the intersection_manager
            intersection_rule.reset()

            exp_status = False
            reset_frames_exp = True
            # Reset the frame number
            sub_seq_no = frame_state.start_frame
            # RESET FOR RESTARTING THE GAME AGAIN
            pygame.display.update()
            car_list, camera, save_sim_flow_data = reset_all(window, exp_no)

        # validate frames for the valid frame bounds, if any car has invalid frames return
        valid_frames = validate_last_time_key(frame_state.end_frame / frame_division, car_list)

        # if there is no valid frames, just repeat the iteration with reset frame
        if valid_frames:
            # First Save the data and then draw it. This is for one simulation without window move
            draw_status = save_sim_flow_data.populate_sim_data_car_seq(car_list, stop_line_list, frame_state)

            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(" no_data_required status = ", draw_status)

            if draw_status:
                if sg.DEBUG == sg.DEBUG_LEVEL_1:
                    print(" **************** IMAGE CREATION STARTS *************")

                # Get the Draw object and coordinate transformation
                draw = Drawer(
                    exp_no,
                    car_list,
                    path_list,
                    stop_line_list,
                    sprite_mid,
                    camera,
                    frame_state,
                    dataset_storage_choice,
                    save_sim_flow_data,
                )

                reset_frames_exp = draw.draw_all_traffic(
                    window, sub_seq_no, dataset_storage_choice, tetrys_content_list
                )

                # Regenarate the frames. In case of complete traffic there is no transformation in 128, so skip it.
                if sg.DISPLAY_TRAFFIC:
                    reset_frames_exp = True

                exp_status = True
        else:
            # No need to reset the frames
            reset_frames_exp = False
            # Set the exp_status = True
            exp_status = True


#####################################################################


def reset_all(window, exp_no):
    """ Reset the simulation and all the objects's parameters
        that is  part of the simulation.
        Args:
            window(pygame window) : current frame
            exp_no(int)           : Current experiment number
    """
    if sg.DEBUG == sg.DEBUG_LEVEL_1:
        print(" RESET EVERYTHING ")

    print("**************START ITERATION No *******************", exp_no)
    # Get the generate object
    generator = Generator()
    Car_list, Path_list, Stop_line_list, Sprite_mid = generator.generate_objects(window)
    print("**************END ITERATION No *******************", exp_no)

    # Initialize the camera view
    camera = Camera(Car_list)

    # Initialize the save_sim_flow_data
    save_sim_flow_data = Save_Sim_Flow_Data(len(Car_list))

    return Car_list, camera, save_sim_flow_data


#####################################################################
