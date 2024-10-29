####################################################
# Uber, Inc. (c) 2019
# Description : Regenerated data after the simulation
####################################################
import stop_and_go_globals as sg
from stop_and_go_data_type import CarState, HeadingDirection

####################################################


class CollisionCheck(object):
    def get_lead_trail_car(self, dist1, dist2, seq1, seq2, head_dir):
        """ Get the leading and trailing sequnce number of the car based on the distance
            travelled by the car in the x or y direction
            Args:
                dist1(float)       : Current pose of the car1 in x or y direction
                dist2(float)       : Current pose of the car2 in x or y direction
                seq1(int)          : Sequence number of car1 in the car_list
                seq2(int)          : Sequence number of car2 in the car_list
                head_dir(int)      : Both car are heading same direction.
                                     It is heading direction on the given path
            Returns:
                int                : Sequence number of leading vehcile
                int                : Sequence number of trailing vehcile
                float              : Distance between vehicles
        """
        if (head_dir == HeadingDirection.SOUTH.value) or (head_dir == HeadingDirection.EAST.value):

            if dist1 < dist2:
                lead_vehicle = seq2
                trail_vehicle = seq1
            else:
                lead_vehicle = seq1
                trail_vehicle = seq2
        else:
            if dist1 < dist2:
                lead_vehicle = seq1
                trail_vehicle = seq2
            else:
                lead_vehicle = seq2
                trail_vehicle = seq1

        dist = abs(dist1 - dist2)
        return lead_vehicle, trail_vehicle, dist

    ####################################################

    def get_dist_leading_trailing_car(self, car_list, seq1, seq2):
        """ Check the leading car's distance and update the
            car's speed state flag as constant to the leading vehicle
            Args:
                car_list(list)     : List of car instances
                seq1(int)          : Sequence number of car1 in the car_list
                seq2(int)          : Sequence number of car2 in the car_list
        """

        car1 = car_list[seq1]
        car2 = car_list[seq2]
        dist = -1
        head_dir1 = car1.car_turn_path[car1.sim_state.seq][car1.sim.turn][1]
        head_dir2 = car2.car_turn_path[car2.sim_state.seq][car2.sim.turn][1]
        lead_vehicle = seq1
        trail_vehicle = seq2

        # If pair of car is pointing in the same direction. Their x or
        # y coordinate is same ( they are on the same path ). If they are also
        # inside the frame. Get the seq no of leading, trailing cars and also
        # the distance between them.
        if head_dir1 == head_dir2:
            if (car1.pose.x == car2.pose.x) and (car1.pose.y > 0) and (car2.pose.y > 0):
                lead_vehicle, trail_vehicle, dist = self.get_lead_trail_car(
                    car1.pose.y, car2.pose.y, seq1, seq2, head_dir1
                )
            if (car1.pose.y == car2.pose.y) and (car1.pose.x > 0) and (car2.pose.x > 0):
                lead_vehicle, trail_vehicle, dist = self.get_lead_trail_car(
                    car1.pose.x, car2.pose.x, seq1, seq2, head_dir2
                )
            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(
                    " Distance is ", dist, " lead vehcile seq # ", lead_vehicle, "trail vehcile seq # ", trail_vehicle
                )

        return (dist, lead_vehicle, trail_vehicle)

    ####################################################################

    def check_leading_vehicle_distance(self, car_list):
        """ Check the leading car's distance and update the
            car's speed state flag as constant to the leading vehicle
            Args:
                car_list        : Contains car instances
        """
        num_cars = len(car_list)
        dist = sg.LEAD_VEHICLE_DISTANCE_PIXELS

        for seq1 in range(num_cars - 1):
            # Check the distances in case the current state is cruise_a
            if (car_list[seq1].sim_state.cur_state == CarState.CRUISE_A.value) or (
                car_list[seq1].sim_state.cur_state == CarState.PAST_SIM.value
            ):
                for seq2 in range(seq1 + 1, num_cars):
                    if (car_list[seq2].sim_state.cur_state == CarState.CRUISE_A.value) or (
                        car_list[seq2].sim_state.cur_state == CarState.PAST_SIM.value
                    ):
                        if sg.DEBUG == sg.DEBUG_LEVEL_1:
                            print(
                                " car1 x, y ",
                                car_list[seq1].sim_state.seq,
                                " poses ",
                                car_list[seq1].pose.x,
                                car_list[seq1].pose.y,
                            )
                            print(
                                " car2 x, y ",
                                car_list[seq2].sim_state.seq,
                                " poses",
                                car_list[seq2].pose.x,
                                car_list[seq2].pose.y,
                            )
                        (dist, lead_vehicle_seq, trail_vehicle_seq) = self.get_dist_leading_trailing_car(
                            car_list, seq1, seq2
                        )

                        if (
                            (dist > 0)
                            and (dist <= sg.LEAD_VEHICLE_DISTANCE_PIXELS)
                            and car_list[trail_vehicle_seq].dynamic_state.speed
                            > car_list[lead_vehicle_seq].dynamic_state.speed
                        ):

                            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                                print(
                                    "Setting speed true for seq# to maintain\
                                       the distance",
                                    car_list[trail_vehicle_seq].sim_state.seq,
                                )
                            car_list[trail_vehicle_seq].dynamic_state.set_speed = True
                            car_list[trail_vehicle_seq].dynamic_state.speed = car_list[
                                lead_vehicle_seq
                            ].dynamic_state.speed


############################################################################################################
