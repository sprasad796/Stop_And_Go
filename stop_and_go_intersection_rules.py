####################################################
# Uber, Inc. (c) 2020
# Description : Description of all the actors interacting with the Car
####################################################
import stop_and_go_globals as sg
from stop_and_go_data_type import CarAction, CarLane, CarState, CarTurn

##############################################################################


class Intersection_Rule(object):
    """ Check if the car is at the intersection. Intersection rules
        find the priority cars can move together through the intersection
        without the collision.
    """

    def __init__(self):
        """ Initialize & populate the car_intersection_map to store the car seq as key and
            list of opp_lane, clock_adj_lane, anticlock_adj_lane wrt the stored
            seq key.
            Initialize and populate the priroity_car_seq_queue with the car sequences
            based on First come first go.
            Initialize not_right_of_way_car_seq and boolen if it can move through the
            intersection or not.
        """
        self._not_right_of_way_action_map = _populate_not_right_of_way_action_map()
        self._priority_car_seq_queue = []
        self._not_right_of_way_car_seq = None
        self._not_right_of_way_can_move = False

    #################################################################################

    def reset(self):
        """ Reset the priority queue after each iteration """
        self._priority_car_seq_queue = []

    ######################################################################################

    def update_cars_through_intersection(self, car_list, Sprite_mid):
        """ Check if the car has reached the intersection. if it is at the intersection
            make it stop for at least time_stopped_s. Keep incrementing its time index.
            Check the intersection area is cleaned. Assign the priority to the longest
            waiting car. Always long waiting car has priority to move first.

            Args:
                car_list(list)    : Contains list of cars instances
                Sprite_mid(object): Contains mid intersection object
        """

        # collision detection with each car
        for car in car_list:
            # Car has completed the iteration and waiting for the other car to complete
            if car.sim_state.end:
                car.time_index += car.sim.sim_time_increment_s
                continue
            car.sim_state.overlap = _check_car_intersection_overlap(car, Sprite_mid)
            round_time_index = round(car.time_index, 1)

            if round_time_index in car.sim.sim_motion_state_dict:
                car.sim_state.cur_state = car.sim.sim_motion_state_dict[round_time_index][1]
                if not car.dynamic_state.set_speed:
                    car.dynamic_state.speed = car.sim.sim_motion_state_dict[round_time_index][4]
                    car.dynamic_state.accl = car.sim.sim_motion_state_dict[round_time_index][5]

            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(
                    " Current speed and accleration for the car seq #  and time_index ",
                    car.dynamic_state.speed,
                    car.dynamic_state.accl,
                    car.sim_state.seq,
                    round_time_index,
                )

            if (
                (car.sim_state.overlap)
                and (round_time_index in car.sim.sim_motion_state_dict)
                and (car.sim.sim_motion_state_dict[round_time_index][1] == CarState.DECEL.value)
            ):
                if sg.DEBUG == sg.DEBUG_LEVEL_1:
                    print(" It is into collision and decel for seq at time ", car.sim_state.seq, car.time_index)
                car.time_index += car.sim.sim_time_increment_s

            elif (car.sim_state.overlap) and (car.stop_timer < car.sim.time_stopped_s):
                if sg.DEBUG == sg.DEBUG_LEVEL_1:
                    print(
                        " It is into stop for seq & stop_timer , time_stopped = ",
                        car.sim_state.seq,
                        car.stop_timer,
                        car.sim.time_stopped_s,
                    )
                car.stop_timer += car.sim.sim_time_increment_s
                car.time_index += car.sim.sim_time_increment_s
                car.sim_state.stop_car = True
                car.sim_state.lock = False
            else:
                self._update_car_through_intersection(car, car_list)

    #################################################################################

    def _get_neighbor_car_seq(self, car_seq=1, relationship=CarLane.OPPOSITE):
        """ Get the neighbor car's sequence number based on it's relationship with the
            priority car """
        neighbor_car_seq = None
        total_cars = 4
        opp_car_offset = 2
        clk_car_offset = 1
        anticlk_car_offset = 3

        if relationship == CarLane.OPPOSITE:
            neighbor_car_seq = car_seq + opp_car_offset
        elif relationship == CarLane.CLOCKWISE:
            neighbor_car_seq = car_seq + clk_car_offset
        else:
            neighbor_car_seq = car_seq + anticlk_car_offset

        # If car seq > 4
        if neighbor_car_seq > total_cars:
            neighbor_car_seq = neighbor_car_seq % total_cars

        return neighbor_car_seq

    #####################################################################################

    def _get_all_neighbors_car_seq_list(self, car_seq=1):
        """ Mapping the relation between the cars at the intersection. Car seq_no as key
            and List[opp_lane_car_seq_no, clock_adj_lane_car_seq_no, anticlock_adj_lane_car_seq_no]
            for each car.
        """
        neighbor_car_list = []

        for relationship in (CarLane.OPPOSITE, CarLane.CLOCKWISE, CarLane.ANTICLOCKWISE):
            neighbor_car_list.append(self._get_neighbor_car_seq(car_seq, relationship))

        return neighbor_car_list

    #####################################################################################

    def _reset_not_right_of_way(self):
        """ Reset the not right of way car sequence number and it's boolean status to
            move through the intersection or not.
        """
        self._not_right_of_way_car_seq = None
        self._not_right_of_way_can_move = False

    #################################################################################

    def _update_car_through_intersection(self, car, car_list):
        """ Check with each car instance to find out which can move through the
            intersection or wait until priority cars clear the intersection.
            Update the priority queue based on the intersection rules.

            Args :
                 car(object)    : Car object
                 car_list(list) : List of car's instances

        """
        # First get the priority and not right of way cars
        self._set_priority_and_non_priority_cars(car, car_list)

        # Check if it is near stop region area
        if car.sim_state.overlap:
            # if car is in priority queue
            if ((self._priority_car_seq_queue) and (car.sim_state.seq == self._priority_car_seq_queue[0])) or (
                (car.sim_state.seq == self._not_right_of_way_car_seq) and (self._not_right_of_way_can_move)
            ):
                car.sim_state.stop_car = False
                car.time_index += car.sim.sim_time_increment_s
            else:
                car.sim_state.stop_car = True
                # Keep increasing stop_timer to find the car stayed for longest
                car.stop_time_index += car.sim.sim_time_increment_s
                car.dynamic_state.speed = 0.0
                car.dynamic_state.accl = 0.0
                car.sim_state.cur_state = CarState.STOP.value
                car.stop_timer += car.sim.sim_time_increment_s
        else:
            car.time_index += car.sim.sim_time_increment_s
            car.sim_state.stop_car = False
            car.stop_timer = 0

            # Remove priority element
            if (self._priority_car_seq_queue) and (self._priority_car_seq_queue[0] == car.sim_state.seq):
                self._priority_car_seq_queue.pop(0)
                if sg.DEBUG == sg.DEBUG_LEVEL_1:
                    print("Removing priority element ", car.sim_state.seq)
                self._reset_not_right_of_way()

    #################################################################################

    def _set_priority_and_non_priority_cars(self, car, car_list):
        """ set the priority car based on the longest waiting stopped_timer at
            the intersection.

            Args :
                 car(object)    : Car object
                 car_list(list) : List of car's instances
        """
        if sg.DEBUG == sg.DEBUG_LEVEL_1:
            print(
                "current car stop_timer=",
                car.stop_timer,
                "for car seq# ",
                car.sim_state.seq,
                "stopped_timers_s",
                car.sim.time_stopped_s,
            )
        if car.stop_timer >= car.sim.time_stopped_s:
            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(" Longest timer of car seq # ", car.sim_state.seq, " = ", car.stop_timer)
            if car.sim_state.seq not in self._priority_car_seq_queue:
                self._priority_car_seq_queue.append(car.sim_state.seq)
                if sg.DEBUG == sg.DEBUG_LEVEL_1:
                    print(" priority car seq list # ", self._priority_car_seq_queue)

            self._set_not_right_of_way_keys_with_action(car_list)

    #################################################################################

    def _set_not_right_of_way_car_seq(self):
        """ Get the not right of way car seq from the priority car seq queue.
            It's always be the second element in the queue.

            Returns:
                 int    : Returns the sequence number of the not right of way car
        """
        self._not_right_of_way_car_seq = None

        # Get the second item from the priority_car_queue
        if len(self._priority_car_seq_queue) > 1:
            self._not_right_of_way_car_seq = self._priority_car_seq_queue[1]

    #################################################################################

    def _get_priority_car_lane_key(self, neighbor_car_list, priority_car_seq):
        """ Get the lane position of the priority car wrt to the not right of way
            car.
            Args:
               neighbor_car_list(list)  : List of neighbor car's instances
               priority_car_seq(int)    : Sequence number of the priority car

            Returns:
               int : priority car's lane position wrt to the not right of way car
        """
        lane_key = None

        if neighbor_car_list[CarLane.OPPOSITE.value] == priority_car_seq:
            lane_key = CarLane.OPPOSITE.value
        if neighbor_car_list[CarLane.CLOCKWISE.value] == priority_car_seq:
            lane_key = CarLane.CLOCKWISE.value
        if neighbor_car_list[CarLane.ANTICLOCKWISE.value] == priority_car_seq:
            lane_key = CarLane.ANTICLOCKWISE.value

        return lane_key

    #################################################################################

    def _set_not_right_of_way_keys_with_action(self, car_list):
        """ set the not right of way car sequence number with action to move through the
            interscetion or wait.
            Args:
               car_list(list)  : Car_list have the car instances

            Returns:
               tuple  : (1) Not right of way car sequence number.
                        (2) bool status that this car is allowed to
                            move through the interscetion or not.
        """

        # If there is no priroity car, there can't be not right of way cars
        if not self._priority_car_seq_queue:
            return

        priority_car_seq = self._priority_car_seq_queue[0]
        self._set_not_right_of_way_car_seq()
        self._not_right_of_way_can_move = False

        # Check if there is not_right_of_way_car_seq
        if self._not_right_of_way_car_seq is not None:

            # Get the key for the not_right_of_way car's direction
            not_right_of_way_car_turn = car_list[self._not_right_of_way_car_seq - 1].sim.turn

            # Get the action of not right of way car based on the priority car's position
            not_right_of_way_neighbor_car_list = self._get_all_neighbors_car_seq_list(self._not_right_of_way_car_seq)

            priority_lane_key = self._get_priority_car_lane_key(not_right_of_way_neighbor_car_list, priority_car_seq)

            priority_turn_key = car_list[priority_car_seq - 1].sim.turn

            not_right_of_way_action = self._not_right_of_way_action_map[not_right_of_way_car_turn][priority_lane_key][
                priority_turn_key
            ]

            if sg.DEBUG == sg.DEBUG_LEVEL_1:
                print(
                    "Priority car seq # ",
                    priority_car_seq,
                    " has lane_key = ",
                    priority_lane_key,
                    " turn_key = ",
                    priority_turn_key,
                    " wrt the not right of way car seq # ",
                    self._not_right_of_way_car_seq,
                    " with an action_key = ",
                    not_right_of_way_action,
                )

            if not_right_of_way_action == CarAction.NO_ACTION.value:
                self._not_right_of_way_can_move = True
            if not_right_of_way_action == CarAction.WAIT_CROSS_INTERSECTION.value:
                self._not_right_of_way_can_move = False
            if not_right_of_way_action == CarAction.WAIT_CROSS_CENTER_INTERSECTION.value:
                self._not_right_of_way_can_move = self.is_car_crossed_intersection_center(car_list, priority_car_seq)

    #################################################################################

    """
    def get_priority_car_possible_cross_middle_intersection(self, car_seq, car_turn):

        cross_status_dict = {
                sg.CAR_SEQ_1: {
                    CarTurn.NO.value: True,
                    CarTurn.LEFT.value: True,
                    carTurn.RIGHT.value: False},
                sg.CAR_SEQ_2: {
                    CarTurn.NO.value: True,
                    CarTurn.LEFT.value: True,
                    carTurn.RIGHT.value: False},
                sg.CAR_SEQ_3: {
                    CarTurn.NO.value: True,
                    CarTurn.LEFT.value: True,
                    carTurn.RIGHT.value: False},
                sg.CAR_SEQ_4: {
                    CarTurn.NO.value: True,
                    CarTurn.LEFT.value: True,
                    carTurn.RIGHT.value: False}}

        return cross_status_dict[car_seq][car_turn]
    """

    #################################################################################

    def is_car_crossed_intersection_center(self, car_list, priority_car_seq):
        """ Check if the priority car has crossed the center of the
            intersection with the configured buffer distance.
            Args:
               car_list(list)        : Car_list has car's instances
               priority_car_seq(int) : Sequence number of the priority car

            Returns:
               bool  : If the priority car has crossed the center of the intersection
                       with configured buffer distance.
        """
        cur_x_pos = car_list[priority_car_seq - 1].pose.x
        cur_y_pos = car_list[priority_car_seq - 1].pose.y

        cross_status = False

        pos_x_mid_intersection = sg.WINDOW_LENGTH_PIXELS / 2 + sg.CAR_SAFETY_BUFFER
        neg_x_mid_intersection = sg.WINDOW_LENGTH_PIXELS / 2 - sg.CAR_SAFETY_BUFFER
        pos_y_mid_intersection = sg.WINDOW_WIDTH_PIXELS / 2 + sg.CAR_SAFETY_BUFFER
        neg_y_mid_intersection = sg.WINDOW_WIDTH_PIXELS / 2 - sg.CAR_SAFETY_BUFFER

        if priority_car_seq == sg.CAR_SEQ_1:
            if (car_list[priority_car_seq - 1].sim.turn == CarTurn.NO.value) and (cur_x_pos > pos_x_mid_intersection):
                cross_status = True
            if (
                (car_list[priority_car_seq - 1].sim.turn == CarTurn.LEFT.value)
                and (cur_x_pos > pos_x_mid_intersection)
                and (cur_y_pos < neg_y_mid_intersection)
            ):
                cross_status = True

        elif priority_car_seq == sg.CAR_SEQ_2:
            if (car_list[priority_car_seq - 1].sim.turn == CarTurn.NO.value) and (cur_y_pos > pos_y_mid_intersection):
                cross_status = True
            if (
                (car_list[priority_car_seq - 1].sim.turn == CarTurn.LEFT.value)
                and (cur_x_pos > pos_x_mid_intersection)
                and (cur_y_pos > pos_y_mid_intersection)
            ):
                cross_status = True

        elif priority_car_seq == sg.CAR_SEQ_3:
            if (car_list[priority_car_seq - 1].sim.turn == CarTurn.NO.value) and (cur_x_pos < neg_x_mid_intersection):
                cross_status = True
            if (
                (car_list[priority_car_seq - 1].sim.turn == CarTurn.LEFT.value)
                and (cur_x_pos < neg_x_mid_intersection)
                and (cur_y_pos > pos_y_mid_intersection)
            ):
                cross_status = True

        elif priority_car_seq == sg.CAR_SEQ_4:
            if (car_list[priority_car_seq - 1].sim.turn == CarTurn.NO.value) and (cur_y_pos < neg_y_mid_intersection):
                cross_status = True
            if (
                (car_list[priority_car_seq - 1].sim.turn == CarTurn.LEFT.value)
                and (cur_x_pos < neg_x_mid_intersection)
                and (cur_y_pos < neg_y_mid_intersection)
            ):
                cross_status = True

        return cross_status

    #################################################################################


def _populate_not_right_of_way_action_map():
    """ Mapping of the not right of way car's action information based on the heading
        direction of the priority car.
    """
    not_right_of_way_action_map = {
        CarTurn.NO.value: {
            CarLane.OPPOSITE.value: {
                CarTurn.LEFT.value: CarAction.WAIT_CROSS_INTERSECTION.value,
                CarTurn.RIGHT.value: CarAction.NO_ACTION.value,
                CarTurn.NO.value: CarAction.NO_ACTION.value,
            },
            CarLane.CLOCKWISE.value: {
                CarTurn.LEFT.value: CarAction.WAIT_CROSS_INTERSECTION.value,
                CarTurn.RIGHT.value: CarAction.NO_ACTION.value,
                CarTurn.NO.value: CarAction.WAIT_CROSS_INTERSECTION.value,
            },
            CarLane.ANTICLOCKWISE.value: {
                CarTurn.LEFT.value: CarAction.WAIT_CROSS_CENTER_INTERSECTION.value,
                CarTurn.RIGHT.value: CarAction.WAIT_CROSS_INTERSECTION.value,
                CarTurn.NO.value: CarAction.WAIT_CROSS_CENTER_INTERSECTION.value,
            },
        },
        CarTurn.RIGHT.value: {
            CarLane.OPPOSITE.value: {
                CarTurn.LEFT.value: CarAction.WAIT_CROSS_INTERSECTION.value,
                CarTurn.RIGHT.value: CarAction.NO_ACTION.value,
                CarTurn.NO.value: CarAction.NO_ACTION.value,
            },
            CarLane.CLOCKWISE.value: {
                CarTurn.LEFT.value: CarAction.NO_ACTION.value,
                CarTurn.RIGHT.value: CarAction.NO_ACTION.value,
                CarTurn.NO.value: CarAction.WAIT_CROSS_INTERSECTION.value,
            },
            CarLane.ANTICLOCKWISE.value: {
                CarTurn.LEFT.value: CarAction.NO_ACTION.value,
                CarTurn.RIGHT.value: CarAction.NO_ACTION.value,
                CarTurn.NO.value: CarAction.NO_ACTION.value,
            },
        },
        CarTurn.LEFT.value: {
            CarLane.OPPOSITE.value: {
                CarTurn.LEFT.value: CarAction.WAIT_CROSS_INTERSECTION.value,
                CarTurn.RIGHT.value: CarAction.WAIT_CROSS_INTERSECTION.value,
                CarTurn.NO.value: CarAction.WAIT_CROSS_CENTER_INTERSECTION.value,
            },
            CarLane.CLOCKWISE.value: {
                CarTurn.LEFT.value: CarAction.WAIT_CROSS_CENTER_INTERSECTION.value,
                CarTurn.RIGHT.value: CarAction.NO_ACTION.value,
                CarTurn.NO.value: CarAction.WAIT_CROSS_INTERSECTION.value,
            },
            CarLane.ANTICLOCKWISE.value: {
                CarTurn.LEFT.value: CarAction.WAIT_CROSS_CENTER_INTERSECTION.value,
                CarTurn.RIGHT.value: CarAction.NO_ACTION.value,
                CarTurn.NO.value: CarAction.WAIT_CROSS_INTERSECTION.value,
            },
        },
    }

    return not_right_of_way_action_map


#################################################################################


def _check_car_intersection_overlap(car, sprite_mid):
    """ Check the overlap between the car and the middle intersection
        Args:
            car(object)          : Car instance
            sprite_mid(object)   : Sprite_mid instance
        Returns:
            bool                 : Returns true if there is an intersection between
                                   car and mid intersection
    """
    car_pos_x = car.pose.x
    car_pos_y = car.pose.y
    car_width = car.physical_properties.width
    car_length = car.physical_properties.length
    intersection_x = sprite_mid.x
    interscetion_y = sprite_mid.y
    intersection_width = sprite_mid.width
    intersection_length = sprite_mid.length

    if (
        (car_pos_x + car_length >= intersection_x)
        and (car_pos_x <= intersection_x + intersection_length)
        and (car_pos_y + car_width >= interscetion_y)
        and (car_pos_y <= interscetion_y + intersection_width)
    ):
        return True
    return False


#################################################################################
