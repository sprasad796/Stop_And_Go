####################################################
# Uber, Inc. (c) 2019
# Description : Description of all the actors interacting with the Car
####################################################
import enum

####################################################################
# creating enumerations using class


class HeadingDirection(enum.Enum):
    """ Car's heading direction """

    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3


####################################################################


class CarState(enum.Enum):
    """ Car's state during simulation """

    CRUISE = 0
    DECEL = 1
    STOP = 2
    ACCEL = 3
    CRUISE_A = 4
    PAST_SIM = 5


####################################################################


class CarTurn(enum.Enum):
    """ Car's turn during the intersection """

    NO = 0
    LEFT = 1
    RIGHT = 2


####################################################################


class CarLane(enum.Enum):
    """ Priority car's lane position wrt the non priority car """

    OPPOSITE = 0
    CLOCKWISE = 1
    ANTICLOCKWISE = 2


####################################################################


class CarAction(enum.Enum):
    """ Non priority car's action through the intersection """

    NO_ACTION = 0  # No action to be taken on the non priority cars
    WAIT_CROSS_INTERSECTION = 1  # Let the non priroity cars move only when priority car
    # has crossed the intersection
    WAIT_CROSS_CENTER_INTERSECTION = 2  # Let the non priroity cars move only when priority car
    # has crossed the middle of the intersection


####################################################################


class OptionChoice(enum.IntEnum):
    """ Select the choice from the options """

    JSON_OPTION = 1  # Save the dataset into json file
    TETRYS_OPTION = 2  # Save the dataset into tetrys table without MSDS


####################################################################
