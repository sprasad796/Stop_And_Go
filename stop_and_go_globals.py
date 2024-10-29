#####################################################################
# Uber, Inc. (c) 2019
# Description : All the glocal variables shared among the other stop_and_go scripts
# Author      : Swati Prasad
#####################################################################

# Debugging and Displaying with the log statements
DEBUG_LEVEL_1 = 1  # Debugging Level 1
DEBUG_LEVEL_2 = 2  # Debugging Level 2
DEBUG = 3  # Set the DEBUG  with debugging levels
# TESTING for reference lines
TESTING = False

# Car's parameters
# Image generation directory Relative to the script directory
IMAGE_BASE_DIR = "Images"
REFERENCE_IMAGE_KEYWORD = "_ref_"
TRAFFIC_IMAGE_KEYWORD = "_traffic_"

# Main window frame dimensions
WINDOW_WIDTH_PIXELS = 512
WINDOW_LENGTH_PIXELS = 512

# Car dimensions
CAR_WIDTH_PIXELS = 6
CAR_LENGTH_PIXELS = 8

TOTAL_DATA_POINTS = 10  # Number of data points to be generated
START_EXPERIMENT_NUMBER = 0  # To start the experiment number

# Car's sequence number
CAR_SEQ_1 = 1
CAR_SEQ_2 = 2
CAR_SEQ_3 = 3
CAR_SEQ_4 = 4

NUM_CARS_FOR_TESTING = 4  # Number of test cars

# Path's sequence number
PATH_SEQ_0 = 0
PATH_SEQ_1 = 1
PATH_SEQ_2 = 2
PATH_SEQ_3 = 3

# Camera view frame dimensions
SUB_IMAGE_WIDTH = 256
SUB_IMAGE_LENGTH = 256

# To Display the intersection
SPRITE_AREA = False

# CAMERA CONFIG POSITION. IF X, Y position is configured it will overwrite the car seq no
FIXED_FRAME_CAMERA_VIEW = True
SET_CAMERA_ONCE = False  # Configure the camera view once
REFERENCE_CAR_SEQ = 1  # Reference car sequence number
# Fixed camera position
# CAMERA_POS_X = 64
# CAMERA_POS_Y = 64

LANE_BUFFER_PIXELS = 12  # LANE_BUFFER to have some distance between 2 cars
PATH_LINE_WIDTH = 3  # Width of the path
NUMBER_OF_PATHS = 4  # Number of paths

# calculation = mid_window(64) - car_width(5) - (50 * 1= 50)
STOP_LINE_LEN_OFFSET = 19  # Stop line offset from mid of frame
STOP_LINE_HOR_PIXELS = 12  # Horizontal length of the stop line
STOP_LINE_WIDTH = 3  # Stop line width
STOP_SAFE_OFFSET = 2  # To add offset to the stop line
STOP_LINE_GENERATE_MIN = 50  # Stop line should start after this
STOP_LINE_GENERATE_MAX = 200  # Stop line should be generated before this

# Time parameters with mean and std
MEAN_STOPPED_TIME = 0.5  # Mean of the stopped time
STD_STOPPED_TIME = 0.2  # Standard deviation of the stopped time

# Sprite Mid Length offset
SPRITE_MID_LEN_OFFSET = 20  # Offset to the sprite mid length

# Data generation variables
DATASET_SPAN_FRAMES = 250  # Number of frames to be generated
DATASET_REF_FRAMES = 39  # Reference Frame must be less than the start_frame
DATASET_TOTAL_FRAMES = 960  # Maximum total number of frames
# No moving window, value = 0
DATASET_MOVING_WINDOW = 0  # Moving the window after each simulation
DATASET_START_FRAMES = 700  # Start of the frame number
DATASET_START_FRAME_DEV = 250  # Start of the frame number deviation

CAR_BOUNDARY_OFFSET = 2  # Offset for car outside the boundary
TURN_ERROR_OFFSET = 0.1  # Offset for the turn error
TIME_INCREMENT_STEP = 0.1  # Incremenet time after 0.1 sec

ENABLE_VEHICLE_COLLISION_CHECK = True  # Check the collision between cars
LEAD_VEHICLE_DISTANCE_PIXELS = 25  # Following vehicle should maintain 25 pixels from leading vehicle
CAR_SAFETY_BUFFER = 1  # safety buffer at the interscetion
# minimum car length pixels from leading vehicle

# Mask image parameters
SUB_IMAGE_UPPER_MASK = 35  # Upper mask value to generate masked image
SUB_IMAGE_LOWER_MASK = 20  # Lower mask value to generate masked image
WHITE_PIXEL = 255  # White pixel value

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

CONFIG_FILE = "stop_and_go_config.yml"  # Read the car parameters from configuratio file
DATSET_CONFIG_FILE = "stop_and_go_dataset_config.yml"
# To create the subimage for testing
DISPLAY_TRAFFIC = False  # To display the traffic

OUPUT_JSON_FILENAME = "Metadata.json.dat"  # Write the output to the json file
# Camera view subimages
GENERATE_SUBIMAGE = True  # To generate the subimage

#####################################################################
