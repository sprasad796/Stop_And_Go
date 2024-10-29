####################################################
# Uber, Inc. (c) 2019
####################################################
# from av.ml.petastorm_utilities.tetrys.write_tetrys import write_tetrys
import cv2
import numpy as np
import stop_and_go_globals as sg
import yaml
from pyspark import SparkConf, SparkContext
from pyspark.sql import SparkSession
from stop_and_go_data import JsonFileManager
from stop_and_go_data_type import OptionChoice
from stop_and_go_dataset_schema import create_image_schema, create_lane_schema, create_schema

from atg.ml.tetrystables.impl.write_tetrys import write_tetrys

####################################################


class DatasetGenerator(object):
    """ Generate the dataset in the two formats a) Json file
                                                b) tetrys """

    ####################################################

    """ Initialize with dataset and output choice to be selected
        for stroing dataset. There are two choices to be selected
        from 1) Store in Json
             2) Store in tetrys table.
        Args:
            save_sim_flow_data(dict)    : Contains the car's trajectory information indexed by time
            daatset_choice(int)         : The input choice to user to select to store the
                                          datasets
    """

    def __init__(self, save_sim_flow_data, dataset_choice):
        self.save_sim_flow_data = save_sim_flow_data
        self.jsonManager = JsonFileManager(sg.OUPUT_JSON_FILENAME)
        self.dataset_storage_choice = dataset_choice

    ####################################################

    def write_file_content(self, exp_no, sub_seq_no, frame_idx, cur_time, car_list, tetrys_content_list):
        """ Create the json object with the traffic, sdv & stop sign information.
            Args:
                exp_no(int)                    : Current iteration number
                sub_seq_no(int)                : Current Image number of the given iteration number
                frame_idx(int)                 : Frame offset number
                cur_time(float)                : Current simulation time
                car_list(list)                 : Car list of car objects
                tetrys_content_list(list)      : Store the each iteration's info
        """
        exp_str = str(exp_no)
        exp_str_fill = exp_str.zfill(5)

        cur_time = round(sub_seq_no * sg.TIME_INCREMENT_STEP, 1)
        content_list = self.save_sim_flow_data.sim_data_dict_list
        json_obj = {}

        json_obj["frame_no"] = frame_idx
        json_obj["num_actors"] = len(car_list) - 1
        json_obj["pix_per_m"] = 1.0
        json_obj["ref_frame_no"] = sg.DATASET_REF_FRAMES
        json_obj["seq_no"] = exp_no
        json_obj["sim_name"] = exp_str_fill

        json_obj["ref_state"] = {}
        json_obj["ref_state"]["loc_x_p"] = round(content_list[0][cur_time][1].center_x_p, 4)
        json_obj["ref_state"]["loc_y_p"] = round(content_list[0][cur_time][1].center_y_p, 4)
        json_obj["ref_state"]["width_p"] = float(sg.CAR_WIDTH_PIXELS)
        json_obj["ref_state"]["length_p"] = float(sg.CAR_LENGTH_PIXELS)
        json_obj["ref_state"]["heading_rad"] = round(float(content_list[0][cur_time][1].rot_heading_rad), 4)
        json_obj["ref_state"]["speed_pps"] = round(float(content_list[0][cur_time][1].speed_pps), 4)
        json_obj["ref_state"]["acc_ppss"] = round(float(content_list[0][cur_time][1].acc_ppss), 4)
        json_obj["ref_state"]["turn"] = content_list[0][cur_time][1].cur_turn

        json_obj["traffic"] = []
        for i in range(1, len(content_list)):
            actor = {}
            actor["loc_x_p"] = round(float(content_list[i][cur_time][1].center_x_p), 4)
            actor["loc_y_p"] = round(float(content_list[i][cur_time][1].center_y_p), 4)
            actor["width_p"] = float(sg.CAR_WIDTH_PIXELS)
            actor["length_p"] = float(sg.CAR_LENGTH_PIXELS)
            actor["heading_rad"] = round(float(content_list[i][cur_time][1].rot_heading_rad), 4)
            actor["speed_pps"] = round(float(content_list[i][cur_time][1].speed_pps), 4)
            actor["acc_ppss"] = round(float(content_list[i][cur_time][1].acc_ppss), 4)
            actor["turn"] = content_list[i][cur_time][1].cur_turn
            json_obj["traffic"].append(actor)

        stop_lines = content_list[0][cur_time][2]
        json_obj["stop_signs"] = [
            {
                "loc_x_p": round(float(stop_lines.stop_line_states[i].center_x_p), 4),
                "loc_y_p": round(float(stop_lines.stop_line_states[i].center_y_p), 4),
            }
            for i in range(sg.NUM_CARS_FOR_TESTING)
        ]

        if self.dataset_storage_choice == OptionChoice.JSON_OPTION:
            self.jsonManager.write_json(json_obj)
        elif self.dataset_storage_choice == OptionChoice.TETRYS_OPTION:
            # Store the object as tetrys object
            self._store_tetrys_object(json_obj)
            # Append the tetrys object in list
            tetrys_content_list.append(json_obj)
            # At the end of the experiment dump into tetrys table
            if len(tetrys_content_list) == (sg.TOTAL_DATA_POINTS - sg.START_EXPERIMENT_NUMBER) * sg.DATASET_SPAN_FRAMES:
                # Initialize spark session and config
                self._set_spark_session_and_config()

                # dump into tetrys tables
                self._dump_into_tetrys_table(tetrys_content_list)
                self._dump_images_into_images_tetrys_table()
                self._dump_lanes_into_lanes_tetrys_table()

    ####################################################

    def _store_tetrys_object(self, tetrys_obj):
        """ Create the tetrys object with the traffic, sdv & stop sign information.
            Args:
                tetrys_obj(object)  : Store the traffic, sdv & stop sign information for
                                      the tetrys table
        """
        # Store av's parameters
        tetrys_obj["av_position"] = np.array([tetrys_obj["ref_state"]["loc_x_p"], tetrys_obj["ref_state"]["loc_y_p"]])
        tetrys_obj["av_velocity"] = tetrys_obj["ref_state"]["speed_pps"]
        tetrys_obj["av_acceleration"] = tetrys_obj["ref_state"]["acc_ppss"]
        tetrys_obj["av_heading"] = tetrys_obj["ref_state"]["heading_rad"]
        tetrys_obj["av_dimension"] = np.array([tetrys_obj["ref_state"]["length_p"], tetrys_obj["ref_state"]["width_p"]])

        # Store objects stop sign parameters
        tetrys_obj["stop_sign_type"] = "STOP_SIGNS"
        stop_signs_list = []
        for i in range(4):
            stop_signs_list.append([tetrys_obj["stop_signs"][i]["loc_x_p"], tetrys_obj["stop_signs"][i]["loc_y_p"]])

        tetrys_obj["stop_sign_position"] = np.array(stop_signs_list)

        # Store the traffic parameters

        traffic_obj = ["vehicle_n", "vehicle_w", "vehicle_s"]
        for i in range(tetrys_obj["num_actors"]):
            tetrys_obj[traffic_obj[i] + "_type"] = "VEHICLE"
            tetrys_obj[traffic_obj[i] + "_heading"] = tetrys_obj["traffic"][i]["heading_rad"]
            tetrys_obj[traffic_obj[i] + "_velocity"] = tetrys_obj["traffic"][i]["speed_pps"]
            tetrys_obj[traffic_obj[i] + "_acceleration"] = tetrys_obj["traffic"][i]["acc_ppss"]

            tetrys_obj[traffic_obj[i] + "_position"] = np.array(
                [[tetrys_obj["traffic"][i]["loc_x_p"], tetrys_obj["traffic"][i]["loc_y_p"]]]
            )
            tetrys_obj[traffic_obj[i] + "_dimension"] = np.array(
                [[tetrys_obj["traffic"][i]["length_p"], tetrys_obj["traffic"][i]["width_p"]]]
            )

        # Delete keys not reuqired in tetrys
        del tetrys_obj["traffic"]
        del tetrys_obj["ref_state"]
        del tetrys_obj["stop_signs"]
        del tetrys_obj["num_actors"]

        # print(" tetrys object is ", tetrys_obj)

    ####################################################

    def _set_spark_session(self, spark_config):
        """Creates a local spark instance needed to map dataset at scale.

        Args:
            spark_config(dict)   : Configuration settings for spark session.

        Returns:
            SparkSession         : Return the spark session for mapping dataset at scale.
        """

        # configure spark session
        conf = SparkConf()
        for key, value in spark_config.items():
            conf.set("spark." + key, value)

        try:
            sc = SparkContext(conf=conf)
        except BaseException:
            sc.stop()
            sc = SparkContext(conf=conf)

        self.spark = SparkSession(sc)

    #####################################################################

    def _load_dataset_config(self, filename):
        """Loads dataset yaml file into a dictionary

        Args:
            filename(str)     :  .yml configuration file
        Returns:
            dict              :  dictionary with all the configuration key-value pairs
        """

        with open(filename, "r") as fn:
            data = yaml.safe_load(fn)
        return data

    #####################################################################

    def _set_spark_session_and_config(self):
        """Set spark instance and load config """

        # Load dataset config
        self.config = self._load_dataset_config(sg.DATSET_CONFIG_FILE)

        # Set a spark session for creating datasets
        self._set_spark_session(spark_config=self.config["spark"])

        # Get the spark Context
        self.sc = self.spark.sparkContext

    #####################################################################

    def _dump_into_tetrys_table(self, tetrys_content_list):
        """ Dump the tetrys object list for all the iterations into
            configured tetrys table location.

            Args:
               tetrys_content_list(list) : List of the all the tetrys objects
        """
        ip_dataset_schema = create_schema()

        # Create rdd from the tetrys_content_list
        tetrys_data_rdd = self.sc.parallelize([tetrys_content_list])

        # Write tetrys objects into table
        tetrys_table_path = self.config["write_dataset_url"]
        spark_key_column = self.config["key_column"]
        spark_order_by_column = self.config["order_by_column"]

        # print(" tetrys path ", tetrys_table_path, spark_key_column, spark_order_by_column)

        write_tetrys(
            self.spark,
            row_generators=tetrys_data_rdd,
            output_url=tetrys_table_path,
            key_column=spark_key_column,
            order_by_column=spark_order_by_column,
            column_group_match_regex={"column_group1": ["av*"]},
            schema=ip_dataset_schema,
            chunk_size=2 ** 20,
        )

    #####################################################################

    def _dump_images_into_images_tetrys_table(self):
        """ Dump the images for all the iterations into
            configured tetrys table location.
        """
        ip_image_schema = create_image_schema()

        # Create images list
        tetrys_image_list = []
        for exp in range(sg.START_EXPERIMENT_NUMBER, sg.TOTAL_DATA_POINTS, 1):
            for frame in range(sg.DATASET_SPAN_FRAMES):
                tetrys_images = {}

                exp_no = str(exp)
                cur_exp_no = exp_no.zfill(5)
                tetrys_images["seq_no"] = exp

                frame_no = str(frame)
                cur_frame_no = frame_no.zfill(6)

                tetrys_images["frame_no"] = frame
                ref_image_name = (
                    sg.IMAGE_BASE_DIR
                    + "/"
                    + "stop_"
                    + str(cur_exp_no)
                    + sg.REFERENCE_IMAGE_KEYWORD
                    + str(cur_frame_no)
                    + ".jpg"
                )
                trf_image_name = (
                    sg.IMAGE_BASE_DIR
                    + "/"
                    + "stop_"
                    + str(cur_exp_no)
                    + sg.TRAFFIC_IMAGE_KEYWORD
                    + str(cur_frame_no)
                    + ".jpg"
                )

                tetrys_images["ref_image"] = cv2.imread(ref_image_name)
                tetrys_images["traffic_image"] = cv2.imread(trf_image_name)

                tetrys_image_list.append(tetrys_images)

        # Create rdd from the tetrys_content_list
        tetrys_images_data_rdd = self.sc.parallelize([tetrys_image_list])

        # Write tetrys objects into table
        tetrys_image_table_path = self.config["write_image_dataset_url"]
        spark_key_column = self.config["key_column"]
        spark_order_by_column = self.config["order_by_column"]

        write_tetrys(
            self.spark,
            row_generators=tetrys_images_data_rdd,
            output_url=tetrys_image_table_path,
            key_column=spark_key_column,
            order_by_column=spark_order_by_column,
            column_group_match_regex={"column_group2": [".*"]},
            schema=ip_image_schema,
            chunk_size=2 ** 20,
        )

    #####################################################################

    def _dump_lanes_into_lanes_tetrys_table(self):
        """ Dump the lane images for all the iterations into
            configured tetrys table location.
        """
        ip_lane_schema = create_lane_schema()

        # Create images list
        tetrys_lane_list = []
        frame = 0

        for exp in range(sg.START_EXPERIMENT_NUMBER, sg.TOTAL_DATA_POINTS, 1):
            tetrys_lanes = {}

            exp_no = str(exp)
            cur_exp_no = exp_no.zfill(5)
            tetrys_lanes["seq_no"] = exp

            frame_no = str(frame)
            cur_frame_no = frame_no.zfill(6)

            tetrys_lanes["frame_no"] = frame
            lane_image_name = (
                sg.IMAGE_BASE_DIR + "/" + "stop_" + str(cur_exp_no) + "_LANES_" + str(cur_frame_no) + ".jpg"
            )

            tetrys_lanes["lane_image"] = cv2.imread(lane_image_name)

            tetrys_lane_list.append(tetrys_lanes)

        # Create rdd from the tetrys_content_list
        tetrys_lanes_data_rdd = self.sc.parallelize([tetrys_lane_list])

        # Write tetrys objects into table
        tetrys_lane_table_path = self.config["write_lane_image_dataset_url"]
        spark_key_column = self.config["key_column"]
        spark_order_by_column = self.config["order_by_column"]

        write_tetrys(
            self.spark,
            row_generators=tetrys_lanes_data_rdd,
            output_url=tetrys_lane_table_path,
            key_column=spark_key_column,
            order_by_column=spark_order_by_column,
            column_group_match_regex={"column_group3": [".*"]},
            schema=ip_lane_schema,
            chunk_size=2 ** 20,
        )

    #####################################################################
