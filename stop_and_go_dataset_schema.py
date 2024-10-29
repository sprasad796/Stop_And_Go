# UATC LLC (c) 2020
###################################################################################
import numpy as np
from petastorm.codecs import NdarrayCodec, ScalarCodec
from petastorm.unischema import Unischema, UnischemaField
from pyspark.sql.types import FloatType, IntegerType, StringType

###################################################################################


def create_schema():
    objects = ["vehicle_n", "vehicle_w", "vehicle_s", "stop_sign"]
    objects_schema = []

    field_state_schema = [
        UnischemaField("seq_no", np.int64, (), ScalarCodec(IntegerType()), False),
        UnischemaField("frame_no", np.uint8, (), ScalarCodec(IntegerType()), False),
        UnischemaField("pix_per_m", np.float64, (), ScalarCodec(FloatType()), True),
        UnischemaField("ref_frame_no", np.uint8, (), ScalarCodec(IntegerType()), True),
        UnischemaField("sim_name", np.string_, (), ScalarCodec(StringType()), True),
    ]

    av_state_schema = [
        UnischemaField("av_position", np.float64, (2,), NdarrayCodec(), True),
        UnischemaField("av_dimension", np.float64, (2,), NdarrayCodec(), True),
        UnischemaField("av_heading", np.float64, (), ScalarCodec(FloatType()), True),
        UnischemaField("av_velocity", np.float64, (), ScalarCodec(FloatType()), True),
        UnischemaField("av_acceleration", np.float64, (), ScalarCodec(FloatType()), True),
    ]

    for object in objects:
        cur_objects_schema = [
            UnischemaField(object + "_type", np.string_, (), ScalarCodec(StringType()), True),
            UnischemaField(object + "_position", np.float64, (None, 2), NdarrayCodec(), True),
            UnischemaField(object + "_dimension", np.float64, (None, 2), NdarrayCodec(), True),
            UnischemaField(object + "_heading", np.float64, (), ScalarCodec(FloatType()), True),
            UnischemaField(object + "_velocity", np.float64, (), ScalarCodec(FloatType()), True),
            UnischemaField(object + "_acceleration", np.float64, (), ScalarCodec(FloatType()), True),
        ]
        objects_schema.append(cur_objects_schema)

    # Complete Unischema
    ip_schema = Unischema(
        "ip_schema",
        field_state_schema
        + av_state_schema
        + objects_schema[0]
        + objects_schema[1]
        + objects_schema[2]
        + objects_schema[3],
    )

    # print (ip_schema)

    return ip_schema


###################################################################################


def create_image_schema():
    images = ["ref", "traffic"]
    images_schema = []

    field_state_schema = [
        UnischemaField("seq_no", np.int64, (), ScalarCodec(IntegerType()), False),
        UnischemaField("frame_no", np.uint8, (), ScalarCodec(IntegerType()), False),
    ]

    for image in images:
        cur_objects_schema = [UnischemaField(image + "_image", np.uint8, (256, 256, 3), NdarrayCodec(), False)]
        images_schema.append(cur_objects_schema)

    # Complete Image Unischema
    ip_image_schema = Unischema("ip_image_schema", field_state_schema + images_schema[0] + images_schema[1])

    # print (ip_image_schema)

    return ip_image_schema


###################################################################################


def create_lane_schema():

    field_state_schema = [
        UnischemaField("seq_no", np.int64, (), ScalarCodec(IntegerType()), False),
        UnischemaField("frame_no", np.uint8, (), ScalarCodec(IntegerType()), False),
    ]

    cur_objects_schema = [UnischemaField("lane_image", np.uint8, (256, 256, 3), NdarrayCodec(), False)]

    # Complete Image Unischema
    ip_lane_schema = Unischema("ip_lane_schema", field_state_schema + cur_objects_schema)

    # print (ip_lane_schema)

    return ip_lane_schema


# Testing Purpose Only ############################################################


if __name__ == "__main__":
    create_schema()
    create_image_schema()
    create_lane_schema()

###################################################################################
