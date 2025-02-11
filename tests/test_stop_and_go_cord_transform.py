import unittest
from unittest.mock import MagicMock, patch
import math
import stop_and_go_globals as sg
from stop_and_go_data_type import CarTurn
from stop_and_go_cord_transform import CoordinateTransform

class TestCoordinateTransform(unittest.TestCase):

    def setUp(self):
        self.exp_no = 1
        self.sub_seq_no = 1
        self.camera = MagicMock()
        self.car_list = [MagicMock()]
        self.path_list = [MagicMock()]
        self.frame_state = MagicMock()
        self.save_sim_flow_data = MagicMock()
        self.coord_transform = CoordinateTransform(
            self.exp_no, self.sub_seq_no, self.camera, self.car_list, self.path_list, self.frame_state, self.save_sim_flow_data
        )

    def test_get_translation_metrics(self):
        self.frame_state.ref_frame = 10
        sg.TIME_INCREMENT_STEP = 0.1
        sg.REFERENCE_CAR_SEQ = 1
        sg.SUB_IMAGE_WIDTH = 256
        sg.SUB_IMAGE_LENGTH = 256
        self.save_sim_flow_data.sim_data_dict_list = [
            {1.0: [None, MagicMock(center_main_x_p=128, center_main_y_p=128)]}
        ]
        result = self.coord_transform.get_translation_metrics()
        self.assertEqual(result, (0, 0))

    def test_save_sub_positions(self):
        self.coord_transform.update_heading_angle = MagicMock()
        self.coord_transform.get_translation_metrics = MagicMock(return_value=(0, 0))
        self.coord_transform.save_sub_cars_pos = MagicMock()
        self.coord_transform.save_sub_paths_pos = MagicMock()
        result = self.coord_transform.save_sub_positions(10)
        self.coord_transform.update_heading_angle.assert_called_once()
        self.coord_transform.get_translation_metrics.assert_called_once()
        self.coord_transform.save_sub_cars_pos.assert_called_once()
        self.coord_transform.save_sub_paths_pos.assert_called_once()
        self.assertEqual(result, (0, 0))

    def test_get_updated_heading_angle(self):
        sg.CAR_SEQ_1 = 1
        sg.CAR_SEQ_2 = 2
        sg.CAR_SEQ_3 = 3
        sg.CAR_SEQ_4 = 4
        result = self.coord_transform.get_updated_heading_angle(1, 1, CarTurn.LEFT.value, math.pi / 4)
        self.assertEqual(result, -math.pi / 4)

    def test_update_heading_angle(self):
        self.save_sim_flow_data.sim_data_dict_list = [
            {1.0: [None, MagicMock(rot_heading_rad=math.pi / 4)]}
        ]
        self.car_list[0].sim_state.seq = 1
        self.car_list[0].sim.turn = CarTurn.LEFT
        self.coord_transform.get_updated_heading_angle = MagicMock(return_value=-math.pi / 4)
        self.coord_transform.update_heading_angle(1.0)
        self.assertEqual(self.save_sim_flow_data.sim_data_dict_list[0][1.0][1].rot_heading_rad, -math.pi / 4)

    def test_save_sub_cars_pos(self):
        self.save_sim_flow_data.sim_data_dict_list = [
            {1.0: [None, MagicMock(loc_main_x_p=128, loc_main_y_p=128, center_main_x_p=128, center_main_y_p=128)]}
        ]
        self.car_list[0].sim_state.seq = 1
        self.coord_transform.save_sub_cars_pos(1.0, (0, 0))
        self.assertEqual(self.save_sim_flow_data.sim_data_dict_list[0][1.0][1].loc_x_p, 128)
        self.assertEqual(self.save_sim_flow_data.sim_data_dict_list[0][1.0][1].loc_y_p, 128)
        self.assertEqual(self.save_sim_flow_data.sim_data_dict_list[0][1.0][1].center_x_p, 128)
        self.assertEqual(self.save_sim_flow_data.sim_data_dict_list[0][1.0][1].center_y_p, 128)

    def test_save_sub_paths_pos(self):
        sg.NUMBER_OF_PATHS = 1
        sg.REFERENCE_CAR_SEQ = 1
        self.save_sim_flow_data.sim_data_dict_list = [
            {1.0: [None, None, MagicMock(stop_line_states=[MagicMock(center_x_p=128, center_y_p=128)])]}
        ]
        self.coord_transform.save_sub_paths_pos(1.0, (0, 0))
        self.assertEqual(self.save_sim_flow_data.sim_data_dict_list[0][1.0][2].stop_line_states[0].center_x_p, 128)
        self.assertEqual(self.save_sim_flow_data.sim_data_dict_list[0][1.0][2].stop_line_states[0].center_y_p, 128)

    @patch('stop_and_go_cord_transform.RotateImage')
    @patch('stop_and_go_cord_transform.SubImage')
    def test_update_from_main_to_sub(self, MockSubImage, MockRotateImage):
        window = MagicMock()
        frame = 10
        image_name = sg.TRAFFIC_IMAGE_KEYWORD
        subimg_instance = MockSubImage.return_value
        rotate_img_instance = MockRotateImage.return_value
        subimg_instance.create_subimage.return_value = (MagicMock(), False)
        subimg_instance.create_sub_mask_image.return_value = MagicMock()
        rotate_img_instance.get_rotated_images.return_value = (MagicMock(), MagicMock())
        self.coord_transform.save_sub_positions = MagicMock(return_value=(0, 0))
        rotate_img_instance.save_rotated_positions = MagicMock()

        result = self.coord_transform.update_from_main_to_sub(window, frame, image_name)
        self.assertEqual(len(result), 3)
        self.coord_transform.save_sub_positions.assert_called_once()
        rotate_img_instance.save_rotated_positions.assert_called_once()

if __name__ == '__main__':
    unittest.main()