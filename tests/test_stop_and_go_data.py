import unittest
from unittest.mock import Mock, patch
from stop_and_go_data import Save_Sim_Flow_Data, Vehicle_State, Road_State

# test_stop_and_go_data.py


class TestSaveSimFlowData(unittest.TestCase):

    @patch('stop_and_go_data.CollisionCheck')
    def setUp(self, MockCollisionCheck):
        self.car_nums = 4
        self.save_sim_flow_data = Save_Sim_Flow_Data(self.car_nums)
        self.mock_collision_check = MockCollisionCheck.return_value

    def test_init(self):
        self.assertEqual(len(self.save_sim_flow_data.sim_data_dict_list), self.car_nums)
        self.assertEqual(self.save_sim_flow_data.car_nums, self.car_nums)
        self.assertIsInstance(self.save_sim_flow_data.collision_check, Mock)

    def test_populate_road_states(self):
        stop_line_list = [Mock(), Mock(), Mock(), Mock()]
        for i in range(4):
            stop_line_list[i].start = (i, i)
            stop_line_list[i].stop = (i + 1, i + 1)
        road_state = Road_State()
        result = self.save_sim_flow_data.populate_road_states(stop_line_list, road_state)
        for i in range(4):
            self.assertEqual(result.stop_line_states[i].center_x_p, (i + (i + 1)) / 2)
            self.assertEqual(result.stop_line_states[i].center_y_p, (i + (i + 1)) / 2)

    def test_populate_vehicle_states(self):
        vehicle_state = Vehicle_State()
        car = Mock()
        car.pose.x = 1.0
        car.pose.y = 2.0
        car.dynamic_state.speed = 3.0
        car.dynamic_state.accl = 4.0
        car.pose.heading_angle = 5.0
        car.physical_properties.width = 6.0
        car.physical_properties.length = 7.0
        car.sim_state.cur_state = 'CRUISE'
        car.sim.turn_no = {Mock(): 'no'}
        car.sim.turn = Mock()
        center_x = 8.0
        center_y = 9.0
        boundary_points = [Mock(), Mock()]
        self.save_sim_flow_data.populate_vehicle_states(vehicle_state, car, center_x, center_y, boundary_points)
        self.assertEqual(vehicle_state.loc_main_x_p, 1.0)
        self.assertEqual(vehicle_state.loc_main_y_p, 2.0)
        self.assertEqual(vehicle_state.loc_x_p, 1.0)
        self.assertEqual(vehicle_state.loc_y_p, 2.0)
        self.assertEqual(vehicle_state.center_x_p, 8.0)
        self.assertEqual(vehicle_state.center_y_p, 9.0)
        self.assertEqual(vehicle_state.center_main_x_p, 8.0)
        self.assertEqual(vehicle_state.center_main_y_p, 9.0)
        self.assertEqual(vehicle_state.speed_pps, 3.0)
        self.assertEqual(vehicle_state.acc_ppss, 4.0)
        self.assertEqual(vehicle_state.heading_rad, 5.0)
        self.assertEqual(vehicle_state.rot_heading_rad, 5.0)
        self.assertEqual(vehicle_state.boundary, boundary_points)
        self.assertEqual(vehicle_state.width_p, 6.0)
        self.assertEqual(vehicle_state.length_p, 7.0)
        self.assertEqual(vehicle_state.cur_state, 'CRUISE')
        self.assertEqual(vehicle_state.cur_turn, 'no')

    @patch('stop_and_go_data.sg')
    def test_populate_sim_data_car_seq(self, mock_sg):
        mock_sg.NUMBER_OF_PATHS = 4
        mock_sg.DEBUG = 0
        mock_sg.ENABLE_VEHICLE_COLLISION_CHECK = False
        car_list = [Mock(), Mock(), Mock(), Mock()]
        stop_line_list = [Mock(), Mock(), Mock(), Mock()]
        frame_state = Mock()
        frame_state.start_frame = 0
        frame_state.end_frame = 100
        for car in car_list:
            car.time_index = 0
            car.stop_time_index = 0
            car.sim_state.seq = 1
            car.sim_state.cur_state = 'CRUISE'
            car.pose.x = 1.0
            car.pose.y = 2.0
            car.dynamic_state.speed = 3.0
            car.dynamic_state.accl = 4.0
            car.pose.heading_angle = 5.0
            car.physical_properties.width = 6.0
            car.physical_properties.length = 7.0
            car.sim.turn_no = {Mock(): 'no'}
            car.sim.turn = Mock()
            car.get_car_center.return_value = (8.0, 9.0)
            car.get_car_point_list.return_value = [Mock(), Mock()]
        result = self.save_sim_flow_data.populate_sim_data_car_seq(car_list, stop_line_list, frame_state)
        self.assertFalse(result)
        self.assertIn(0.0, self.save_sim_flow_data.sim_data_dict_list[0])
        self.assertIsInstance(self.save_sim_flow_data.sim_data_dict_list[0][0.0][1], Vehicle_State)
        self.assertIsInstance(self.save_sim_flow_data.sim_data_dict_list[0][0.0][2], Road_State)

if __name__ == '__main__':
    unittest.main()
