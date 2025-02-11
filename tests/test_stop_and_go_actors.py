import unittest
import pygame
from stop_and_go_actors import Stop_Area, Stop_Line, Turn_Status, Pose, DynamicState, PhysicalProperties, SimState, Car, Path
import stop_and_go_globals as sg
from stop_and_go_data_type import CarTurn

class TestStopAndGoActors(unittest.TestCase):

    def setUp(self):
        pygame.init()
        self.window = pygame.display.set_mode((sg.WINDOW_WIDTH_PIXELS, sg.WINDOW_LENGTH_PIXELS))

    def tearDown(self):
        pygame.quit()

    def test_stop_area_initialization(self):
        stop_area = Stop_Area(10, 20, 30, 40, self.window)
        self.assertEqual(stop_area.x, 10)
        self.assertEqual(stop_area.y, 20)
        self.assertEqual(stop_area.width, 30)
        self.assertEqual(stop_area.length, 40)
        self.assertEqual(stop_area.corners[1], [0, 20, 50])
        self.assertEqual(stop_area.corners[2], [0, 50, 10])
        self.assertEqual(stop_area.corners[3], [0, 50, 20])
        self.assertEqual(stop_area.corners[4], [0, 10, 50])

    def test_stop_line_initialization(self):
        stop_line = Stop_Line(10, 20, 30, 40, 5)
        self.assertEqual(stop_line.start, (10, 20))
        self.assertEqual(stop_line.stop, (30, 40))
        self.assertEqual(stop_line.line_width, 5)
        self.assertEqual(stop_line.color, sg.RED)

    def test_turn_status_initialization(self):
        turn_status = Turn_Status(1, CarTurn.LEFT.value, 10, 20)
        self.assertFalse(turn_status.turning)
        self.assertFalse(turn_status.turned)
        self.assertEqual(turn_status.turn_center_x, 0.0)
        self.assertEqual(turn_status.turn_center_y, 0.0)
        self.assertEqual(turn_status.radius, 0.0)
        self.assertEqual(turn_status.car_seq, 1)
        self.assertEqual(turn_status.car_turn, CarTurn.LEFT.value)
        self.assertEqual(turn_status.car_width, 10)
        self.assertEqual(turn_status.car_length, 20)

    def test_pose_initialization(self):
        pose = Pose(10, 20, 30)
        self.assertEqual(pose.x, 10)
        self.assertEqual(pose.y, 20)
        self.assertEqual(pose.heading_angle, 30)

    def test_dynamic_state_initialization(self):
        dynamic_state = DynamicState()
        self.assertEqual(dynamic_state.speed, 0)
        self.assertEqual(dynamic_state.accl, 0)
        self.assertFalse(dynamic_state.set_speed)

    def test_physical_properties_initialization(self):
        physical_properties = PhysicalProperties(10, 20, 1)
        self.assertEqual(physical_properties.width, 10)
        self.assertEqual(physical_properties.length, 20)
        self.assertEqual(physical_properties.points, [])

    def test_sim_state_initialization(self):
        sim_state = SimState(1)
        self.assertEqual(sim_state.cur_state, CarState.CRUISE.value)
        self.assertEqual(sim_state.dp, 0)
        self.assertEqual(sim_state.t_dp, 0)
        self.assertEqual(sim_state.seq, 1)
        self.assertFalse(sim_state.end)
        self.assertFalse(sim_state.lock)
        self.assertFalse(sim_state.overlap)
        self.assertFalse(sim_state.stop_car)

    def test_car_initialization(self):
        sim = unittest.mock.Mock()
        car = Car(10, 20, 30, 40, 1, sim, 50, self.window)
        self.assertEqual(car.pose.x, 10)
        self.assertEqual(car.pose.y, 20)
        self.assertEqual(car.pose.heading_angle, 50)
        self.assertEqual(car.prev_pose.x, 0)
        self.assertEqual(car.prev_pose.y, 0)
        self.assertEqual(car.prev_pose.heading_angle, 0)
        self.assertEqual(car.initial_pose.x, 10)
        self.assertEqual(car.initial_pose.y, 20)
        self.assertEqual(car.initial_pose.heading_angle, 0)
        self.assertEqual(car.dynamic_state.speed, 0)
        self.assertEqual(car.physical_properties.width, 30)
        self.assertEqual(car.physical_properties.length, 40)
        self.assertEqual(car.sim_state.seq, 1)
        self.assertEqual(car.window, self.window)
        self.assertEqual(car.stop_timer, 0)
        self.assertEqual(car.sim, sim)
        self.assertEqual(car.time_index, 0)
        self.assertEqual(car.stop_time_index, 0)
        self.assertFalse(car.Turn_Status.turning)
        self.assertFalse(car.Turn_Status.turned)
        self.assertEqual(car.car_turn_path[sg.CAR_SEQ_1], [
            (sg.PATH_SEQ_0, HeadingDirection.EAST.value),
            (sg.PATH_SEQ_3, HeadingDirection.NORTH.value),
            (sg.PATH_SEQ_1, HeadingDirection.SOUTH.value),
        ])

    def test_path_initialization(self):
        path = Path(10, 20, 30, 40, 1, 50)
        self.assertEqual(path.start, (10, 20))
        self.assertEqual(path.stop, (30, 40))
        self.assertEqual(path.path_seq, 1)
        self.assertEqual(path.const_xy, 50)

if __name__ == '__main__':
    unittest.main()