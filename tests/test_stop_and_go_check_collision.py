import unittest
from stop_and_go_check_collision import CollisionCheck
from stop_and_go_data_type import CarState, HeadingDirection
import stop_and_go_globals as sg

class MockCar:
    def __init__(self, pose_x, pose_y, seq, turn, cur_state, speed):
        self.pose = MockPose(pose_x, pose_y)
        self.sim_state = MockSimState(seq, cur_state)
        self.sim = MockSim(turn)
        self.car_turn_path = {seq: {turn: [None, HeadingDirection.NORTH.value]}}
        self.dynamic_state = MockDynamicState(speed)

class MockPose:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class MockSimState:
    def __init__(self, seq, cur_state):
        self.seq = seq
        self.cur_state = cur_state

class MockSim:
    def __init__(self, turn):
        self.turn = turn

class MockDynamicState:
    def __init__(self, speed):
        self.speed = speed
        self.set_speed = False

class TestCollisionCheck(unittest.TestCase):
    def setUp(self):
        self.collision_check = CollisionCheck()
        sg.DEBUG = sg.DEBUG_LEVEL_1
        sg.LEAD_VEHICLE_DISTANCE_PIXELS = 10

    def test_get_lead_trail_car(self):
        lead_vehicle, trail_vehicle, dist = self.collision_check.get_lead_trail_car(5, 10, 1, 2, HeadingDirection.NORTH.value)
        self.assertEqual(lead_vehicle, 2)
        self.assertEqual(trail_vehicle, 1)
        self.assertEqual(dist, 5)

    def test_get_dist_leading_trailing_car(self):
        car1 = MockCar(5, 10, 0, 0, CarState.CRUISE_A.value, 5)
        car2 = MockCar(5, 15, 1, 0, CarState.CRUISE_A.value, 5)
        car_list = [car1, car2]
        dist, lead_vehicle, trail_vehicle = self.collision_check.get_dist_leading_trailing_car(car_list, 0, 1)
        self.assertEqual(dist, 5)
        self.assertEqual(lead_vehicle, 1)
        self.assertEqual(trail_vehicle, 0)

    def test_check_leading_vehicle_distance(self):
        car1 = MockCar(5, 10, 0, 0, CarState.CRUISE_A.value, 5)
        car2 = MockCar(5, 15, 1, 0, CarState.CRUISE_A.value, 10)
        car_list = [car1, car2]
        self.collision_check.check_leading_vehicle_distance(car_list)
        self.assertTrue(car2.dynamic_state.set_speed)
        self.assertEqual(car2.dynamic_state.speed, car1.dynamic_state.speed)

if __name__ == '__main__':
    unittest.main()