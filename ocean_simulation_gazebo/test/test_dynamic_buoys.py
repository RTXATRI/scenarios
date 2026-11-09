#!/usr/bin/env python3
"""Static buoy XY routes, fixed height/attitude and simulation-time movement."""

import importlib.util
from copy import deepcopy
import math
import pathlib
import types
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "navigation_dynamic_buoys.py"
SPEC = importlib.util.spec_from_file_location("navigation_dynamic_buoys", str(SCRIPT))
buoys = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(buoys)



class DynamicBuoyTests(unittest.TestCase):
    def test_world_selection(self):
        self.assertEqual(set(buoys.routes_for_world("/worlds/map1-3.world")),
                         {"dynamic_obstacle_1", "dynamic_obstacle_2"})
        self.assertEqual(set(buoys.routes_for_world("map2-1")), {"dynamic_obstacle_3"})
        for world in ("", "map1-1.world", "map1-2.world", "map2-2.world", "map1.world"):
            self.assertEqual(buoys.routes_for_world(world), {})

    def test_original_route_endpoints(self):
        self.assertEqual(buoys.ROUTES["dynamic_obstacle_1"], ((-529., 215.5), (-562., 215.5)))
        self.assertEqual(buoys.ROUTES["dynamic_obstacle_2"],
                         ((-547.5, 234.5), (-547.5, 246.5), (-516., 246.5)))
        self.assertEqual(buoys.ROUTES["dynamic_obstacle_3"], ((-498.5, 196.), (-498.5, 234.5)))

    def test_reflected_motion_passes_corners_in_both_directions(self):
        route = buoys.ROUTES["dynamic_obstacle_2"]
        length = buoys.route_length(route)
        self.assertAlmostEqual(length, 43.5)
        self.assertEqual(buoys.route_position(route, 0.), route[0])
        self.assertEqual(buoys.route_position(route, 12.), route[1])
        self.assertEqual(buoys.route_position(route, length), route[2])
        self.assertEqual(buoys.route_position(route, 2. * length - 12.), route[1])
        self.assertEqual(buoys.route_position(route, 2. * length), route[0])
        self.assertEqual(buoys.route_position(route, 20.), (-539.5, 246.5))
        self.assertEqual(buoys.route_position(route, 2. * length - 20.), (-539.5, 246.5))

    def test_pause_speed_and_large_time_step(self):
        route = buoys.ROUTES["dynamic_obstacle_1"]
        self.assertEqual(buoys.route_position(route, .7 * 10.), (-536., 215.5))
        self.assertEqual(buoys.route_position(route, 66. * 1000. + 5.), (-534., 215.5))
        for distance in range(200):
            x, y = buoys.route_position(route, distance)
            self.assertTrue(-562. <= x <= -529.)
            self.assertEqual(y, 215.5)

    def test_invalid_geometry_and_distance(self):
        for route in ((), ((0., 0.),), ((0., 0.), (0., 0.)),
                      ((math.nan, 0.), (1., 0.)), ((0., 0., .3), (1., 0., .3))):
            with self.assertRaises(ValueError):
                buoys.route_length(route)
        for distance in (-1., math.nan, math.inf):
            with self.assertRaises(ValueError):
                buoys.route_position(buoys.ROUTES["dynamic_obstacle_1"], distance)

    def node(self):
        node = buoys.NavigationDynamicBuoys.__new__(buoys.NavigationDynamicBuoys)
        node.clock = 10.
        node.calls = []
        node.rospy = types.SimpleNamespace(
            Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: node.clock)),
            ServiceException=RuntimeError, logwarn_throttle=lambda *args: None)
        node.routes = buoys.routes_for_world("map1-3")
        node.speed = .7
        node.present_models = set(node.routes)
        node.ready = False
        node.fixed_poses = {}
        node.last_time = None
        node.distance = 0.
        node.initial_pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=-529., y=215.5, z=.5),
            orientation=types.SimpleNamespace(x=0., y=0., z=math.sin(.2), w=math.cos(.2)))
        node.get_state = lambda *args: types.SimpleNamespace(
            success=True, status_message="", pose=deepcopy(node.initial_pose))
        node.set_state = lambda state: node.calls.append(state) or types.SimpleNamespace(success=True, status_message="")
        node.ModelState = lambda: types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(), orientation=types.SimpleNamespace()))
        return node

    def test_model_wait_pause_and_simulation_clock_motion(self):
        node = self.node()
        node.present_models = set()
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.calls, [])
        node.present_models = set(node.routes)
        node.update(None)
        self.assertTrue(node.ready)
        node.update(None)  # Paused ROS clock produces no service update.
        self.assertEqual(node.calls, [])
        node.clock = 11.
        node.update(None)
        self.assertEqual(len(node.calls), 2)
        self.assertAlmostEqual(node.calls[0].pose.position.x, -529.7)
        self.assertEqual(node.calls[0].pose.position.z, .5)
        self.assertEqual(vars(node.calls[0].pose.orientation), vars(node.initial_pose.orientation))
        self.assertAlmostEqual(node.distance, .7)

    def test_get_and_set_service_response_failures_preserve_wait_state(self):
        node = self.node()
        node.get_state = lambda *args: types.SimpleNamespace(success=False, status_message="not spawned")
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.calls, [])
        node.get_state = lambda *args: types.SimpleNamespace(
            success=True, status_message="", pose=deepcopy(node.initial_pose))
        node.update(None)
        node.clock = 11.
        node.set_state = lambda state: types.SimpleNamespace(success=False, status_message="missing model")
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.distance, 0.)

    def test_cached_height_and_attitude_are_not_aliased_to_output(self):
        node = self.node()
        node.initial_pose.position.z = 1.2
        node.update(None)
        node.initial_pose.position.z = 9.
        node.clock = 11.
        node.update(None)
        self.assertEqual(node.calls[0].pose.position.z, 1.2)
        node.calls[0].pose.orientation.w = 0.
        node.clock = 12.
        node.update(None)
        self.assertAlmostEqual(node.calls[-1].pose.orientation.w, math.cos(.2))

    def test_simulation_reset_rereads_pose_and_restarts_distance(self):
        node = self.node()
        node.update(None)
        node.clock = 11.
        node.update(None)
        self.assertAlmostEqual(node.distance, .7)
        node.initial_pose.position.z = .8
        node.clock = 1.
        node.update(None)
        self.assertTrue(node.ready)
        self.assertEqual(node.distance, 0.)
        node.clock = 2.
        node.update(None)
        first = next(state for state in reversed(node.calls) if state.model_name == 'dynamic_obstacle_1')
        self.assertAlmostEqual(first.pose.position.x, -529.7)
        self.assertEqual(first.pose.position.z, .8)

    def test_invalid_initial_pose_does_not_enable_motion(self):
        for invalid in (math.nan, math.inf):
            node = self.node()
            node.initial_pose.position.z = invalid
            node.update(None)
            self.assertFalse(node.ready)
            self.assertEqual(node.calls, [])
        node = self.node()
        for axis in ('x', 'y', 'z', 'w'):
            setattr(node.initial_pose.orientation, axis, 0.)
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.calls, [])

    def test_zero_speed_and_service_exception_keep_models_fixed(self):
        node = self.node()
        node.speed = 0.
        node.update(None)
        node.clock = 12.
        node.update(None)
        self.assertEqual(node.distance, 0.)
        self.assertEqual((node.calls[0].pose.position.x, node.calls[0].pose.position.y), (-529., 215.5))
        node = self.node()
        node.update(None)
        node.clock = 11.
        def failed(_state):
            raise RuntimeError('service unavailable')
        node.set_state = failed
        node.update(None)
        self.assertFalse(node.ready)
        self.assertEqual(node.distance, 0.)



if __name__ == "__main__":
    unittest.main()
