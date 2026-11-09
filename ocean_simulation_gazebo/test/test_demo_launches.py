#!/usr/bin/env python3
"""Parse the real launch graph without starting ROS or Gazebo."""

import contextlib
import copy
import importlib.util
import io
import math
import shlex
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock
import xml.etree.ElementTree as ET

import roslaunch.config
import roslaunch.substitution_args
import roslaunch.xmlloader
import rospkg

PACKAGE = Path(__file__).resolve().parents[1]


def load(name, *arguments):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(PACKAGE / "launch" / name), config,
        argv=["gui:=false", *arguments], verbose=False)
    return config


def node(config, name):
    return next(n for n in config.nodes if n.name == name)


class DemoTests(unittest.TestCase):
    def test_worlds_contain_environment_only(self):
        for group in ("usv_path_planning", "uav_usv_coop"):
            for path in (PACKAGE / "worlds" / group).iterdir():
                if not path.is_file():
                    continue
                with self.subTest(world=path.name):
                    world = ET.parse(path).getroot().find("world")
                    self.assertIsNone(world.find("state"))
                    for include in world.iter("include"):
                        self.assertEqual(include.findtext("static"), "true")
                        self.assertNotIn(include.findtext("uri"),
                                         ("model://wamv", "model://lite_quad_uav", "model://iris"))
                        self.assertNotIn(include.findtext("name"), ("wamv", "usv_1", "uav_1"))
                    self.assertFalse(world.findall("model"))
                    self.assertEqual(world.findtext("plugin/wind_obj/name"), "wamv")

    def test_environment_resources_have_no_force_plugins_or_motion_joints(self):
        ros = rospkg.RosPack()
        roots = [PACKAGE / "models/other"] + [Path(ros.get_path(name)) / directory
            for name, directory in (("vrx_gazebo", "models"), ("wave_gazebo", "world_models"),
                                    ("wave_gazebo", "models"))]
        visited = set()

        def inspect(uri, inherited=False):
            if (uri, inherited) in visited:
                return
            visited.add((uri, inherited))
            folder = next((r / uri[8:] for r in roots if (r / uri[8:] / "model.config").is_file()), None)
            self.assertIsNotNone(folder, uri)
            config = ET.parse(folder / "model.config").getroot()
            model = ET.parse(folder / config.findtext("sdf").strip()).getroot().find("model")

            def nested(element, parent_static):
                fixed = element.findtext("static", str(parent_static).lower()) in ("true", "1")
                self.assertTrue(fixed, uri)
                self.assertFalse(element.findall("joint"), uri)
                self.assertFalse(any("buoyancy" in p.get("filename", "").lower()
                                     for p in element.findall(".//plugin")), uri)
                for include in element.findall("include"):
                    inspect(include.findtext("uri"), fixed)
                for child in element.findall("model"):
                    nested(child, fixed)

            nested(model, inherited)

        for group in ("usv_path_planning", "uav_usv_coop"):
            for path in (PACKAGE / "worlds" / group).glob("*.world"):
                for include in ET.parse(path).getroot().findall("world/include"):
                    inspect(include.findtext("uri"), True)

    def test_usv_demo_does_not_resolve_or_start_px4_or_mavros(self):
        original = roslaunch.substitution_args._find

        def guard(resolved, command, args, context):
            self.assertNotIn(args[0], ("px4", "mavros"))
            return original(resolved, command, args, context)

        with mock.patch.object(roslaunch.substitution_args, "_find", side_effect=guard):
            config = load("demo_usv.launch")
        self.assertEqual({(n.package, n.type) for n in config.nodes},
                         {("gazebo_ros", "gzserver"), ("gazebo_ros", "spawn_model")})
        self.assertEqual([n.name for n in config.nodes].count("spawn_wamv"), 1)

    def test_default_and_custom_vehicle_poses(self):
        for entry, vehicles in (("demo_usv.launch", ("usv",)),
                                ("demo_uav_usv.launch", ("usv", "uav"))):
            defaults = {"usv": (-570, 167, 0.1, 0, 0, 1),
                        "uav": (-584.75, 177, 1.75, 0, 0, 0)}
            custom = {"usv": (-580, 170, 0.2, 0.1, -0.2, 0.5),
                      "uav": (-585, 178, 2.2, -0.1, 0.2, 1.57)}
            for poses in (defaults, custom):
                arguments = [vehicle + "_" + axis + ":=" + str(value)
                             for vehicle in vehicles
                             for axis, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), poses[vehicle])]
                config = load(entry, *arguments)
                for vehicle in vehicles:
                    spawn = node(config, "spawn_wamv" if vehicle == "usv" else "spawn_uav")
                    flags = shlex.split(spawn.args)
                    for flag, value in zip(("-x", "-y", "-z", "-R", "-P", "-Y"), poses[vehicle]):
                        self.assertEqual(float(flags[flags.index(flag) + 1]), value)
                    self.assertEqual(flags[flags.index("-model") + 1], "wamv" if vehicle == "usv" else "uav_1")
                    self.assertTrue(Path(flags[flags.index("-file") + 1]).is_file())
                    self.assertEqual(flags[flags.index("-reference_frame") + 1], "world")
                    self.assertNotIn("-unpause", flags)
                    self.assertFalse(spawn.required)
                    self.assertFalse(spawn.respawn)
                    if vehicle == "usv":
                        self.assertEqual(flags[flags.index("-robot_namespace") + 1], "")

    def test_cooperative_px4_resources_and_mavros_namespace(self):
        config = load("demo_uav_usv.launch", "fcu_url:=udp://:14640@localhost:14657", "gcs_url:=udp://@127.0.0.1:14650")
        sitl = node(config, "sitl")
        self.assertEqual(sitl.package, "px4")
        self.assertEqual(dict(sitl.env_args)["PX4_SIM_MODEL"], "gazebo-classic_iris")
        self.assertIn("-d", shlex.split(sitl.args))
        mavros = node(config, "mavros")
        self.assertEqual(mavros.namespace, "/uav_1/")
        self.assertEqual(config.params["/uav_1/mavros/fcu_url"].value, "udp://:14640@localhost:14657")
        self.assertEqual(config.params["/uav_1/mavros/gcs_url"].value, "udp://@127.0.0.1:14650")
        environment = dict(node(config, "gazebo").env_args)
        roots = environment["GAZEBO_MODEL_PATH"].split(":")
        px4 = Path(rospkg.RosPack().get_path("px4"))
        self.assertEqual(roots[0], str(px4 / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"))
        gps = next(Path(root) / "gps" for root in roots if (Path(root) / "gps/model.config").is_file())
        self.assertTrue((gps / ET.parse(gps / "model.config").getroot().findtext("sdf")).is_file())
        self.assertTrue(any((Path(root) / "gps/mesh/gps.dae").is_file() for root in roots))

    def test_default_worlds_and_paused_override(self):
        for entry, world in (("demo_usv.launch", "usv_path_planning/map1-3.world"),
                             ("demo_uav_usv.launch", "uav_usv_coop/map1.world")):
            config = load(entry)
            self.assertIn(str(PACKAGE / "worlds" / world), shlex.split(node(config, "gazebo").args))
            self.assertNotIn("-u", shlex.split(node(config, "gazebo").args))
            self.assertTrue(config.params["/use_sim_time"].value)
            custom = PACKAGE / "worlds/usv_path_planning/map1-1.world"
            config = load(entry, "world:=" + str(custom), "paused:=true", "verbose:=true")
            args = shlex.split(node(config, "gazebo").args)
            self.assertIn(str(custom), args)
            self.assertIn("-u", args)
            self.assertIn("--verbose", args)


class CircleDemoTests(unittest.TestCase):
    def test_clear_water_environment_preserves_shore_and_physics(self):
        world = ET.parse(PACKAGE / 'worlds/uav_usv_coop/map1_circle.world').getroot().find('world')
        base = ET.parse(PACKAGE / 'worlds/usv_path_planning/map1-1.world').getroot().find('world')
        expected = {'sydneyregatta', 'post_0', 'post_1', 'post_2', 'antenna', 'ocean_waves',
                    'ground_station_0', 'ground_station_1', 'ground_station_2'}
        includes = {element.findtext('name'): element for element in world.findall('include')}
        self.assertEqual(set(includes), expected)
        for element in base.findall('include'):
            name = element.findtext('name')
            if name in expected:
                self.assertEqual(ET.tostring(includes[name]), ET.tostring(element))
        for name in ('physics', 'spherical_coordinates', 'gravity', 'magnetic_field', 'plugin'):
            self.assertEqual(ET.tostring(world.find(name)), ET.tostring(base.find(name)))

    def test_simulation_only_root_mavros_and_synchronized_spawn(self):
        graph = load('demo_uav_usv_circle.launch', 'paused:=true')
        self.assertEqual(len(graph.nodes), 6)
        self.assertFalse({'std_usv', 'lite_quad_uav'}.intersection(n.package for n in graph.nodes))
        self.assertEqual(sum(n.type == 'mavros_node' for n in graph.nodes), 1)
        self.assertEqual(sum(n.type == 'px4' for n in graph.nodes), 1)
        self.assertEqual(sum(n.type == 'spawn_model' for n in graph.nodes), 2)
        self.assertEqual(node(graph, 'mavros').namespace, '/')
        self.assertEqual(graph.params['/mavros/fcu_url'].value, 'udp://:14540@localhost:14557')
        motion = node(graph, 'circle_motion')
        self.assertEqual(motion.namespace, '/wamv/simulation/')
        self.assertFalse(motion.required)
        self.assertFalse(motion.respawn)
        self.assertIn('-u', shlex.split(node(graph, 'gazebo').args))
        self.assertIn('libwamv_circle_motion_plugin.so', shlex.split(node(graph, 'gazebo').args))
        for name, position in (('spawn_wamv', (-565., 180., .1)),
                               ('spawn_uav', (-584.75, 177., .75))):
            flags = shlex.split(node(graph, name).args)
            self.assertEqual(tuple(float(flags[flags.index(flag)+1]) for flag in ('-x', '-y', '-z')), position)
            self.assertNotIn('-unpause', flags)
        flags = shlex.split(node(graph, 'spawn_wamv').args)
        self.assertAlmostEqual(float(flags[flags.index('-Y')+1]), math.pi/2.)
        self.assertIn('wamv_apriltag', flags[flags.index('-file')+1])

    def test_custom_circle_direction_and_aircraft_pose(self):
        graph = load('demo_uav_usv_circle.launch', 'circle_center_x:=-568', 'circle_center_y:=182',
                     'circle_radius:=3', 'circle_speed:=0.8', 'clockwise:=true', 'roll_amplitude:=0',
                     'uav_x:=-583', 'uav_yaw:=0.4', 'fcu_url:=udp://:14640@localhost:14657')
        prefix = '/wamv/simulation/circle_motion/'
        self.assertTrue(graph.params[prefix+'clockwise'].value)
        self.assertEqual(graph.params[prefix+'circle_speed'].value, .8)
        self.assertEqual(graph.params[prefix+'roll_amplitude'].value, 0.)
        flags = shlex.split(node(graph, 'spawn_wamv').args)
        self.assertEqual(float(flags[flags.index('-x')+1]), -565.)
        self.assertEqual(float(flags[flags.index('-y')+1]), 182.)
        self.assertAlmostEqual(float(flags[flags.index('-Y')+1]), -math.pi/2.)
        flags = shlex.split(node(graph, 'spawn_uav').args)
        self.assertEqual(float(flags[flags.index('-x')+1]), -583.)
        self.assertEqual(float(flags[flags.index('-Y')+1]), .4)


class CircleSupervisorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('_circle_supervisor_tests', PACKAGE / 'scripts/wamv_circle_motion.py')
        cls.motion = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.motion)

    def make_node(self):
        node = self.motion.WamvCircleMotion.__new__(self.motion.WamvCircleMotion)
        node.rospy = mock.Mock()
        node.rospy.is_shutdown.return_value = False
        node.parameters = self.motion.motion_parameters()
        node.start_attempted = False
        node.finished = threading.Event()
        node.stop_lock = threading.Lock()
        reply = type('Reply', (), {'success': True, 'message': 'ready'})()
        node.start_service = mock.Mock(return_value=reply)
        node.keepalive_service = mock.Mock(return_value=reply)
        node.stop_service = mock.Mock(return_value=reply)
        return node

    def test_invalid_parameters(self):
        for name in self.motion.DEFAULTS:
            if name == 'clockwise':
                continue
            for value in (math.nan, math.inf, True):
                with self.subTest(name=name, value=value), self.assertRaises((ValueError, TypeError)):
                    self.motion.motion_parameters(**{name: value})
        for name in ('circle_radius', 'roll_period', 'pitch_period', 'heave_period', 'motion_rate'):
            with self.assertRaises(ValueError):
                self.motion.motion_parameters(**{name: 0.})
        for name in ('circle_speed', 'roll_amplitude', 'pitch_amplitude', 'heave_amplitude'):
            with self.assertRaises(ValueError):
                self.motion.motion_parameters(**{name: -1.})
        with self.assertRaises(ValueError):
            self.motion.motion_parameters(clockwise='false')

    def test_startup_waits_for_model_with_a_wall_deadline(self):
        node = self.make_node()
        node.start_service.side_effect = [type('Reply', (), {'success': False, 'message': 'waiting for model wamv'})(),
                                          type('Reply', (), {'success': True, 'message': 'ready'})()]
        node.wait_ready()
        self.assertEqual(node.start_service.call_count, 2)
        self.assertEqual(node.rospy.wait_for_service.call_count, 3)
        with mock.patch.object(self.motion, 'READY_TIMEOUT', 0.):
            with self.assertRaises(TimeoutError):
                self.make_node().wait_ready()

    def test_plugin_failure_exits_and_still_stops_after_partial_start(self):
        node = self.make_node()
        node.start_service.return_value = type('Reply', (), {'success': False, 'message': 'invalid configuration'})()
        with self.assertRaisesRegex(RuntimeError, 'invalid configuration'):
            node.wait_ready()
        self.assertTrue(node.stop())
        node.stop_service.assert_called_once()

    def test_heartbeat_uses_wall_time_even_with_paused_simulation_clock(self):
        node = self.make_node()
        node.finished = mock.Mock()
        node.finished.is_set.side_effect = [False, True]
        node.run()
        node.keepalive_service.assert_called_once()
        node.finished.wait.assert_called_once_with(.02)
        node.rospy.Time.now.assert_not_called()
        node.rospy.Rate.assert_not_called()

    def test_backend_failure_reaches_supervisor(self):
        node = self.make_node()
        node.keepalive_service.return_value = type('Reply', (), {'success': False, 'message': 'model removed'})()
        with self.assertRaisesRegex(RuntimeError, 'model removed'):
            node.run()
        self.assertTrue(node.stop())

    def test_stop_before_start_is_harmless_and_rpc_failure_is_reported(self):
        node = self.make_node()
        self.assertTrue(node.stop())
        node.stop_service.assert_not_called()
        node.start_attempted = True
        node.stop_service.side_effect = OSError('connection lost')
        self.assertFalse(node.stop())
        node.rospy.logwarn.assert_called_once()

    def test_wall_timeout_does_not_block_cleanup(self):
        release = threading.Event()
        try:
            with self.assertRaises(TimeoutError):
                self.motion.wall_call(lambda: release.wait(), .02)
        finally:
            release.set()
        with self.assertRaisesRegex(OSError, 'transport'):
            self.motion.wall_call(mock.Mock(side_effect=OSError('transport')), 1.)

    def test_main_rejects_invalid_parameters_before_starting_plugin(self):
        import rospy
        with mock.patch.object(rospy, 'init_node'), \
                mock.patch.object(rospy, 'get_param', side_effect=lambda name, default: -1. if name=='~circle_radius' else default), \
                mock.patch.object(rospy, 'logerr') as error, \
                mock.patch.object(self.motion, 'WamvCircleMotion') as node:
            self.assertEqual(self.motion.main(), 1)
            node.assert_not_called()
            error.assert_called_once()


class NavigationAssetToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = PACKAGE / 'scripts/generate_navigation_maps.py'
        spec = importlib.util.spec_from_file_location('_simulation_map_tool_test', str(path))
        cls.generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.generator)

    def test_both_output_paths_are_required(self):
        for arguments in ([], ['--output-dir', 'maps'], ['--model-output', 'model.yaml']):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()), \
                    mock.patch.object(self.generator, 'generate') as generate:
                with self.assertRaises(SystemExit) as error:
                    self.generator.main(arguments)
                self.assertEqual(error.exception.code, 2)
                generate.assert_not_called()

    def test_explicit_outputs_and_read_only_check_without_control_package(self):
        original = rospkg.RosPack.get_path
        def resolve(ros, name):
            self.assertNotEqual(name, 'std_usv', 'The simulation tool must not resolve its caller')
            return original(ros, name)
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            maps, model = directory / 'maps', directory / 'control/model.yaml'
            with mock.patch.object(rospkg.RosPack, 'get_path', new=resolve):
                outputs = self.generator.generate(maps, model)
            self.assertEqual(set(outputs), {maps / name for name in (
                'map1.yaml', 'map1.pgm', 'map2.yaml', 'map2.pgm', 'provenance.yaml')} | {model})
            self.assertFalse(maps.exists())
            self.assertFalse(model.parent.exists())
            arguments = ['--output-dir', str(maps), '--model-output', str(model)]
            with mock.patch.object(self.generator, 'generate', return_value=outputs), \
                    contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(self.generator.main(arguments + ['--check']), 1)
                self.assertFalse(maps.exists())
                self.assertFalse(model.parent.exists())
                self.assertEqual(self.generator.main(arguments), 0)
                self.assertEqual(self.generator.main(arguments + ['--check']), 0)
                model.write_bytes(b'stale calibration')
                self.assertEqual(self.generator.main(arguments + ['--check']), 1)
                self.assertEqual(model.read_bytes(), b'stale calibration')
            for path, content in outputs.items():
                if path != model:
                    self.assertEqual(path.read_bytes(), content)

    def test_simulation_package_does_not_depend_on_the_control_package(self):
        manifest = ET.parse(PACKAGE / 'package.xml').getroot()
        self.assertNotIn('std_usv', [element.text for element in manifest
                                   if element.tag.endswith('depend')])


class SimulationSpeedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'scripts/set_simulation_speed.py'
        spec = importlib.util.spec_from_file_location('_simulation_speed_test', str(path))
        cls.speed = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.speed)

    def physics(self, rate=1000.):
        from gazebo_msgs.srv import GetPhysicsPropertiesResponse
        p = GetPhysicsPropertiesResponse()
        p.success, p.time_step, p.max_update_rate, p.pause = True, .001, rate, True
        p.gravity.z = -9.8
        p.ode_config.sor_pgs_iters = 50
        p.ode_config.sor_pgs_w = 1.3
        p.ode_config.erp = .2
        p.ode_config.max_contacts = 20
        return p

    def test_rates_use_current_step_and_zero_means_unlimited(self):
        for speed, rate in ((.5, 500.), (1., 1000.), (2., 2000.), (3., 3000.), (0., 0.)):
            self.assertEqual(self.speed.target_update_rate(speed, .001), rate)
        self.assertEqual(self.speed.target_update_rate(2., .002), 1000.)

    def test_invalid_speed_step_overflow_and_underflow_are_rejected(self):
        for value in (-1, float('nan'), float('inf'), -float('inf'), True, 'invalid'):
            with self.subTest(speed=value), self.assertRaises((ValueError, TypeError)):
                self.speed.validate_speed(value)
        for step in (0., -1., float('nan'), float('inf')):
            with self.subTest(step=step), self.assertRaises(ValueError):
                self.speed.target_update_rate(0., step)
        for speed, step in ((1e308, 1e-308), (5e-324, 1e308)):
            with self.assertRaises(ValueError):
                self.speed.target_update_rate(speed, step)

    def test_configuration_preserves_physics_and_paused_state(self):
        before = self.physics()
        untouched = copy.deepcopy(before)
        after = copy.deepcopy(before)
        after.max_update_rate = 2000.
        get = mock.Mock(side_effect=[before, after])
        set_ = mock.Mock(return_value=type('Reply', (), {'success': True})())
        result = self.speed.configure_speed(2., get, set_)
        set_.assert_called_once_with(before.time_step, 2000., before.gravity, before.ode_config)
        self.assertEqual(before, untouched)
        self.assertTrue(result['paused'])
        self.assertEqual(result['actual_update_rate'], 2000.)

    def test_existing_rate_is_read_back_without_rewriting(self):
        before = self.physics()
        get, set_ = mock.Mock(side_effect=[before, copy.deepcopy(before)]), mock.Mock()
        result = self.speed.configure_speed(1., get, set_)
        self.assertFalse(result['changed'])
        self.assertEqual(get.call_count, 2)
        set_.assert_not_called()

    def test_unlimited_is_written_as_exact_zero(self):
        before, after = self.physics(), self.physics(0.)
        get = mock.Mock(side_effect=[before, after])
        set_ = mock.Mock(return_value=type('Reply', (), {'success': True})())
        self.assertEqual(self.speed.configure_speed(0., get, set_)['actual_update_rate'], 0.)
        self.assertEqual(set_.call_args.args[1], 0.)
        self.assertFalse(self.speed.matching_rate(1e-12, 0.))

    def test_failed_reads_writes_and_transport_errors_are_reported(self):
        failed = self.physics()
        failed.success, failed.status_message = False, 'test failure'
        with self.assertRaisesRegex(RuntimeError, 'GetPhysicsProperties failed'):
            self.speed.configure_speed(2., mock.Mock(return_value=failed), mock.Mock())
        with self.assertRaisesRegex(RuntimeError, 'SetPhysicsProperties failed'):
            self.speed.configure_speed(2., mock.Mock(return_value=self.physics()), mock.Mock(return_value=failed))
        with self.assertRaisesRegex(RuntimeError, 'Physics readback failed'):
            self.speed.configure_speed(1., mock.Mock(side_effect=[self.physics(), failed]), mock.Mock())
        with self.assertRaisesRegex(OSError, 'transport failed'):
            self.speed.configure_speed(2., mock.Mock(side_effect=OSError('transport failed')), mock.Mock())

    def test_incorrect_readback_never_reports_success(self):
        with self.assertRaisesRegex(RuntimeError, 'rate mismatch'):
            self.speed.configure_speed(2., mock.Mock(return_value=self.physics()), mock.Mock(
                return_value=type('Reply', (), {'success': True})()))
        for field in ('time_step', 'gravity', 'ode_config', 'pause'):
            before, after = self.physics(), self.physics()
            if field == 'time_step': after.time_step = .002
            elif field == 'gravity': after.gravity.z = 0.
            elif field == 'ode_config': after.ode_config.sor_pgs_iters = 1
            else: after.pause = False
            with self.subTest(field=field), self.assertRaisesRegex(RuntimeError, field):
                self.speed.configure_speed(1., mock.Mock(side_effect=[before, after]), mock.Mock())

    def test_wall_timeout_bounds_a_blocked_call_without_simulation_clock(self):
        release = threading.Event()
        try:
            with self.assertRaises(TimeoutError):
                self.speed.with_wall_timeout(lambda: release.wait(), timeout=.02)
        finally:
            release.set()
        self.assertEqual(self.speed.with_wall_timeout(lambda: 7, timeout=1.), 7)
        with self.assertRaisesRegex(OSError, 'worker failure'):
            self.speed.with_wall_timeout(mock.Mock(side_effect=OSError('worker failure')), timeout=1.)

    def test_main_exits_zero_on_success_and_nonzero_on_invalid_input(self):
        import rospy
        before = self.physics()
        get, set_ = mock.Mock(return_value=before), mock.Mock()
        def proxy(name, _service_type):
            return get if name.endswith('get_physics_properties') else set_
        with mock.patch.object(rospy, 'init_node'), mock.patch.object(rospy, 'wait_for_service'), \
                mock.patch.object(rospy, 'ServiceProxy', side_effect=proxy), \
                mock.patch.object(rospy, 'get_param', return_value=1.), \
                mock.patch.object(rospy, 'loginfo') as info:
            self.assertEqual(self.speed.main(), 0)
            info.assert_called_once()
            set_.assert_not_called()
        with mock.patch.object(rospy, 'init_node'), \
                mock.patch.object(rospy, 'get_param', return_value=-1.), \
                mock.patch.object(rospy, 'wait_for_service') as wait, \
                mock.patch.object(rospy, 'logfatal') as fatal:
            self.assertEqual(self.speed.main(), 1)
            wait.assert_not_called()
            fatal.assert_called_once()


if __name__ == "__main__":
    unittest.main()
