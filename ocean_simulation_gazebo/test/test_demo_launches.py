#!/usr/bin/env python3
"""Parse the real launch graph without starting ROS or Gazebo."""

import shlex
import unittest
from pathlib import Path
from unittest import mock
import xml.etree.ElementTree as ET

import roslaunch.config
import roslaunch.substitution_args
import roslaunch.xmlloader
import rospkg

from test_map1_worlds import PACKAGE


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
                        self.assertNotIn(include.findtext("uri"),
                                         ("model://wamv", "model://lite_quad_uav", "model://iris"))
                        self.assertNotIn(include.findtext("name"), ("wamv", "usv_1", "uav_1"))
                    self.assertFalse(world.findall("model"))
                    self.assertEqual(world.findtext("plugin/wind_obj/name"), "wamv")

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


if __name__ == "__main__":
    unittest.main()
