#!/usr/bin/env python3
"""Map2 regression checks; run with unittest discovery after sourcing ROS."""

import importlib.util
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import rospkg

# Load shared fixtures by file so catkin/nose's -P mode needs no sys.path edits.
SPEC = importlib.util.spec_from_file_location(
    "_map1_test_fixtures", Path(__file__).with_name("test_map1_worlds.py"))
fixtures = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fixtures)
COMMON_NAMES, PACKAGE, WORLDS, semantic = (
    fixtures.COMMON_NAMES, fixtures.PACKAGE, fixtures.WORLDS, fixtures.semantic)


MAPS = ("map2-2.world", "map2-1.world")  # Environment-only and additional-obstacle variants.
CLONES = {
    "robotx_dock_2022_clone": (
        "dock_2022_static", (-490.99, 196.534, -0.160401, -0.003809, -0.021412, -0.000363)),
    "robotx_dock_2018_1_clone": (
        "dock_2018_static", (-548.837, 175.353, -0.118926, 0.001312, -0.006723, -0.000006)),
    "robotx_dock_2022_0_clone": (
        "dock_2022_static", (-510.876, 204.626, -0.110702, -0.000626, -0.030207, -0.001586)),
}
OBSTACLES = {
    "custom_dock_block_a": (-470, 167, 0.5, 0, 0, 0),
    "custom_dock_block_b": (-470, 267, 0.5, 0, 0, 0),
    "custom_dock_block_c": (-570, 267, 0.5, 0, 0, 0),
    "dynamic_obstacle_3": (-498.5, 196, 0.5, 0, 0, 0),
}
for group, (x, y) in enumerate(((-552, 184.5), (-509, 193.5), (-511.5, 234.5), (-557, 213.5)), 1):
    for level, z in enumerate((0.3, 0.7, 1.1, 1.5), 1):
        OBSTACLES["custom_dock_block_%d-%d" % (group, level)] = (x, y, z, 0, 0, 0)


def includes(world):
    return {element.findtext("name"): element for element in world.findall("include")}


class Map2Tests(unittest.TestCase):
    def test_compact_layouts_and_shared_models(self):
        reference = includes(ET.parse(WORLDS / "map1-1.world").getroot().find("world"))
        for filename, count in zip(MAPS, (25, 45)):
            with self.subTest(world=filename):
                path = WORLDS / filename
                world = ET.parse(path).getroot().find("world")
                models = includes(world)
                expected = COMMON_NAMES | set(CLONES)
                if filename == MAPS[1]:
                    expected |= set(OBSTACLES)
                self.assertLessEqual(len(path.read_text().splitlines()), 350)
                self.assertNotIn("/home/", path.read_text())
                self.assertIsNone(world.find("state"))
                self.assertFalse(world.findall("model"))
                self.assertEqual(set(models), expected)
                self.assertEqual(len(models), count)
                self.assertEqual(len(world.findall("include")), count)
                for name in COMMON_NAMES:
                    self.assertEqual(semantic(models[name]), semantic(reference[name]), name)
                for name, (resource, pose) in CLONES.items():
                    self.assertEqual(models[name].findtext("uri"), "model://" + resource)
                    self.assertEqual(tuple(map(float, models[name].findtext("pose").split())), pose)

    def test_obstacle_positions_and_types(self):
        world = ET.parse(WORLDS / MAPS[1]).getroot().find("world")
        models = includes(world)
        for name, pose in OBSTACLES.items():
            with self.subTest(obstacle=name):
                element = models[name]
                self.assertEqual(tuple(map(float, element.findtext("pose").split())), pose)
                resource = "robotx_light_buoy_static" if name == "dynamic_obstacle_3" else "dock_block_4x4"
                self.assertEqual(element.findtext("uri"), "model://" + resource)

    def test_environment_and_original_overview(self):
        reference = ET.parse(WORLDS / "map1-1.world").getroot().find("world")
        for filename in MAPS:
            world = ET.parse(WORLDS / filename).getroot().find("world")
            self.assertEqual(world.get("name"), "robotx_example_course")
            for tag in ("scene", "spherical_coordinates", "light", "plugin", "gravity",
                        "magnetic_field", "atmosphere", "physics", "wind"):
                self.assertEqual(semantic(world.find(tag)), semantic(reference.find(tag)), tag)
            self.assertEqual(tuple(map(float, world.findtext("gui/camera/pose").split())),
                             (-530.104, 214.192, 145.393, 0.000026, 1.5698, 1.56892))

    def test_model_resources_and_obstacle_dynamics(self):
        ros = rospkg.RosPack()
        roots = [PACKAGE / "models/vehicle", PACKAGE / "models/other"] + [
            Path(ros.get_path(package)) / subdir for package, subdir in (
                ("wamv_description", "models"), ("wamv_gazebo", "models"),
                ("vrx_gazebo", "models"), ("wave_gazebo", "world_models"),
                ("wave_gazebo", "models"),
            )
        ]
        world = ET.parse(WORLDS / MAPS[1]).getroot().find("world")
        for uri in {element.findtext("uri") for element in world.findall("include")}:
            self.assertTrue(uri.startswith("model://"), uri)
            model_dir = next((root / uri[8:] for root in roots if (root / uri[8:] / "model.config").is_file()), None)
            self.assertIsNotNone(model_dir, uri)
            config = ET.parse(model_dir / "model.config").getroot()
            sdf = model_dir / config.findtext("sdf").strip()
            self.assertTrue(sdf.is_file(), str(sdf))
            model = ET.parse(sdf).getroot().find("model")
            if uri.endswith("dock_block_4x4"):
                self.assertIn(model.findtext("static"), ("true", "1"))
            elif uri.endswith("robotx_light_buoy_static"):
                self.assertIn(model.findtext("static"), ("true", "1"))
                self.assertFalse(model.findall("plugin"))
                self.assertIsNotNone(model.find(".//plugin[@filename='liblight_buoy_plugin.so']"))

    def test_xacro_generates_environment_only(self):
        template = WORLDS / "map2_base.world.xacro"
        default = ET.fromstring(subprocess.check_output(["xacro", str(template)]))
        self.assertEqual(semantic(default), semantic(ET.parse(WORLDS / MAPS[0]).getroot()))
        self.assertIsNone(default.find("world/include[name='wamv']"))
        self.assertFalse(ET.parse(template).getroot().findall("{http://www.ros.org/wiki/xacro}arg"))


if __name__ == "__main__":
    unittest.main()
