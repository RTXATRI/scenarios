#!/usr/bin/env python3
"""Regression checks for the map1 layouts and shared WAM-V resources.

Run after sourcing the ROS / catkin workspaces:
    python3 -m unittest discover -s test -p 'test_map1_worlds.py' -v
"""

import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import rospkg


PACKAGE = Path(__file__).resolve().parents[1]
WORLDS = PACKAGE / "worlds/usv_path_planning"
MAPS = ("map1-1.world", "map1-2.world", "map1-3.world")
COMMON_NAMES = {
    "sydneyregatta", "post_0", "post_1", "post_2", "antenna",
    "ground_station_0", "ground_station_1", "ground_station_2",
    "blue_projectile", "ocean_waves", "robotx_dock_2018",
    "robotx_dock_2022",
} | {"robotx_dock_2018_" + str(i) for i in range(7)} | {
    "robotx_dock_2022_" + str(i) for i in range(3)
}
OBSTACLES = {
    "map1-1.world": {},
    "map1-2.world": {
        "custom_dock_block_1": (-556.5, 185, 0.5),
        "custom_dock_block_2": (-529, 208.5, 0.5),
        "custom_dock_block_3": (-555.7, 216.2, 0.5),
        "custom_dock_block_4": (-562, 218.8, 0.5),
        "custom_dock_block_5": (-547.5, 234.5, 1.5),
        "custom_dock_block_6": (-516, 234.5, 1.5),
    },
    "map1-3.world": {
        "custom_dock_block_1": (-556.5, 185, 0.5),
        "custom_dock_block_4": (-562, 218.8, 0.5),
        "custom_dock_block_6": (-516, 234.5, 1.5),
        "dynamic_obstacle_1": (-529, 215.5, 0.5),
        "dynamic_obstacle_2": (-547.5, 234.5, 0.5),
    },
}


def semantic(element):
    """Ignore formatting and equivalent numeric spellings, including -0."""
    value = " ".join((element.text or "").split())
    try:
        value = tuple(float(x) for x in value.split()) if value else ""
    except ValueError:
        pass
    return (element.tag, element.attrib, value,
            tuple(semantic(child) for child in element))


class Map1Tests(unittest.TestCase):
    def test_layouts_and_obstacle_variants(self):
        for name in MAPS:
            with self.subTest(world=name):
                path = WORLDS / name
                world = ET.parse(path).getroot().find("world")
                self.assertLessEqual(len(path.read_text().splitlines()), 350)
                self.assertIsNone(world.find("state"))
                self.assertFalse(world.findall("model"))
                includes = {x.findtext("name"): x for x in world.findall("include")}
                self.assertEqual(set(includes), COMMON_NAMES | set(OBSTACLES[name]))
                self.assertEqual(len(includes), len(world.findall("include")))
                self.assertNotIn("wamv", includes)
                for key, position in OBSTACLES[name].items():
                    self.assertEqual(tuple(map(float, includes[key].findtext("pose").split()[:3])), position)
                for key in COMMON_NAMES:
                    uri = includes[key].findtext("uri")
                    if key.startswith("robotx_dock_2018"):
                        self.assertEqual(uri, "model://dock_2018_dynamic")
                    elif key.startswith("robotx_dock_2022"):
                        self.assertEqual(uri, "model://map1_dock_2022")

    def test_2022_placards_attach_to_existing_spine_blocks(self):
        model = ET.parse(PACKAGE / "models/other/map1_dock_2022/model.sdf").getroot().find("model")
        for number, row in ((1, 1), (2, 4), (3, 7)):
            child = model.findtext("joint[@name='joint_placard%d']/child" % number)
            self.assertEqual(child, "robotx_dock_2022_base::dock_block_3_%d::link" % row)

    def test_boat_dynamics_and_sensor_interfaces(self):
        model = ET.parse(PACKAGE / "models/vehicle/wamv/model.sdf").getroot().find("model")
        self.assertEqual(model.get("name"), "wamv")
        self.assertEqual(len(model.findall("link")), 13)
        self.assertEqual(len(model.findall("joint")), 12)
        self.assertEqual(model.findtext("static"), "0")
        self.assertAlmostEqual(float(model.findtext("link/inertial/mass")), 250.185)
        thrust = model.find("plugin[@name='wamv_gazebo_thrust']")
        self.assertEqual(thrust.findtext("robotNamespace"), "wamv")
        self.assertEqual({t.findtext("cmdTopic") for t in thrust.findall("thruster")}, {
            "thrusters/" + side + "_thrust_cmd"
            for side in ("left_front", "right_front", "left_rear", "right_rear")
        })
        sensors = list(model.iter("sensor"))
        self.assertEqual([s.get("type") for s in sensors].count("camera"), 3)
        self.assertEqual([s.get("type") for s in sensors].count("gpu_ray"), 1)
        topics = {x.text for x in model.iter("topicName")}
        self.assertTrue({
            "wamv/sensors/gps/gps/fix", "wamv/sensors/imu/imu/data",
            "wamv/sensors/lidars/lidar_wamv/points",
            "wamv/sensors/pingers/pinger/range_bearing",
        }.issubset(topics))

    def test_portable_meshes_and_textures(self):
        ros = rospkg.RosPack()
        roots = [PACKAGE / "models/vehicle", PACKAGE / "models/other"] + [
            Path(ros.get_path(package)) / subdir for package, subdir in (
                ("wamv_description", "models"), ("wamv_gazebo", "models"),
                ("vrx_gazebo", "models"), ("wave_gazebo", "world_models"),
                ("wave_gazebo", "models"),
            )
        ]
        for relative in ("models/vehicle/wamv/model.sdf", "models/other/map1_ocean/model.sdf"):
            path = PACKAGE / relative
            self.assertNotIn("/home/", path.read_text())
            for uri in ET.parse(path).getroot().iter("uri"):
                value = uri.text.strip()
                if not value.startswith("model://"):
                    continue
                matches = [r / value[len("model://"):] for r in roots]
                found = next((p for p in matches if p.exists()), None)
                self.assertIsNotNone(found, value)
                if found.suffix == ".dae":
                    for image in ET.parse(found).findall(".//{*}library_images/{*}image/{*}init_from"):
                        self.assertTrue((found.parent / image.text).is_file(), image.text)

    def test_xacro_generates_environment_only(self):
        template = WORLDS / "map1_base.world.xacro"
        default = ET.fromstring(subprocess.check_output(["xacro", str(template)]))
        self.assertEqual(semantic(default), semantic(ET.parse(WORLDS / MAPS[0]).getroot()))
        self.assertIsNone(default.find("world/include[name='wamv']"))
        self.assertFalse(ET.parse(template).getroot().findall("{http://www.ros.org/wiki/xacro}arg"))


if __name__ == "__main__":
    unittest.main()
