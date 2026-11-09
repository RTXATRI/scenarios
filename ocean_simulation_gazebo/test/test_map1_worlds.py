#!/usr/bin/env python3
"""Regression checks for the map1 layouts and shared WAM-V resources.

Run after sourcing the ROS / catkin workspaces:
    python3 -m unittest discover -s test -p 'test_map1_worlds.py' -v
"""

import math
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
                        self.assertEqual(uri, "model://dock_2018_static")
                    elif key.startswith("robotx_dock_2022"):
                        self.assertEqual(uri, "model://dock_2022_static")

    def test_static_docks_preserve_each_block_and_placard_pose(self):
        vrx = Path(rospkg.RosPack().get_path("vrx_gazebo")) / "models"
        for year, count, placards in ((2018, 20, 2), (2022, 22, 3)):
            model = ET.parse(PACKAGE / ("models/other/dock_%d_static/model.sdf" % year)).getroot().find("model")
            self.assertEqual(model.findtext("static"), "true")
            self.assertEqual(len(model.findall("link")), 1)
            self.assertFalse(model.findall(".//joint"))
            self.assertEqual(len(model.findall("model")), placards)
            self.assertTrue(all(m.findtext("static") == "true" for m in model.findall("model")))
            self.assertFalse(any(v.get("name") in ("bay_box", "bay_box_exterior")
                                 for v in model.findall(".//visual")))
            base = model.find("link")
            self.assertEqual(len(base.findall("collision")), count + (4 if year == 2018 else 0))
            self.assertEqual(len(base.findall("visual")), count + (4 if year == 2018 else 0))
            original = ET.parse(vrx / ("dock_%d_base_dynamic/model.sdf" % year)).getroot().find("model")
            for block in original.findall("include"):
                name = block.findtext("name")
                expected = tuple(map(float, block.findtext("pose").split()))
                collision = base.find("collision[@name='%s_collision']" % name)
                self.assertEqual(tuple(map(float, collision.findtext("pose").split())), expected)
                self.assertEqual(tuple(map(float, collision.findtext("geometry/box/size").split())), (2, 2, .4))
                visual = base.find("visual[@name='%s_visual']" % name)
                pose = tuple(map(float, visual.findtext("pose").split()))
                self.assertAlmostEqual(pose[0], expected[0] - .25)
                self.assertAlmostEqual(pose[1], expected[1] + .25)
                self.assertAlmostEqual(pose[2], expected[2] + .2)
            for index, placard in enumerate(model.findall("model"), 1):
                pose = tuple(map(float, placard.findtext("pose").split()))
                expected = ((7.5 if index == 1 else 6.5), 3, 1.6) if year == 2018 else (5.75, 3 + 6 * (index - 1), 1.5)
                self.assertEqual(pose[:3], expected)
                self.assertEqual(len(placard.findall(".//plugin")), 1)

    def test_red_side_walls_have_matching_visible_and_collision_geometry(self):
        self.check_solid_blocks('red_wall', 'Red', (8, 1.5, 2), (-3.5, 17.5), 'bay_box_exterior')

    def test_green_blocks_have_matching_visible_and_collision_geometry(self):
        self.check_solid_blocks('green_block', 'Green', (1.5, 4, 2), (1.5, 12.5), 'bay_box')

    def check_solid_blocks(self, prefix, color, size, centers, original_name):
        model = ET.parse(PACKAGE / "models/other/dock_2018_static/model.sdf").getroot().find("model")
        base = model.find("link")
        for side, x in zip(('left', 'right'), centers):
            collision = base.find("collision[@name='%s_%s_collision']" % (prefix, side))
            visual = base.find("visual[@name='%s_%s_visual']" % (prefix, side))
            self.assertIsNotNone(collision)
            self.assertIsNotNone(visual)
            self.assertEqual(semantic(collision.find("geometry")), semantic(visual.find("geometry")))
            self.assertEqual(semantic(collision.find("pose")), semantic(visual.find("pose")))
            self.assertEqual(tuple(map(float, collision.findtext("geometry/box/size").split())), size)
            self.assertEqual(tuple(map(float, collision.findtext("pose").split())), (x, 3, .35, 0, 0, 1.5707963267948966))
            self.assertEqual(visual.findtext("material/script/name"), "Gazebo/" + color)
            self.assertEqual(float(visual.findtext("transparency", "0")), 0.)
            self.assertFalse(visual.findall("plugin"))
            self.assertIsNone(collision.find("surface/contact/collide_without_contact"))
            self.assertIsNone(collision.find("surface/contact/collide_bitmask"))
        original = ET.parse(Path(rospkg.RosPack().get_path("vrx_gazebo")) /
                            "models/dock_2018_dynamic/model.sdf").getroot().find("model")
        for placard in original.findall("model"):
            old = placard.find("link[@name='bay_visualization']/visual[@name='%s']" % original_name)
            outer = list(map(float, placard.findtext("pose").split()))
            mount = list(map(float, placard.findtext("include/pose").split()))
            local = list(map(float, old.findtext("pose").split()))
            x, y = mount[0]+local[0], mount[1]+local[1]
            center = (outer[0]+math.cos(outer[5])*x-math.sin(outer[5])*y,
                      outer[1]+math.sin(outer[5])*x+math.cos(outer[5])*y,
                      outer[2]+mount[2]+local[2])
            side = 'left' if center[0] < sum(centers) / 2 else 'right'
            new = base.find("visual[@name='%s_%s_visual']" % (prefix, side))
            for a, b in zip(center, map(float, new.findtext("pose").split()[:3])):
                self.assertAlmostEqual(a, b, places=10)
            self.assertEqual(semantic(new.find('geometry')), semantic(old.find('geometry')))

    def test_static_buoys_and_projectile_keep_real_collisions(self):
        for name, count in (("robotx_light_buoy_static", 2), ("blue_projectile_static", 1)):
            model = ET.parse(PACKAGE / "models/other" / name / "model.sdf").getroot().find("model")
            self.assertEqual(model.findtext("static"), "true")
            self.assertEqual(len(model.findall(".//collision")), count)
            self.assertFalse(model.findall(".//joint"))
            self.assertFalse(any(p.get("filename") == "libbuoyancy_gazebo_plugin.so"
                                 for p in model.findall(".//plugin")))
            if name.startswith("robotx"):
                self.assertIsNotNone(model.find(".//plugin[@filename='liblight_buoy_plugin.so']"))
                body = model.find("link/collision[@name='collision_body']")
                self.assertEqual(float(body.findtext("geometry/cylinder/radius")), .4)
                self.assertEqual(float(body.findtext("geometry/cylinder/length")), 1.75)

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
        relatives = ["models/vehicle/wamv/model.sdf", "models/other/map1_ocean/model.sdf"]
        relatives += ["models/other/" + name + "/model.sdf" for name in
                      ("dock_2018_static", "dock_2022_static", "robotx_light_buoy_static", "blue_projectile_static")]
        for relative in relatives:
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
