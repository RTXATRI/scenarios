#!/usr/bin/env python3
"""Opt-in Gazebo regression: assembly geometry, velocities and supervisor recovery.

Run with the ROS/dependency workspace environment and an OpenGL display available.
Only this script's ROS/Gazebo masters and process groups are used.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--package-path', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--duration', type=float, default=126.)
    parser.add_argument('--full-demo', action='store_true',
                        help='Also load PX4/MAVROS/UAV; clock reset is checked in the boat-only fixture')
    return parser.parse_args()


def free_port():
    with socket.socket() as connection:
        connection.bind(('127.0.0.1', 0))
        return connection.getsockname()[1]


def main():
    args = arguments()
    output, package = args.output_dir.resolve(), args.package_path.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not math.isfinite(args.duration) or args.duration <= 0.:
        raise ValueError('duration must be positive and finite')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % free_port(),
                      GAZEBO_MASTER_URI='http://127.0.0.1:%d' % free_port(),
                      ROS_IP='127.0.0.1', ROS_HOME=str(output / 'ros'),
                      ROS_LOG_DIR=str(output / 'ros/log'),
                      GAZEBO_LOG_PATH=str(output / 'gazebo_logs'),
                      GAZEBO_MODEL_DATABASE_URI='')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_PACKAGE_PATH'] = str(package.parent) + ':' + os.environ.get('ROS_PACKAGE_PATH', '')
    # --package-path can point at an isolated install prefix.
    if package.parent.name == 'share':
        library = package.parent.parent / 'lib'
        for variable in ['LD_LIBRARY_PATH', 'GAZEBO_PLUGIN_PATH']:
            os.environ[variable] = str(library) + ':' + os.environ.get(variable, '')
    for folder in ['ros/log', 'gazebo_logs', 'frames']:
        (output / folder).mkdir(parents=True, exist_ok=True)

    import cv2
    import numpy as np
    import rosnode
    import rosgraph
    import rospy
    from cv_bridge import CvBridge
    from gazebo_msgs.msg import LinkStates, ModelState
    from gazebo_msgs.srv import (GetLinkState, GetModelProperties, GetModelState,
                                 GetPhysicsProperties, GetWorldProperties, SetModelState, SpawnModel)
    from geometry_msgs.msg import Pose
    from mavros_msgs.msg import State
    from sensor_msgs.msg import Image
    from std_srvs.srv import Empty, Trigger
    from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply
    cv2.setNumThreads(2)
    world_name = ET.parse(package / 'worlds/uav_usv_coop/map1_circle.world').getroot().find('world').get('name')
    report = {'package': str(package), 'duration_requested': args.duration, 'passed': False}
    report['full_demo'] = args.full_demo
    processes, logs = [], []

    def start(command, filename):
        log = (output / filename).open('w')
        logs.append(log)
        proc = subprocess.Popen(command, env=dict(os.environ), stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    def wait_until(predicate, timeout=35.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.05)
        raise TimeoutError('condition did not become true within %.1f seconds' % timeout)

    def transform(pose):
        q = pose.orientation
        matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return matrix

    def link_modes():
        result = subprocess.run(['gz', 'model', '-w', world_name, '-m', 'wamv', '-i'],
                                env=dict(os.environ), capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise RuntimeError(result.stderr)
        modes = {}
        for block in re.findall(r'^link \{\n(.*?)^\}', result.stdout, re.M | re.S):
            name = re.search(r'^  name: (".*")$', block, re.M)
            mode = re.search(r'^  kinematic: (true|false)$', block, re.M)
            if name and mode:
                modes[json.loads(name.group(1))] = mode.group(1) == 'true'
        assert len(modes) == 13, modes
        return modes

    active_collection = False
    baseline = {}
    inertial_offsets = {}
    for link in ET.parse(package / 'models/vehicle/wamv_apriltag/model.sdf').getroot().findall('model/link'):
        values = [float(value) for value in link.findtext('inertial/pose', '0 0 0 0 0 0').split()]
        inertial_offsets['wamv::' + link.get('name')] = np.array(values[:3])
    sample_lock = threading.Lock()
    samples = []
    last_wall = 0.
    last_theta, phase_angle = None, 0.

    def collect(message):
        nonlocal last_wall, last_theta, phase_angle
        wall = time.monotonic()
        if not active_collection or wall - last_wall < .025:
            return
        last_wall = wall
        indices = {name: index for index, name in enumerate(message.name) if name.startswith('wamv::')}
        base_name = 'wamv::wamv/base_link'
        if base_name not in indices or any(name not in indices for name in baseline):
            return
        base_index = indices[base_name]
        base = transform(message.pose[base_index])
        inverse = np.linalg.inv(base)
        velocity = message.twist[base_index]
        v = np.array([velocity.linear.x, velocity.linear.y, velocity.linear.z])
        w = np.array([velocity.angular.x, velocity.angular.y, velocity.angular.z])
        x, y, z = base[:3, 3]
        theta = math.atan2(y - 180., x + 570.)
        if last_theta is None:
            phase_angle = theta
        else:
            phase_angle += math.atan2(math.sin(theta-last_theta), math.cos(theta-last_theta))
        last_theta = theta
        phase = phase_angle / .1
        roll, pitch = math.radians(2.) * math.sin(2.*math.pi*phase/6.), math.radians(1.) * math.sin(2.*math.pi*phase/8.)
        rd, pd = math.radians(2.) * 2.*math.pi/6. * math.cos(2.*math.pi*phase/6.), math.radians(1.) * 2.*math.pi/8. * math.cos(2.*math.pi*phase/8.)
        yaw = phase_angle + math.pi/2.
        analytic_v = np.array([-.5*math.sin(theta), .5*math.cos(theta), .03*2.*math.pi/7.*math.cos(2.*math.pi*phase/7.)])
        analytic_w = np.array([rd*math.cos(yaw)*math.cos(pitch)-pd*math.sin(yaw),
                               rd*math.sin(yaw)*math.cos(pitch)+pd*math.cos(yaw), .1-rd*math.sin(pitch)])
        expected_z = .1 + .03*math.sin(2.*math.pi*phase/7.)
        attitude = quaternion_matrix(quaternion_from_euler(roll, pitch, yaw))[:3, :3]
        attitude_angle = math.acos(float(np.clip((np.trace(base[:3, :3] @ attitude.T)-1.)/2., -1., 1.)))
        position_error, angle_error, linear_error, angular_error = 0., 0., 0., 0.
        cog_velocity_error = 0.
        for name, initial in baseline.items():
            index = indices[name]
            link = transform(message.pose[index])
            relative = inverse @ link
            position_error = max(position_error, float(np.linalg.norm(relative[:3, 3] - initial[:3, 3])))
            difference = relative[:3, :3] @ initial[:3, :3].T
            angle = math.acos(float(np.clip((np.trace(difference) - 1.) / 2., -1., 1.)))
            angle_error = max(angle_error, angle)
            twist = message.twist[index]
            vi = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            wi = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
            # LinkStates reports the LINK ORIGIN velocity, not the CoG velocity.
            expected = v + np.cross(w, link[:3, 3] - base[:3, 3])
            linear_error = max(linear_error, float(np.linalg.norm(vi - expected)))
            angular_error = max(angular_error, float(np.linalg.norm(wi - analytic_w)))
            offset = link[:3, :3] @ inertial_offsets[name]
            observed_cog = vi + np.cross(wi, offset)
            expected_cog = analytic_v + np.cross(analytic_w, link[:3, 3] + offset - base[:3, 3])
            cog_velocity_error = max(cog_velocity_error, float(np.linalg.norm(observed_cog - expected_cog)))
        radius_error = abs(math.hypot(x + 570., y - 180.) - 5.)
        tangent_error = float(np.linalg.norm(v[:2] - analytic_v[:2]))
        with sample_lock:
            if not active_collection:
                return
            samples.append(dict(stamp=rospy.Time.now().to_sec(), position_error=position_error,
                                angle_error=angle_error, linear_error=linear_error,
                                angular_error=angular_error, radius_error=radius_error,
                                tangent_velocity_error=tangent_error, cog_velocity_error=cog_velocity_error,
                                heave_error=abs(z-expected_z), attitude_error=attitude_angle, position=[x, y, z]))

    def camera_sdf(name, width, height):
        focal = 477.296554 * width / 640.
        hfov = 2.*math.atan(width/(2.*focal))
        return f'''<sdf version="1.6"><model name="circle_check_{name}"><static>true</static>
        <link name="link"><sensor name="camera" type="camera"><always_on>true</always_on><update_rate>10</update_rate>
        <camera><horizontal_fov>{hfov}</horizontal_fov><image><width>{width}</width><height>{height}</height>
        <format>R8G8B8</format></image><clip><near>0.1</near><far>100</far></clip></camera>
        <plugin name="camera" filename="libgazebo_ros_camera.so"><robotNamespace>/circle_check</robotNamespace>
        <cameraName>{name}</cameraName><imageTopicName>image_raw</imageTopicName><cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>circle_check_{name}</frameName><focalLength>{focal}</focalLength>
        <Cx>{width/2}</Cx><Cy>{height/2}</Cy><CxPrime>{width/2}</CxPrime>
        <distortionK1>0</distortionK1><distortionK2>0</distortionK2><distortionK3>0</distortionK3>
        <distortionT1>0</distortionT1><distortionT2>0</distortionT2></plugin></sensor></link></model></sdf>'''

    def snapshot(label):
        boat = get_model('wamv', 'world').pose
        matrix = transform(boat)
        q = [boat.orientation.x, boat.orientation.y, boat.orientation.z, boat.orientation.w]
        for name in ['oblique', 'tag']:
            if name == 'tag':
                position = (matrix @ np.array([.033, 0., 1.7611 + 3., 1.]))[:3]
                orientation = quaternion_multiply(q, quaternion_from_euler(0., math.pi/2., 0.))
            else:
                position = (matrix @ np.array([-5.5, -6., 5., 1.]))[:3]
                target = (matrix @ np.array([0., 0., .9, 1.]))[:3]
                direction = target - position
                orientation = quaternion_from_euler(0., math.atan2(-direction[2], np.linalg.norm(direction[:2])),
                                                    math.atan2(direction[1], direction[0]))
            msg = ModelState(model_name='circle_check_' + name, reference_frame='world')
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, orientation)
            assert set_model(msg).success
            topic = '/circle_check/' + name + '/image_raw'
            # Skip a frame rendered before moving the static validation camera.
            rospy.wait_for_message(topic, Image, timeout=20)
            raw = rospy.wait_for_message(topic, Image, timeout=20)
            pixels = CvBridge().imgmsg_to_cv2(raw, 'bgr8')
            assert cv2.imwrite(str(output / 'frames' / (label + '_' + name + '.png')), pixels)
            if name == 'tag':
                parameters = cv2.aruco.DetectorParameters_create()
                parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
                _, ids, _ = cv2.aruco.detectMarkers(pixels, cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11), parameters=parameters)
                report[label + '_tag_ids'] = None if ids is None else ids.tolist()
                assert ids is not None and 3 in ids, 'rendered AprilTag not detected'

    def interruption(signum, frame):
        raise KeyboardInterrupt()

    try:
        signal.signal(signal.SIGTERM, interruption)
        print('Starting isolated circle demo', flush=True)
        if args.full_demo:
            launch = package / 'launch/demo_uav_usv_circle.launch'
            launch_args = ['gui:=false', 'paused:=true']
        else:
            # The same ship, world, plugin and supervisor without PX4's external
            # lockstep clock, so ResetSimulation can be tested independently.
            launch = output / 'fixture.launch'
            launch.write_text(f'''<launch>
              <include file="{package}/launch/core/gazebo_world.launch">
                <arg name="world" value="{package}/worlds/uav_usv_coop/map1_circle.world"/>
                <arg name="gui" value="false"/><arg name="paused" value="true"/>
                <arg name="extra_gazebo_args" value="-s libwamv_circle_motion_plugin.so"/>
              </include>
              <include file="{package}/launch/core/wamv_spawn.launch">
                <arg name="usv_model" value="wamv_apriltag"/><arg name="usv_x" value="-565"/>
                <arg name="usv_y" value="180"/><arg name="usv_z" value="0.1"/>
                <arg name="usv_yaw" value="1.5707963267948966"/>
              </include>
              <node pkg="ocean_simulation_gazebo" type="wamv_circle_motion.py" name="circle_motion"
                    ns="wamv/simulation" output="screen"/>
            </launch>''')
            launch_args = []
        simulation = start(['roslaunch', str(launch), *launch_args], 'simulation.log')
        master = rosgraph.Master('/circle_assembly_regression')

        def clock_configured():
            try:
                return master.getParam('/use_sim_time') is True
            except Exception:
                return False

        # init_node reads /use_sim_time once. Initializing before roslaunch sets
        # it would silently measure wall time instead of simulation time.
        wait_until(clock_configured, 30.)
        rospy.init_node('circle_assembly_regression', anonymous=True, disable_signals=True)
        prefix = '/wamv/simulation/circle_motion'
        rospy.wait_for_service(prefix + '/keepalive', timeout=75)
        keepalive = rospy.ServiceProxy(prefix + '/keepalive', Trigger)
        wait_until(lambda: keepalive().success, 45.)
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        get_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        props = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        assert physics().pause
        modes = link_modes()
        assert all(modes.values()), modes
        bodies = props('wamv').body_names
        assert len(bodies) == 13
        for name in bodies:
            full = 'wamv::' + name
            result = get_link(full, 'wamv::wamv/base_link')
            assert result.success
            baseline[full] = transform(result.link_state.pose)
        print('Paused startup: 13 links acquired; heartbeat stays alive', flush=True)
        time.sleep(2.3)
        assert keepalive().success and physics().pause
        for name, width, height in [('oblique', 1280, 720), ('tag', 640, 360)]:
            p = Pose()
            p.orientation.w = 1.
            response = spawn('circle_check_' + name, camera_sdf(name, width, height), '', p, 'world')
            assert response.success, response.status_message
        sub = rospy.Subscriber('/gazebo/link_states', LinkStates, collect, queue_size=1)
        unpause()
        begin = get_world().sim_time
        # WorldUpdateBegin publishes LinkStates before BeforePhysicsUpdate on
        # the first tick; skip that still-paused, deliberately zero-velocity state.
        wait_until(lambda: get_world().sim_time - begin >= .05, 20.)
        active_collection = True
        wait_until(lambda: get_world().sim_time - begin >= .5, 20.)
        if args.full_demo:
            wait_until(lambda: rospy.wait_for_message('/mavros/state', State, timeout=5).connected, 25.)
            report['mavros_connected'] = True
        snapshot('early')
        deadline = time.monotonic() + max(100., args.duration * 4.)
        next_notice = 20.
        while get_world().sim_time - begin < args.duration:
            assert simulation.poll() is None, 'Gazebo launch exited'
            assert keepalive().success, 'native motion stopped'
            if time.monotonic() > deadline:
                raise TimeoutError('circle regression exceeded wall-clock deadline')
            with sample_lock:
                if samples and samples[-1]['position_error'] > .001:
                    raise AssertionError('assembly position drift: ' + str(samples[-1]))
            elapsed = get_world().sim_time - begin
            if elapsed >= next_notice:
                print('Assembly sampling:', round(elapsed, 2), 'simulation seconds', flush=True)
                next_notice += 20.
            time.sleep(.05)
        snapshot('two_laps' if args.duration >= 4.*math.pi*5./.5 else 'end')
        pause()
        active_collection = False
        with sample_lock:
            measured = list(samples)
        assert len(measured) >= 20, len(measured)
        fields = ['position_error', 'angle_error', 'linear_error', 'angular_error', 'radius_error',
                  'tangent_velocity_error', 'cog_velocity_error', 'heave_error', 'attitude_error']
        report['sample_count'] = len(measured)
        report['maxima'] = {key: max(s[key] for s in measured) for key in fields}
        report['simulation_duration'] = get_world().sim_time - begin
        assert report['maxima']['position_error'] <= .001, report['maxima']
        assert report['maxima']['angle_error'] <= math.radians(.1), report['maxima']
        assert report['maxima']['radius_error'] <= .01, report['maxima']
        assert report['maxima']['linear_error'] <= .001, report['maxima']
        assert report['maxima']['angular_error'] <= .001, report['maxima']
        assert report['maxima']['tangent_velocity_error'] <= .01, report['maxima']
        assert report['maxima']['cog_velocity_error'] <= .001, report['maxima']
        assert report['maxima']['heave_error'] <= .001, report['maxima']
        assert report['maxima']['attitude_error'] <= math.radians(.1), report['maxima']
        print('Assembly/velocity maxima:', report['maxima'], flush=True)
        before = get_model('wamv', 'world').pose
        # /clock callbacks can still contain one queued pre-pause message.
        # Query the simulator's actual clock for a strict paused-state check.
        stamp = get_world().sim_time
        time.sleep(.4)
        assert np.allclose(transform(before), transform(get_model('wamv', 'world').pose), atol=1e-10)
        assert get_world().sim_time == stamp and physics().pause
        report['pause_stable'] = True
        if not args.full_demo:
            # Rewind with an established trajectory phase, not a fresh start.
            reset()
            assert physics().pause and get_world().sim_time < .001
            unpause()
            wait_until(lambda: get_world().sim_time >= 4., 25.)
            pause()
            assert keepalive().success and all(link_modes().values())
            reset_state = get_model('wamv', 'world')
            assert reset_state.success, reset_state.status_message
            t = get_world().sim_time - .001
            expected = np.array([-570.+5.*math.cos(.1*t), 180.+5.*math.sin(.1*t),
                                 .1+.03*math.sin(2.*math.pi*t/7.)])
            report['reset_phase_position_error'] = float(np.linalg.norm(transform(reset_state.pose)[:3, 3]-expected))
            assert report['reset_phase_position_error'] <= .001, report['reset_phase_position_error']
            reset_errors = []
            for name, initial in baseline.items():
                link = get_link(name, 'wamv::wamv/base_link')
                assert link.success, link.status_message
                reset_errors.append(float(np.linalg.norm(transform(link.link_state.pose)[:3, 3]-initial[:3, 3])))
            report['reset_assembly_error'] = max(reset_errors)
            assert report['reset_assembly_error'] <= .001
            report['reset_restarted'] = True
        rosnode.kill_nodes(['/wamv/simulation/circle_motion'])
        wait_until(lambda: not keepalive().success, 8.)
        assert not any(link_modes().values()) and physics().pause
        report['normal_stop_restored'] = True
        print('Normal stop restored modes without unpausing', flush=True)
        supervisor = start(['rosrun', 'ocean_simulation_gazebo', 'wamv_circle_motion.py',
                            '__ns:=/wamv/simulation', '__name:=circle_motion'], 'restarted_supervisor.log')
        wait_until(lambda: keepalive().success, 10.)
        resumed_at = get_world().sim_time
        unpause()
        wait_until(lambda: get_world().sim_time >= resumed_at + 4., 25.)
        pause()
        report['resumed_link_modes'] = link_modes()
        report['resumed_active'] = keepalive().success
        assert all(report['resumed_link_modes'].values()), report['resumed_link_modes']
        model_state = get_model('wamv', 'world')
        report['resumed_model_success'] = model_state.success
        report['resumed_model_message'] = model_state.status_message
        assert model_state.success, model_state.status_message
        base_state = get_link('wamv::wamv/base_link', 'world')
        assert base_state.success, base_state.status_message
        report['resumed_base_position'] = [base_state.link_state.pose.position.x,
                                           base_state.link_state.pose.position.y,
                                           base_state.link_state.pose.position.z]
        position = model_state.pose.position
        report['resumed_position'] = [position.x, position.y, position.z]
        assert abs(math.hypot(position.x+570., position.y-180.)-5.) <= .01, report['resumed_position']
        report['supervisor_restarted'] = True
        # Simulate abrupt supervisor loss while paused: only the owned process is killed.
        supervisor.kill()
        supervisor.wait(timeout=5)
        # Polling keepalive here would itself keep the controller alive.
        time.sleep(2.5)
        assert not keepalive().success
        assert not any(link_modes().values()) and physics().pause
        report['heartbeat_loss_restored'] = True
        for name in baseline:
            velocity = get_link(name, 'world').link_state.twist
            assert sum(v*v for v in [velocity.linear.x, velocity.linear.y, velocity.linear.z,
                                    velocity.angular.x, velocity.angular.y, velocity.angular.z]) < 1e-12
        report['stopped_velocities_zero'] = True
        rospy.set_param(prefix + '/circle_radius', 0.)
        failed = rospy.ServiceProxy(prefix + '/start', Trigger)()
        assert not failed.success and not any(link_modes().values())
        rospy.set_param(prefix + '/circle_radius', 5.)
        report['invalid_start_restored'] = True
        report['passed'] = True
        print('Pause, normal stop, heartbeat loss and invalid startup passed' +
              ('; simulation reset also passed' if not args.full_demo else ''), flush=True)
    except BaseException as error:
        report['error'] = repr(error)
        report['traceback'] = traceback.format_exc()
        traceback.print_exc()
    finally:
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        with sample_lock:
            (output / 'samples.json').write_text(json.dumps(samples) + '\n')
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in logs:
            log.close()
        print('Owned processes stopped; report:', output / 'report.json', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
