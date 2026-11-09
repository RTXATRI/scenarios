#!/usr/bin/env python3
# 文件作用：监督 WAM-V 原生绕圈运动插件，负责参数校验、启动接管、墙钟心跳和正常停止。
# 实际轨迹由 C++ 插件按物理步执行；本脚本通过内部服务管理插件生命周期。
"""Supervise the physics-step WAM-V plugin without driving model poses by RPC."""

import math
import queue
import threading
import time
import sys

DEFAULTS = dict(circle_center_x=-570., circle_center_y=180., circle_radius=5.,
                circle_speed=.5, clockwise=False, usv_z=.1,
                roll_amplitude=math.radians(2.), pitch_amplitude=math.radians(1.),
                heave_amplitude=.03, roll_period=6., pitch_period=8.,
                heave_period=7., motion_rate=50.)
READY_TIMEOUT = 30.
SERVICE_TIMEOUT = 1.
SERVICE_PREFIX = '/wamv/simulation/circle_motion'


def motion_parameters(**overrides):
    values = dict(DEFAULTS, **overrides)
    if set(values) != set(DEFAULTS):
        raise ValueError('Unknown circle motion parameter')
    if not isinstance(values['clockwise'], bool):
        raise ValueError('clockwise must be a boolean')
    for name in DEFAULTS:
        if name == 'clockwise':
            continue
        if isinstance(values[name], bool):
            raise ValueError(name + ' must be a finite number')
        values[name] = float(values[name])
        if not math.isfinite(values[name]):
            raise ValueError(name + ' must be finite')
    for name in ('circle_radius', 'roll_period', 'pitch_period', 'heave_period', 'motion_rate'):
        if values[name] <= 0.:
            raise ValueError(name + ' must be positive')
    for name in ('circle_speed', 'roll_amplitude', 'pitch_amplitude', 'heave_amplitude'):
        if values[name] < 0.:
            raise ValueError(name + ' must be nonnegative')
    return values


def wall_call(function, timeout):
    """A blocked ROS RPC cannot keep startup or shutdown waiting indefinitely."""
    results = queue.Queue(maxsize=1)

    def worker():
        try:
            results.put((function(), None))
        except BaseException as error:
            results.put((None, error))

    threading.Thread(target=worker, daemon=True).start()
    try:
        result, error = results.get(timeout=timeout)
    except queue.Empty:
        raise TimeoutError('Gazebo service call exceeded %.3g real seconds' % timeout)
    if error is not None:
        raise error
    return result


class WamvCircleMotion:
    def __init__(self, rospy, parameters):
        from std_srvs.srv import Trigger
        self.rospy, self.parameters = rospy, parameters
        self.start_service = rospy.ServiceProxy(SERVICE_PREFIX + '/start', Trigger)
        self.keepalive_service = rospy.ServiceProxy(SERVICE_PREFIX + '/keepalive', Trigger, persistent=True)
        self.stop_service = rospy.ServiceProxy(SERVICE_PREFIX + '/stop', Trigger)
        self.start_attempted = False
        self.finished = threading.Event()
        self.stop_lock = threading.Lock()
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        # Client shutdown hooks run before rospy closes its service transports.
        self.finished.set()
        self.stop()

    def wait_ready(self):
        deadline = time.monotonic() + READY_TIMEOUT
        for suffix in ('/start', '/keepalive', '/stop'):
            remaining = deadline - time.monotonic()
            if remaining <= 0.:
                raise TimeoutError('WAM-V motion plugin was not ready within 30 real seconds')
            self.rospy.wait_for_service(SERVICE_PREFIX + suffix, timeout=remaining)
        while not self.rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0.:
                raise TimeoutError('WAM-V model was not ready within 30 real seconds')
            self.start_attempted = True
            # Loading a model/reading its parameters can outlast a normal RPC
            # during Gazebo startup. Acquisition shares the 30 s wall deadline.
            reply = wall_call(self.start_service, remaining)
            if reply.success:
                self.rospy.loginfo('%s', reply.message)
                return
            if not reply.message.startswith('waiting for '):
                raise RuntimeError('WAM-V motion plugin could not start: ' + reply.message)
            self.finished.wait(min(.05, remaining))
        raise self.rospy.ROSInterruptException('shutdown during plugin startup')

    def run(self):
        self.wait_ready()
        self.rospy.loginfo('WAM-V physics-step circle active; supervisor heartbeat rate=%g Hz',
                           self.parameters['motion_rate'])
        while not self.rospy.is_shutdown() and not self.finished.is_set():
            reply = wall_call(self.keepalive_service, SERVICE_TIMEOUT)
            if not reply.success:
                raise RuntimeError('WAM-V motion plugin stopped: ' + reply.message)
            # A wall-clock wait keeps the watchdog alive even while Gazebo is paused.
            self.finished.wait(1. / self.parameters['motion_rate'])

    def stop(self):
        self.finished.set()
        with self.stop_lock:
            if not self.start_attempted:
                return True
            try:
                reply = wall_call(self.stop_service, SERVICE_TIMEOUT)
                if not reply.success:
                    raise RuntimeError(reply.message)
                self.start_attempted = False
                self.rospy.loginfo('%s', reply.message)
                return True
            except Exception as error:
                self.rospy.logwarn('WAM-V link dynamics restoration could not be confirmed: %s', error)
                return False


def main():
    import rospy
    rospy.init_node('circle_motion')
    node = None
    exit_code = 0
    try:
        parameters = motion_parameters(**{name: rospy.get_param('~'+name, default)
                                          for name, default in DEFAULTS.items()})
        node = WamvCircleMotion(rospy, parameters)
        node.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as error:
        rospy.logerr('WAM-V circle motion FAILED: %s', error)
        exit_code = 1
    finally:
        if node is not None and not node.stop():
            exit_code = 1
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
