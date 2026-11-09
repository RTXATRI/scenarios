#!/usr/bin/env python3
# 文件作用：为完整导航实验一次性设置并回读 Gazebo 物理更新率上限，从而配置目标仿真倍率。
# 保留物理步长、重力、ODE 配置及暂停状态，完成后退出。
"""Apply one experiment's Gazebo speed limit, then exit without unpausing."""

import math
import queue
import sys
import threading


CONFIGURATION_TIMEOUT = 30.


def validate_speed(value):
    if isinstance(value, bool):
        raise ValueError('sim_speed must be a finite nonnegative number')
    speed = float(value)
    if not math.isfinite(speed) or speed < 0:
        raise ValueError('sim_speed must be finite and >= 0 (0 means unlimited)')
    return speed


def target_update_rate(speed, time_step):
    speed = validate_speed(speed)
    if not math.isfinite(time_step) or time_step <= 0:
        raise ValueError('Gazebo time_step must be finite and positive')
    rate = speed / time_step if speed else 0.
    if not math.isfinite(rate) or (speed > 0 and rate <= 0):
        raise ValueError('sim_speed / time_step is outside the finite positive range')
    return rate


def matching_rate(actual, expected):
    return (math.isfinite(actual) and actual >= 0 and
            math.isclose(actual, expected, rel_tol=1e-9, abs_tol=0.))


def configure_speed(speed, get_physics, set_physics):
    """Only the update rate may differ after the service round trip."""
    speed = validate_speed(speed)
    before = get_physics()
    if not before.success:
        raise RuntimeError('GetPhysicsProperties failed: ' + before.status_message)
    rate = target_update_rate(speed, before.time_step)
    changed = not matching_rate(before.max_update_rate, rate)
    if changed:
        response = set_physics(before.time_step, rate, before.gravity, before.ode_config)
        if not response.success:
            raise RuntimeError('SetPhysicsProperties failed: ' + response.status_message)
    after = get_physics()
    if not after.success:
        raise RuntimeError('Physics readback failed: ' + after.status_message)
    if not matching_rate(after.max_update_rate, rate):
        raise RuntimeError('Physics readback rate mismatch: requested=%r actual=%r' %
                           (rate, after.max_update_rate))
    for field in ('time_step', 'gravity', 'ode_config', 'pause'):
        if getattr(after, field) != getattr(before, field):
            raise RuntimeError('Physics readback changed preserved field: ' + field)
    return dict(sim_speed=speed, time_step=after.time_step, target_update_rate=rate,
                actual_update_rate=after.max_update_rate, paused=after.pause, changed=changed)


def with_wall_timeout(function, timeout=CONFIGURATION_TIMEOUT):
    """Bound ROS waits and RPCs even when /clock is paused or a server hangs.

    The daemon worker cannot keep this one-shot process alive after a timeout.
    A timed-out write may already have reached Gazebo; never report it as applied.
    """
    results = queue.Queue(maxsize=1)

    def work():
        try:
            results.put((function(), None))
        except BaseException as error:
            results.put((None, error))

    threading.Thread(target=work, daemon=True).start()
    try:
        result, error = results.get(timeout=timeout)
    except queue.Empty:
        raise TimeoutError('Simulation speed configuration exceeded %.1f real seconds; '
                           'Gazebo state is unconfirmed' % timeout)
    if error is not None:
        raise error
    return result


def main():
    import rospy
    from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties

    rospy.init_node('set_simulation_speed')

    def apply():
        speed = validate_speed(rospy.get_param('~sim_speed', 1.))
        get_name = '/gazebo/get_physics_properties'
        set_name = '/gazebo/set_physics_properties'
        rospy.wait_for_service(get_name, timeout=CONFIGURATION_TIMEOUT)
        rospy.wait_for_service(set_name, timeout=CONFIGURATION_TIMEOUT)
        return configure_speed(speed, rospy.ServiceProxy(get_name, GetPhysicsProperties),
                               rospy.ServiceProxy(set_name, SetPhysicsProperties))

    try:
        result = with_wall_timeout(apply)
        rospy.loginfo('Simulation speed configured: requested=%g%s step=%.9g '
                      'target_update_rate=%.9g actual_update_rate=%.9g paused=%s changed=%s',
                      result['sim_speed'], ' (unlimited)' if result['sim_speed'] == 0 else 'x',
                      result['time_step'], result['target_update_rate'],
                      result['actual_update_rate'], result['paused'], result['changed'])
        return 0
    except Exception as error:
        rospy.logfatal('Simulation speed configuration FAILED: %s', error)
        return 1


if __name__ == '__main__':
    sys.exit(main())
