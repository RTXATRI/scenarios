/**
 * 文件作用：定义无人船圆周与摇晃运动参数、校验规则和解析轨迹采样。
 * 输出模型位姿、世界线速度/角速度，并计算各部件质心速度，供原生插件与数学检查共用。
 */

#ifndef OCEAN_SIMULATION_GAZEBO_WAMV_CIRCLE_MOTION_HPP
#define OCEAN_SIMULATION_GAZEBO_WAMV_CIRCLE_MOTION_HPP

#include <cmath>
#include <stdexcept>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace ocean_simulation_gazebo
{
namespace circle_motion
{
constexpr double kPi = 3.14159265358979323846;

struct Parameters
{
  double center_x = -570.;
  double center_y = 180.;
  double radius = 5.;
  double speed = .5;
  bool clockwise = false;
  double z = .1;
  double roll_amplitude = 2. * kPi / 180.;
  double pitch_amplitude = kPi / 180.;
  double heave_amplitude = .03;
  double roll_period = 6.;
  double pitch_period = 8.;
  double heave_period = 7.;
  double motion_rate = 50.;

  void Validate() const
  {
    for (double value : {center_x, center_y, radius, speed, z, roll_amplitude,
                         pitch_amplitude, heave_amplitude, roll_period,
                         pitch_period, heave_period, motion_rate})
      if (!std::isfinite(value))
        throw std::invalid_argument("Circle motion parameters must be finite");
    if (radius <= 0. || roll_period <= 0. || pitch_period <= 0. ||
        heave_period <= 0. || motion_rate <= 0.)
      throw std::invalid_argument("Radius, periods and motion_rate must be positive");
    if (speed < 0. || roll_amplitude < 0. || pitch_amplitude < 0. || heave_amplitude < 0.)
      throw std::invalid_argument("Speed and amplitudes must be nonnegative");
  }
};

struct Sample
{
  ignition::math::Pose3d pose;
  ignition::math::Vector3d linear;
  ignition::math::Vector3d angular;
};

inline Sample SampleMotion(const Parameters &p, double elapsed)
{
  if (!std::isfinite(elapsed) || elapsed < 0.)
    throw std::invalid_argument("Elapsed simulation time must be finite and nonnegative");
  const double direction = p.clockwise ? -1. : 1.;
  const double omega = direction * p.speed / p.radius;
  const double theta = omega * elapsed;
  const double yaw = theta + direction * kPi / 2.;
  const double fr = 2. * kPi / p.roll_period;
  const double fp = 2. * kPi / p.pitch_period;
  const double fh = 2. * kPi / p.heave_period;
  const double roll = p.roll_amplitude * std::sin(fr * elapsed);
  const double pitch = p.pitch_amplitude * std::sin(fp * elapsed);
  const double roll_dot = p.roll_amplitude * fr * std::cos(fr * elapsed);
  const double pitch_dot = p.pitch_amplitude * fp * std::cos(fp * elapsed);
  Sample result;
  result.pose = ignition::math::Pose3d(
      p.center_x + p.radius * std::cos(theta),
      p.center_y + p.radius * std::sin(theta),
      p.z + p.heave_amplitude * std::sin(fh * elapsed), roll, pitch, yaw);
  result.linear.Set(-p.radius * omega * std::sin(theta),
                    p.radius * omega * std::cos(theta),
                    p.heave_amplitude * fh * std::cos(fh * elapsed));
  // World angular velocity of Rz(yaw) Ry(pitch) Rx(roll).
  result.angular.Set(roll_dot * std::cos(yaw) * std::cos(pitch) - pitch_dot * std::sin(yaw),
                     roll_dot * std::sin(yaw) * std::cos(pitch) + pitch_dot * std::cos(yaw),
                     omega - roll_dot * std::sin(pitch));
  if (!result.pose.IsFinite() || !result.linear.IsFinite() || !result.angular.IsFinite())
    throw std::invalid_argument("Derived circle motion values must remain finite");
  return result;
}

inline ignition::math::Vector3d CenterOfMassVelocity(
    const Sample &state, const ignition::math::Vector3d &center_of_mass)
{
  return state.linear + state.angular.Cross(center_of_mass - state.pose.Pos());
}
}  // namespace circle_motion
}  // namespace ocean_simulation_gazebo
#endif
