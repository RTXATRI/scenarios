#include <gtest/gtest.h>
#include "../src/wamv_circle_motion.hpp"
#include <limits>

namespace cm = ocean_simulation_gazebo::circle_motion;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;

TEST(CircleMotion, ClosureSpeedAndTangentInBothDirections)
{
  for (bool clockwise : {false, true})
  {
    cm::Parameters p;
    p.clockwise = clockwise;
    p.Validate();
    const double period = 2. * cm::kPi * p.radius / p.speed;
    const auto start = cm::SampleMotion(p, 0.);
    const auto quarter = cm::SampleMotion(p, period / 4.);
    const auto end = cm::SampleMotion(p, period);
    EXPECT_NEAR(quarter.pose.Pos().X(), -570., 1e-10);
    EXPECT_NEAR(quarter.pose.Pos().Y(), clockwise ? 175. : 185., 1e-10);
    // Heave has its own period; circle closure applies to horizontal position.
    EXPECT_NEAR(std::hypot(start.pose.Pos().X()-end.pose.Pos().X(),
                           start.pose.Pos().Y()-end.pose.Pos().Y()), 0., 1e-10);
    for (double t : {0., 1.23, period / 4., period})
    {
      const auto state = cm::SampleMotion(p, t);
      EXPECT_NEAR(std::hypot(state.linear.X(), state.linear.Y()), .5, 1e-12);
      const double bearing = std::atan2(state.linear.Y(), state.linear.X());
      EXPECT_NEAR(std::sin(state.pose.Rot().Yaw() - bearing), 0., 1e-12);
    }
  }
}

TEST(CircleMotion, WorldVelocityMatchesPoseDerivative)
{
  cm::Parameters p;
  constexpr double dt = 1e-5;
  for (double t : {1.23, 3., 7.})
  {
    const auto left = cm::SampleMotion(p, t - dt);
    const auto mid = cm::SampleMotion(p, t);
    const auto right = cm::SampleMotion(p, t + dt);
    EXPECT_LT(((right.pose.Pos() - left.pose.Pos()) / (2. * dt) - mid.linear).Length(), 1e-7);
    const auto &a = left.pose.Rot();
    const auto &b = right.pose.Rot();
    const Quaterniond qdot((b.W()-a.W())/(2.*dt), (b.X()-a.X())/(2.*dt),
                           (b.Y()-a.Y())/(2.*dt), (b.Z()-a.Z())/(2.*dt));
    const auto omega = qdot * mid.pose.Rot().Inverse();
    EXPECT_LT((Vector3d(omega.X(), omega.Y(), omega.Z()) * 2. - mid.angular).Length(), 1e-8);
  }
}

TEST(CircleMotion, OffsetCenterOfMassMatchesIndependentFiniteDifference)
{
  cm::Parameters p;
  constexpr double dt = 1e-5;
  // Include mount rotation and an inertial offset; link origin is not its CoG.
  for (const auto &assembly : {Pose3d(-2.37, 1.027, .318, 0., 0., .785),
                               Pose3d(1.6, -.7, .25, 0., 0., .785)})
  {
    const Pose3d inertial(-.16, .02, -.24, .1, 0., 0.);
    for (double t : {.4, 2., 5.})
    {
      const auto left = cm::SampleMotion(p, t - dt);
      const auto mid = cm::SampleMotion(p, t);
      const auto right = cm::SampleMotion(p, t + dt);
      const auto a = (left.pose * assembly * inertial).Pos();
      const auto b = (right.pose * assembly * inertial).Pos();
      const auto cog = (mid.pose * assembly * inertial).Pos();
      EXPECT_LT(((b-a)/(2.*dt) - cm::CenterOfMassVelocity(mid, cog)).Length(), 1e-7);
    }
  }
}

TEST(CircleMotion, RotationRequiresDifferentVelocitiesAtDifferentPoints)
{
  cm::Sample sample;
  sample.pose = Pose3d::Zero;
  sample.linear = Vector3d(1., 2., 3.);
  sample.angular = Vector3d(0., 0., 2.);
  const auto v = cm::CenterOfMassVelocity(sample, Vector3d(4., 5., 0.));
  EXPECT_DOUBLE_EQ(v.X(), -9.);
  EXPECT_DOUBLE_EQ(v.Y(), 10.);
  EXPECT_DOUBLE_EQ(v.Z(), 3.);
}

TEST(CircleMotion, ZeroSpeedAndDisabledRocking)
{
  cm::Parameters p;
  p.speed = 0.;
  const auto start = cm::SampleMotion(p, 0.);
  const auto roll = cm::SampleMotion(p, p.roll_period/4.);
  EXPECT_NEAR(roll.pose.Rot().Roll(), p.roll_amplitude, 1e-12);
  EXPECT_DOUBLE_EQ(start.pose.Pos().X(), roll.pose.Pos().X());
  EXPECT_DOUBLE_EQ(start.pose.Pos().Y(), roll.pose.Pos().Y());
  const auto pitch = cm::SampleMotion(p, p.pitch_period/4.);
  EXPECT_NEAR(pitch.pose.Rot().Pitch(), p.pitch_amplitude, 1e-12);
  EXPECT_NEAR(cm::SampleMotion(p, p.heave_period/4.).pose.Pos().Z(), p.z+p.heave_amplitude, 1e-12);
  p.roll_amplitude = p.pitch_amplitude = p.heave_amplitude = 0.;
  const auto fixed = cm::SampleMotion(p, 1.23);
  EXPECT_DOUBLE_EQ(fixed.linear.Length(), 0.);
  EXPECT_DOUBLE_EQ(fixed.angular.Length(), 0.);
}

TEST(CircleMotion, InvalidInputAndDerivedOverflow)
{
  cm::Parameters p;
  p.radius = 0.;
  EXPECT_THROW(p.Validate(), std::invalid_argument);
  p = cm::Parameters();
  p.speed = -1.;
  EXPECT_THROW(p.Validate(), std::invalid_argument);
  p = cm::Parameters();
  p.center_x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(p.Validate(), std::invalid_argument);
  p = cm::Parameters();
  EXPECT_THROW(cm::SampleMotion(p, -1.), std::invalid_argument);
  EXPECT_THROW(cm::SampleMotion(p, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  p.roll_period = 1e-310;
  EXPECT_THROW(cm::SampleMotion(p, 0.), std::invalid_argument);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
