/**
 * 文件作用：实现绕圈场景专用的 Gazebo 系统插件，在每个物理步统一更新整船装配位姿和各 link 速度。
 * 通过启动、心跳、停止服务管理临时运动学接管，停止或监督失联后恢复原 link 模式。
 */

#include "wamv_circle_motion.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <boost/thread/recursive_mutex.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <XmlRpcValue.h>

namespace ocean_simulation_gazebo
{
namespace cm = circle_motion;

// Loaded only by the circle demo through gzserver -s. Shared model SDFs stay intact.
class WamvCircleMotionPlugin : public gazebo::SystemPlugin
{
public:
  void Load(int, char **) override
  {
    world_created_ = gazebo::event::Events::ConnectWorldCreated(
        std::bind(&WamvCircleMotionPlugin::OnWorldCreated, this, std::placeholders::_1));
  }

  void Init() override
  {
    if (!ros::isInitialized())
      throw std::runtime_error("wamv_circle_motion_plugin requires gazebo_ros");
    node_ = std::make_unique<ros::NodeHandle>("/wamv/simulation/circle_motion");
    node_->setCallbackQueue(&queue_);
    start_service_ = node_->advertiseService("start", &WamvCircleMotionPlugin::Start, this);
    keepalive_service_ = node_->advertiseService("keepalive", &WamvCircleMotionPlugin::Keepalive, this);
    stop_service_ = node_->advertiseService("stop", &WamvCircleMotionPlugin::Stop, this);
    watchdog_ = node_->createWallTimer(ros::WallDuration(.05), &WamvCircleMotionPlugin::Watchdog, this);
    update_ = gazebo::event::Events::ConnectBeforePhysicsUpdate(
        std::bind(&WamvCircleMotionPlugin::OnUpdate, this, std::placeholders::_1));
    spinner_ = std::make_unique<ros::AsyncSpinner>(1, &queue_);
    spinner_->start();
    ROS_INFO("WAM-V physics-step motion plugin ready");
  }

  ~WamvCircleMotionPlugin() override
  {
    start_service_.shutdown();
    keepalive_service_.shutdown();
    stop_service_.shutdown();
    watchdog_.stop();
    if (spinner_)
      spinner_->stop();
    update_.reset();
    world_created_.reset();
    try
    {
      WithWorld([this](const gazebo::physics::WorldPtr &) { RestoreLocked("plugin unloaded"); });
    }
    catch (const std::exception &error)
    {
      gzerr << "WAM-V motion cleanup: " << error.what() << '\n';
    }
  }

private:
  struct LinkState
  {
    gazebo::physics::LinkPtr link;
    ignition::math::Pose3d relative_pose;
    bool kinematic;
  };

  void OnWorldCreated(const std::string &name)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!world_)
      world_ = gazebo::physics::get_world(name);
  }

  // Lock order is always physics -> controller. Taking a world snapshot first
  // avoids holding the controller lock while waiting for the physics thread.
  template<typename Function>
  bool WithWorld(Function function)
  {
    gazebo::physics::WorldPtr world;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      world = world_;
    }
    if (!world || !world->Physics())
      return false;
    boost::recursive_mutex::scoped_lock physics_guard(*world->Physics()->GetPhysicsUpdateMutex());
    std::lock_guard<std::mutex> guard(mutex_);
    function(world);
    return true;
  }

  double ReadNumber(const std::string &name, double fallback)
  {
    XmlRpc::XmlRpcValue value;
    if (!node_->getParam(name, value))
      return fallback;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      return static_cast<int>(value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      return static_cast<double>(value);
    throw std::invalid_argument(name + " must be a number, not a boolean or string");
  }

  cm::Parameters ReadParameters()
  {
    cm::Parameters p;
    p.center_x = ReadNumber("circle_center_x", p.center_x);
    p.center_y = ReadNumber("circle_center_y", p.center_y);
    p.radius = ReadNumber("circle_radius", p.radius);
    p.speed = ReadNumber("circle_speed", p.speed);
    p.z = ReadNumber("usv_z", p.z);
    p.roll_amplitude = ReadNumber("roll_amplitude", p.roll_amplitude);
    p.pitch_amplitude = ReadNumber("pitch_amplitude", p.pitch_amplitude);
    p.heave_amplitude = ReadNumber("heave_amplitude", p.heave_amplitude);
    p.roll_period = ReadNumber("roll_period", p.roll_period);
    p.pitch_period = ReadNumber("pitch_period", p.pitch_period);
    p.heave_period = ReadNumber("heave_period", p.heave_period);
    p.motion_rate = ReadNumber("motion_rate", p.motion_rate);
    XmlRpc::XmlRpcValue clockwise;
    if (node_->getParam("clockwise", clockwise))
    {
      if (clockwise.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        throw std::invalid_argument("clockwise must be a boolean");
      p.clockwise = static_cast<bool>(clockwise);
    }
    p.Validate();
    cm::SampleMotion(p, 0.);
    return p;
  }

  bool Start(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response)
  {
    try
    {
      // Parameter-server calls must not block physics while holding its lock.
      const auto parameters = ReadParameters();
      if (!WithWorld([&](const gazebo::physics::WorldPtr &world)
      {
        if (active_)
          throw std::runtime_error("WAM-V circle motion is already active");
        auto model = world->ModelByName("wamv");
        if (!model || model->GetLinks().empty())
        {
          response.message = "waiting for model wamv";
          return;
        }
        // Factory insertion exposes the model before all ODE bodies are Init'd.
        // Do not take ownership of a partially constructed assembly.
        for (const auto &link : model->GetLinks())
        {
          const auto ode = boost::dynamic_pointer_cast<gazebo::physics::ODELink>(link);
          if (ode && !ode->GetODEId())
          {
            response.message = "waiting for WAM-V link initialization";
            return;
          }
        }
        model_ = model;
        links_.clear();
        const auto inverse = model_->WorldPose().Inverse();
        for (const auto &link : model_->GetLinks())
          links_.push_back({link, inverse * link->WorldPose(), link->GetKinematic()});
        try
        {
          for (const auto &state : links_)
          {
            // Gazebo 11 ODE's SetKinematic(true) is not idempotent: invoking
            // it on an already kinematic body can switch it back to dynamic.
            if (!state.link->GetKinematic())
              state.link->SetKinematic(true);
            if (!state.link->GetKinematic())
              throw std::runtime_error("Cannot acquire kinematic link " + state.link->GetName());
            state.link->SetLinearVel(ignition::math::Vector3d::Zero);
            state.link->SetAngularVel(ignition::math::Vector3d::Zero);
          }
          parameters_ = parameters;
          heartbeat_timeout_ = std::max(2., 3. / parameters.motion_rate);
          last_heartbeat_ = ros::WallTime::now();
          epoch_ = last_sim_time_ = std::numeric_limits<double>::quiet_NaN();
          active_ = true;
          last_error_.clear();
          response.success = true;
          response.message = "WAM-V assembly acquired: " + std::to_string(links_.size()) + " links";
          ROS_INFO_STREAM(response.message);
        }
        catch (...)
        {
          RestoreLocked("startup failed");
          throw;
        }
      }))
        response.message = "waiting for Gazebo world";
    }
    catch (const std::exception &error)
    {
      response.success = false;
      response.message = error.what();
      ROS_ERROR_STREAM("WAM-V motion acquisition failed: " << error.what());
    }
    return true;
  }

  bool Keepalive(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    response.success = active_;
    response.message = active_ ? "WAM-V motion active" : last_error_;
    if (active_)
      last_heartbeat_ = ros::WallTime::now();
    return true;
  }

  bool Stop(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response)
  {
    try
    {
      WithWorld([this](const gazebo::physics::WorldPtr &) { RestoreLocked("stopped by supervisor"); });
      response.success = true;
      response.message = "WAM-V velocities zeroed and original link modes restored";
    }
    catch (const std::exception &error)
    {
      response.success = false;
      response.message = error.what();
    }
    return true;
  }

  void RestoreLocked(const std::string &reason)
  {
    active_ = false;
    last_error_ = reason;
    // A deleted/replaced model must never be modified through stale link handles.
    if (model_ && world_ && world_->ModelByName("wamv") == model_)
    {
      for (const auto &state : links_)
      {
        state.link->SetLinearVel(ignition::math::Vector3d::Zero);
        state.link->SetAngularVel(ignition::math::Vector3d::Zero);
        state.link->SetForce(ignition::math::Vector3d::Zero);
        state.link->SetTorque(ignition::math::Vector3d::Zero);
        if (state.link->GetKinematic() != state.kinematic)
          state.link->SetKinematic(state.kinematic);
      }
      ROS_INFO_STREAM("WAM-V original link modes restored: " << reason);
    }
    links_.clear();
    model_.reset();
  }

  void Watchdog(const ros::WallTimerEvent &)
  {
    // Healthy heartbeats must not wait for physics/model-loading locks: this
    // timer shares the supervisor service queue, including during paused load.
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!active_ || (ros::WallTime::now() - last_heartbeat_).toSec() <= heartbeat_timeout_)
        return;
    }
    WithWorld([this](const gazebo::physics::WorldPtr &)
    {
      if (active_ && (ros::WallTime::now() - last_heartbeat_).toSec() > heartbeat_timeout_)
      {
        ROS_ERROR("WAM-V circle supervisor heartbeat expired");
        RestoreLocked("supervisor heartbeat expired");
      }
    });
  }

  void OnUpdate(const gazebo::common::UpdateInfo &info)
  {
    WithWorld([&](const gazebo::physics::WorldPtr &world)
    {
      if (!active_)
        return;
      try
      {
        if (world->ModelByName("wamv") != model_)
          throw std::runtime_error("WAM-V model was removed or replaced");
        // A reset can restore SDF modes before this controller has ever ticked.
        for (const auto &state : links_)
          if (!state.link->GetKinematic())
            state.link->SetKinematic(true);
        const double now = info.simTime.Double();
        const bool reset = std::isfinite(last_sim_time_) && now < last_sim_time_;
        if (reset)
        {
          epoch_ = std::numeric_limits<double>::quiet_NaN();
        }
        if (now <= 0.)
        {
          last_sim_time_ = now;
          return;
        }
        const bool starting_phase = !std::isfinite(epoch_);
        if (starting_phase)
          epoch_ = now;
        if (!reset && now == last_sim_time_)
          return;
        const auto sample = cm::SampleMotion(parameters_, now - epoch_);
        model_->SetWorldPose(sample.pose);
        // Setting a model pose alone carries forward existing relative errors.
        // Reconstruct every link from the captured assembly at this same tick.
        for (const auto &state : links_)
        {
          // Scripted targets do not integrate external wrenches. Clear them
          // after the model plugins run, before ODE: a hydrodynamics plugin
          // can produce NaN forces when its time history crosses a reset;
          // even a kinematic ODE body is corrupted by 0 * NaN.
          state.link->SetForce(ignition::math::Vector3d::Zero);
          state.link->SetTorque(ignition::math::Vector3d::Zero);
          state.link->SetWorldPose(sample.pose * state.relative_pose);
          state.link->SetAngularVel(sample.angular);
          state.link->SetLinearVel(cm::CenterOfMassVelocity(sample, state.link->WorldCoGPose().Pos()));
        }
        if (starting_phase)
          ROS_INFO_STREAM("WAM-V circle phase started at simulation time " << now
                          << "; model pose " << model_->WorldPose());
        last_sim_time_ = now;
      }
      catch (const std::exception &error)
      {
        ROS_ERROR_STREAM("WAM-V circle motion failed: " << error.what());
        RestoreLocked(error.what());
      }
    });
  }

  std::mutex mutex_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  std::vector<LinkState> links_;
  cm::Parameters parameters_;
  bool active_ = false;
  double epoch_ = std::numeric_limits<double>::quiet_NaN();
  double last_sim_time_ = std::numeric_limits<double>::quiet_NaN();
  double heartbeat_timeout_ = 2.;
  ros::WallTime last_heartbeat_;
  std::string last_error_ = "motion not started";
  gazebo::event::ConnectionPtr world_created_, update_;
  ros::CallbackQueue queue_;
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::ServiceServer start_service_, keepalive_service_, stop_service_;
  ros::WallTimer watchdog_;
};

GZ_REGISTER_SYSTEM_PLUGIN(WamvCircleMotionPlugin)
}  // namespace ocean_simulation_gazebo
