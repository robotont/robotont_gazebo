/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "OdomOmnidirectional.hh"

#include <ignition/msgs/odometry.pb.h>

#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Velocity command.
struct Commands
{
  /// \brief Linear velocity.
  double lin;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin(0.0), ang(0.0) {}
};

class ignition::gazebo::systems::OmniDrivePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Entity of the left joint
  public: std::vector<Entity> leftJoints;

  /// \brief Entity of the right joint
  public: std::vector<Entity> rightJoints;

  /// \brief Name of left joint
  public: std::vector<std::string> leftJointNames;

  /// \brief Name of right joint
  public: std::vector<std::string> rightJointNames;

  /// \brief Name of odom frame
  public: std::vector<std::string> odom_frameNames;

  /// \brief Calculated speed of left joint
  // public: double leftJointSpeed{0};

  /// \brief Calculated speed of right joint
  // public: double rightJointSpeed{0};

  /// \brief Distance between wheels
  public: double wheelSeparation{1.0};

  /// \brief Wheel radius
  public: double wheelRadius{0.2};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Diff drive odometry.
  public: math::DiffDriveOdometry odom;

  /// \brief Omni drive odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Linear velocity limiter.
  // public: std::unique_ptr<SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  // public: std::unique_ptr<SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
};

//////////////////////////////////////////////////
OmniDrive::OmniDrive()
  : dataPtr(std::make_unique<OmniDrivePrivate>())
{
}

//////////////////////////////////////////////////
void OmniDrive::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "OmniDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",this->dataPtr->wheelSeparation).first;
 
  

  double odomFreq = _sdf->Get<double>("odometryRate", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  
  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
  if (_sdf->HasElement("commandTopic"))
    topic = _sdf->Get<std::string>("commandTopic");
  this->dataPtr->node.Subscribe(topic, &OmniDrivePrivate::OnCmdVel,
      this->dataPtr.get());

  // std::string odomTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
  //   "/odometry"};

  std::string odomTopic{this->dataPtr->model.Name(_ecm) + "/odom"};
  if (_sdf->HasElement("odometryTopic"))
    odomTopic = _sdf->Get<std::string>("odometryTopic");
  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  ignmsg << "Odom subscribing to twist messages on [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void OmniDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OmniDrive::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  
}

//////////////////////////////////////////////////
void OmniDrive::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("OmniDrive::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void OmniDrivePrivate::UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OmniDrive::UpdateOdometry");
  // Initialize, if not already initialized.
  // if (!this->odom.Initialized())
  // {
  //   this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
  //   return;
  // }

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  msgs::Odometry msg;  

  const double dt = std::chrono::duration<double>(diff).count();
  
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  msg.mutable_pose()->mutable_position()->set_x(x);
  msg.mutable_pose()->mutable_position()->set_y(y);

  math::Quaterniond orientation(0, 0, th);
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(vx);
  msg.mutable_twist()->mutable_linear()->set_y(vy);
  msg.mutable_twist()->mutable_angular()->set_z(vth);

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->model.Name(_ecm) + "/odom");

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (linkName)
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
  }

  // Publish the message
  this->odomPub.Publish(msg);
}

//////////////////////////////////////////////////
void OmniDrivePrivate::UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("OmniDrive::UpdateVelocity");

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    vx = this->targetVel.linear().x();
    vy = this->targetVel.linear().y();
    vth = this->targetVel.angular().z();
  }

  const double dt = std::chrono::duration<double>(_info.dt).count();  
}

//////////////////////////////////////////////////
void OmniDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

IGNITION_ADD_PLUGIN(OmniDrive,
                    ignition::gazebo::System,
                    OmniDrive::ISystemConfigure,
                    OmniDrive::ISystemPreUpdate,
                    OmniDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OmniDrive, "ignition::gazebo::systems::OmniDrive")
