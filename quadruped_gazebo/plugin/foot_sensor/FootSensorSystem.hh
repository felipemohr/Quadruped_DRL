/**
 * @file FootSensorSystem.hh
 * @author Felipe Mohr (felipe18mohr@gmail.com)
 * @brief Foot Sensor System plugin for Gazebo Ignition simulation
 * @version 1.0
 * @date 2024-06-29
 *
 * @copyright Copyright (c) 2024
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

#ifndef GZ_GTAZEBO_SYSTEM_FOOT_SENSOR_HH_
#define GZ_GTAZEBO_SYSTEM_FOOT_SENSOR_HH_

#include <memory>
#include <unordered_map>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensor.hh>

#include <gz/transport/Node.hh>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{

class FootSensor
{
public:
  /**
   * @brief Publishes contact and force data
   *
   * @param _now The current time
   */
  void Publish(const std::chrono::steady_clock::duration &_now);

  /** @brief Contact sensor component */
  std::unique_ptr<gz::sim::components::ContactSensor> contactSensor;

  /** @brief Ign transport publisher to push boolean messages */
  gz::transport::Node::Publisher contactPub;

  /** @brief Ign transport publisher to publish wrench messages */
  gz::transport::Node::Publisher forcePub;

private:
  /** @brief Boolean message to publish sensor contact data */
  ignition::msgs::Boolean boolMsg;

  /** @brief Wrench message to publish sensor force data */
  ignition::msgs::Wrench wrenchMsg;
};

class FootSensorSystemPrivate
{
public:
  /** @brief A map of Contact entity to its Foot sensor */
  std::unordered_map<Entity, std::unique_ptr<FootSensor>> entitySensorMap;

  /** @brief Ign transport node */
  gz::transport::Node node;
};

class FootSensorSystem : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemPostUpdate
{
public:
  /** @brief Constructor */
  FootSensorSystem();

  /** @brief Destructor */
  ~FootSensorSystem() final = default;

  /** Documentation inherited */
  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &_eventMgr) final;

  /** Documentation inherited */
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) final;

  /** Documentation inherited */
  void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) final;

private:
  /** @brief Private data pointer */
  std::unique_ptr<FootSensorSystemPrivate> dataPtr;
};

} // namespace systems
} // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
} // namespace gazebo
} // namespace ignition

#endif
