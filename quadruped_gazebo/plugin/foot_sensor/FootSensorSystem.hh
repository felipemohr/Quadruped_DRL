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

#ifndef GZ_GAZEBO_SYSTEMS_FOOT_SENSOR_HH_
#define GZ_GAZEBO_SYSTEMS_FOOT_SENSOR_HH_

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
   * @brief Load the Foot sensor with the collisionEntites from the Contact sensor
   *
   * @param _topic String with the topic name
   * @param _collisionEntity The entity that act as contact
   */
  void Load(const std::string &_topic, const Entity &_collisionEntity);

  /**
   * @brief Publish contact and force data
   *
   * @param _info The UpdateInfo of the given simulation
   * @param _ecm The EntityComponentManager of the given simulation
   */
  void Publish(const UpdateInfo &_info, const EntityComponentManager &_ecm);

  /** @brief Contact sensor component */
  std::unique_ptr<gz::sim::components::ContactSensor> contactSensor;

  /** @brief Ign transport publisher to push boolean messages */
  gz::transport::Node::Publisher contactPub;

  /** @brief Ign transport publisher to publish wrench messages */
  gz::transport::Node::Publisher forcePub;

  /** @brief Entitity for which the Contact sensor publishes data */
  Entity collisionEntity;

private:
  /** @brief Ign transport node */
  gz::transport::Node node;

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
};

class FootSensorSystem : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
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
