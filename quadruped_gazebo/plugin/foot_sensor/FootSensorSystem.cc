/**
 * @file FootSensorSystem.cc
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

#include "FootSensorSystem.hh"

#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

using namespace gz;
using namespace gz::sim;
using namespace systems;

//////////////////////////////////////////////////
void FootSensor::Load(const std::string &_topic, const Entity &_collisionEntity)
{
  this->collisionEntity = _collisionEntity;
  this->contactSensor = std::move(std::make_unique<gz::sim::components::ContactSensor>());
  this->contactPub = this->node.Advertise<ignition::msgs::Boolean>(_topic + "/contact");
  this->forcePub = this->node.Advertise<ignition::msgs::Wrench>(_topic + "/force");
}

//////////////////////////////////////////////////
void FootSensor::Publish(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
  *this->boolMsg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_info.simTime);
  *this->wrenchMsg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_info.simTime);

  auto contacts = _ecm.Component<components::ContactSensorData>(this->collisionEntity);
  this->boolMsg.set_data(contacts->Data().contact_size() > 0);

  this->contactPub.Publish(this->boolMsg);
  this->forcePub.Publish(this->wrenchMsg);
}

//////////////////////////////////////////////////
FootSensorSystem::FootSensorSystem()
    : gz::sim::System(), dataPtr(std::make_unique<FootSensorSystemPrivate>())
{
}

//////////////////////////////////////////////////
void FootSensorSystem::Configure(const Entity &, const std::shared_ptr<const sdf::Element> &,
                                 EntityComponentManager &_ecm, EventManager &)
{
  IGN_PROFILE("FootSensorSystem::Configure");

  _ecm.Each<gz::sim::components::ContactSensor>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::ContactSensor *_contact) -> bool
      {
        // Check if the parent entity is a link
        auto *parentEntity = _ecm.Component<components::ParentEntity>(_entity);
        if (parentEntity == nullptr)
          return true;

        // Contact sensors should only be attached to links
        auto *linkComp = _ecm.Component<components::Link>(parentEntity->Data());
        if (linkComp == nullptr)
          return true;

        // Only the first collision element is used
        auto collisionElem = _contact->Data()->GetElement("contact")->GetElement("collision");
        auto collisionName = collisionElem->Get<std::string>();

        auto childEntities = _ecm.ChildrenByComponents(
            parentEntity->Data(), components::Collision(), components::Name(collisionName));
        auto collisionEntity = childEntities.front();

        // Create component to be filled by physics
        if (!childEntities.empty())
          _ecm.CreateComponent(collisionEntity, components::ContactSensorData());

        std::string sensorName = gz::sim::scopedName(_entity, _ecm, "/", false);
        std::string tmpTopic = _contact->Data()->Get<std::string>("topic", "__default__").first;

        std::string topicName = tmpTopic == "__default__" ? sensorName : tmpTopic;

        auto sensor = std::make_unique<FootSensor>();

        sensor->Load(topicName, collisionEntity);
        this->dataPtr->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void FootSensorSystem::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
  IGN_PROFILE("FootSensorSystem::PostUpdate");

  if (!_info.paused)
  {
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Publish sensor data
      it.second->Publish(_info, _ecm);
    }
  }
}

IGNITION_ADD_PLUGIN(FootSensorSystem, gz::sim::System, FootSensorSystem::ISystemConfigure,
                    FootSensorSystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(FootSensorSystem, "gz::sim::systems::FootSensorSystem")
