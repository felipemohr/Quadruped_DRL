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

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace gz::sim;
using namespace systems;

//////////////////////////////////////////////////
void FootSensor::Publish(const std::chrono::steady_clock::duration &_now)
{
  *boolMsg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
  *wrenchMsg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);

  contactPub.Publish(boolMsg);
  forcePub.Publish(wrenchMsg);
}

//////////////////////////////////////////////////
FootSensorSystem::FootSensorSystem()
    : gz::sim::System(), dataPtr(std::make_unique<FootSensorSystemPrivate>())
{
}

//////////////////////////////////////////////////
void FootSensorSystem::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &,
                                 EntityComponentManager &_ecm, EventManager &)
{
  IGN_PROFILE("FootSensorSystem::Configure");

  _ecm.Each<gz::sim::components::ContactSensor>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::ContactSensor *_contact) -> bool
      {
        std::string sensorName = gz::sim::scopedName(_entity, _ecm, "/", false);
        std::string tmpTopic = _contact->Data()->Get<std::string>("topic", "__default__").first;

        std::string topicName = tmpTopic == "__default__" ? sensorName : tmpTopic;

        ignerr << topicName << std::endl;

        auto sensor = std::make_unique<FootSensor>();
        sensor->contactSensor = std::move(std::make_unique<gz::sim::components::ContactSensor>());
        sensor->contactPub =
            this->dataPtr->node.Advertise<ignition::msgs::Boolean>(topicName + "/contact");
        sensor->forcePub =
            this->dataPtr->node.Advertise<ignition::msgs::Wrench>(topicName + "/force");

        this->dataPtr->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void FootSensorSystem::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  IGN_PROFILE("FootSensorSystem::PreUpdate");
}

//////////////////////////////////////////////////
void FootSensorSystem::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &)
{
  IGN_PROFILE("FootSensorSystem::PostUpdate");

  if (!_info.paused)
  {

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Publish sensor data
      it.second->Publish(_info.simTime);
    }
  }
}

IGNITION_ADD_PLUGIN(FootSensorSystem, gz::sim::System, FootSensorSystem::ISystemConfigure,
                    FootSensorSystem::ISystemPreUpdate, FootSensorSystem::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(FootSensorSystem, "gz::sim::systems::FootSensorSystem")
