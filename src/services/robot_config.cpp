/*
 * Copyright 2015 Aldebaran
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

#include "robot_config.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

RobotConfigService::RobotConfigService( const std::string& name, const std::string& topic, const qi::SessionPtr& session )
  : name_(name),
  topic_(topic),
  session_(session)
{}

void RobotConfigService::reset( rclcpp::Node& node )
{
  service_ = node.create_service<naoqi_bridge_msgs::srv::GetRobotInfo>(topic_, &RobotConfigService::callback);
}

void RobotConfigService::callback( const naoqi_bridge_msgs::srv::GetRobotInfo::Request& req, naoqi_bridge_msgs::srv::GetRobotInfo::Response& resp )
{
  resp->info = helpers::driver::getRobotInfo(session_);
}


}
}
