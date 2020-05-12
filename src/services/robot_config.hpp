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


#ifndef ROBOT_CONFIG_SERVICE_HPP
#define ROBOT_CONFIG_SERVICE_HPP

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <naoqi_bridge_msgs/srv/GetRobotInfo.hpp>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class RobotConfigService
{
public:
  RobotConfigService( const std::string& name, const std::string& topic, const qi::SessionPtr& session );

  ~RobotConfigService(){};

  std::string name() const
  {
    return name_;
  }

  std::string topic() const
  {
    return topic_;
  }

  void reset( rclcpp::Node& node );

  void callback( const naoqi_bridge_msgs::srv::GetRobotInfo::Request& req, naoqi_bridge_msgs::srv::GetRobotInfo::Response& resp );


private:
  const std::string name_;
  const std::string topic_;

  const qi::SessionPtr& session_;
  rclcpp::Service<naoqi_bridge_msgs::srv::GetRobotInfo>::SharedPtr service_;
};

} // service
} // naoqi
#endif
