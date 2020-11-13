// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "halodi_msgs/msg/driving_command.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DrivingCommandPublisher : public rclcpp::Node
{
public:
  DrivingCommandPublisher()
  : Node("driving_command_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<halodi_msgs::msg::DrivingCommand>("/eve/driving_command", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&DrivingCommandPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = halodi_msgs::msg::DrivingCommand();
    message.filter_driving_command = false;
    message.linear_velocity = 1.0;
    message.angular_velocity = 3.0;
    RCLCPP_INFO(this->get_logger(), "DrivingCommand: linear_velocity: '%f', angular_velocity: '%f'", message.linear_velocity, message.angular_velocity);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<halodi_msgs::msg::DrivingCommand>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrivingCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
