// Copyright 2021 Halodi Robotics
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

#include "halodi_msgs/msg/whole_body_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace eve_ros2_examples {

using namespace std::chrono_literals;
using std::placeholders::_1;

class WholeBodyStateSubscriber : public rclcpp::Node {
 public:
  WholeBodyStateSubscriber() : Node("wbs_subscriber") {
    rclcpp::QoS qos(10);
    qos.best_effort();

    subscription_ = this->create_subscription<halodi_msgs::msg::WholeBodyState>(
        "/eve/whole_body_state", qos, std::bind(&WholeBodyStateSubscriber::topicCallback, this, _1));
  }

 private:
  void topicCallback(const halodi_msgs::msg::WholeBodyState::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "The robot pelvis is at postion: %.3f, %.3f, %.3f in world frame",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }
  rclcpp::Subscription<halodi_msgs::msg::WholeBodyState>::SharedPtr subscription_;
};

}  // namespace eve_ros2_examples

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eve_ros2_examples::WholeBodyStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}
