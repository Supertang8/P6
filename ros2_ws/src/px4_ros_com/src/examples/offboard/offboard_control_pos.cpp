/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example using the PX4 ROS 2 Interface Library
 * @file offboard_control_pos.cpp
 *
 * Equivalent to offboard_control_srv.cpp: arms the vehicle and hovers at
 * 5 m altitude (NED position {0, 0, -5}) facing south (yaw = pi rad).
 *
 * The interface library replaces the manual state machine, heartbeat
 * publishing, and service calls with ModeBase / NodeWithMode abstractions.
 */

#include <Eigen/Core>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <rclcpp/rclcpp.hpp>

static const std::string kModeName = "Offboard Pos Setpoint";

class HoverMode : public px4_ros2::ModeBase
{
public:
  explicit HoverMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kModeName})
  {
    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);

    _setpoint_sub = node.create_subscription<geometry_msgs::msg::PoseStamped>(
      "offboard/setpoint", rclcpp::QoS(1).best_effort(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        _position_ned = Eigen::Vector3f{
          static_cast<float>(msg->pose.position.x),
          static_cast<float>(msg->pose.position.y),
          static_cast<float>(msg->pose.position.z)
        };
        const auto & q = msg->pose.orientation;
        _yaw_rad = std::atan2(
          2.f * static_cast<float>(q.w * q.z + q.x * q.y),
          1.f - 2.f * static_cast<float>(q.y * q.y + q.z * q.z));
      });
  }

  void onActivate() override
  {
    RCLCPP_INFO(node().get_logger(), "HoverMode activated, listening for setpoints on 'offboard/setpoint'");
  }

  void updateSetpoint(float /*dt_s*/) override
  {
      _goto_setpoint->update(
          _position_ned,
          _yaw_rad,
          2.0f,   // max horizontal speed [m/s]
          1.0f,   // max vertical speed   [m/s]
          0.5f    // max heading rate     [rad/s]
      );
  }

  private:
  // Default: 5 m altitude, facing south — overridden by incoming topic messages
  Eigen::Vector3f _position_ned{0.f, 0.f, -3.f};
  float           _yaw_rad{static_cast<float>(M_PI)};

  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType>              _goto_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    _setpoint_sub;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<px4_ros2::NodeWithMode<HoverMode>>("offboard_control_pos", true));
  rclcpp::shutdown();
  return 0;
}
