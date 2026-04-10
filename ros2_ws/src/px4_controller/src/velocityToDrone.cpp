#include <Eigen/Core>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

static const std::string kModeName = "Offboard Vel Setpoint";

class VelocityMode : public px4_ros2::ModeBase
{
public:
  explicit VelocityMode(rclcpp::Node & node)
  : ModeBase(node, Settings{kModeName})
  {
    _velocity_setpoint =
      std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    _twist_sub = node.create_subscription<geometry_msgs::msg::TwistStamped>(
      "/offboard/velocity", rclcpp::QoS(1).best_effort(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        _velocity_ned = Eigen::Vector3f{
          static_cast<float>(msg->twist.linear.x),
          static_cast<float>(msg->twist.linear.y),
          static_cast<float>(msg->twist.linear.z)
        };
        _yaw_rate_rad_s = static_cast<float>(msg->twist.angular.z);
      });
  }

  void onActivate() override
  {
    RCLCPP_INFO(
      node().get_logger(),
      "VelocityMode activated, listening for velocity commands on '/offboard/velocity'");
  }

  void updateSetpoint(float /*dt_s*/) override
  {
    // TrajectorySetpointType takes velocity only; yaw-rate is not separately
    // controllable through this type — heading will be managed by PX4 internally.
    _velocity_setpoint->update(_velocity_ned);
  }

private:
  // Default: hover in place until the first message arrives
  Eigen::Vector3f _velocity_ned{0.f, 0.f, 0.f};
  float           _yaw_rate_rad_s{0.f};

  std::shared_ptr<px4_ros2::TrajectorySetpointType>                   _velocity_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr   _twist_sub;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<px4_ros2::NodeWithMode<VelocityMode>>("velocityToDrone", true));
  rclcpp::shutdown();
  return 0;
}