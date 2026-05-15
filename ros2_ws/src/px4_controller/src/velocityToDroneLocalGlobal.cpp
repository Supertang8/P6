#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
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
        // Transform velocity from global frame (startup local position, ENU rotated by initial yaw) to NED
        Eigen::Vector3f vel_global{
          static_cast<float>(msg->twist.linear.x),
          static_cast<float>(msg->twist.linear.y),
          static_cast<float>(msg->twist.linear.z)
        };
        if (_initial_yaw_set) {
          float cos_yaw = cosf(_initial_yaw);
          float sin_yaw = sinf(_initial_yaw);
          _velocity_ned = Eigen::Vector3f{
            vel_global.x() * sin_yaw + vel_global.y() * cos_yaw,  // NED north
            vel_global.x() * cos_yaw - vel_global.y() * sin_yaw,  // NED east
            vel_global.z()  // NED down
          };
        } else {
          // If initial yaw not set, assume 0 (ENU aligned)
          _velocity_ned = Eigen::Vector3f{
            vel_global.y(),
           -vel_global.x(),
            vel_global.z()
          };
        }
        _yaw_rate_rad_s = static_cast<float>(msg->twist.angular.z);
      });

    _attitude_sub = node.create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", rclcpp::QoS(1).best_effort(),
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        if (!_initial_yaw_set) {
          Eigen::Quaternionf q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
          Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX: yaw, pitch, roll
          _initial_yaw = euler[0];
          _initial_yaw_set = true;
          RCLCPP_INFO(this->node().get_logger(), "Initial yaw set to %f radians", _initial_yaw);
        }
      });
  }

  void onActivate() override
  {
    RCLCPP_INFO(
      node().get_logger(),
      "VelocityMode activated, listening for velocity commands on '/offboard/velocity' (global frame aligned with startup orientation)");
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
  float           _initial_yaw{0.f};
  bool            _initial_yaw_set{false};

  std::shared_ptr<px4_ros2::TrajectorySetpointType>                   _velocity_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr   _twist_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr     _attitude_sub;
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
