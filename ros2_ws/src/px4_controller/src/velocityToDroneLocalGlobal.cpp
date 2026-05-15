// Transforms velocity commands from a LIO-SAM gravity-aligned map frame
// (FLU, X = rover/drone startup forward, Y = left, Z = up) into PX4's
// true-NED local frame before forwarding them to the trajectory setpoint.
//
// Assumes the rover and drone are initialized with the same body heading,
// so rover/map and drone/map share an orientation. Only the drone startup
// yaw psi_d (CW from north) is needed, captured once from
// /fmu/out/vehicle_attitude.
//
//   R_ned_from_flu = | cos psi_d   sin psi_d    0 |
//                    | sin psi_d  -cos psi_d    0 |
//                    | 0           0           -1 |

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
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
        Eigen::Vector3f v_in{
          static_cast<float>(msg->twist.linear.x),
          static_cast<float>(msg->twist.linear.y),
          static_cast<float>(msg->twist.linear.z)
        };
        if (!_initial_yaw_set) {
          _velocity_ned.setZero();  // hover until startup yaw is captured
          return;
        }
        _velocity_ned = _R_ned_from_flu * v_in;
      });

    _attitude_sub = node.create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", rclcpp::QoS(1).best_effort(),
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        if (_initial_yaw_set) {
          return;
        }
        // PX4 q = [w, x, y, z], rotates FRD body into the NED earth frame.
        // Standard ZYX yaw extraction (avoids Eigen eulerAngles wrap-around).
        const float w = msg->q[0];
        const float x = msg->q[1];
        const float y = msg->q[2];
        const float z = msg->q[3];
        _initial_yaw_ned = std::atan2(2.f * (w * z + x * y),
                                      1.f - 2.f * (y * y + z * z));

        const float c = std::cos(_initial_yaw_ned);
        const float s = std::sin(_initial_yaw_ned);
        _R_ned_from_flu <<  c,    s,    0.f,
                            s,   -c,    0.f,
                            0.f,  0.f, -1.f;

        _initial_yaw_set = true;
        RCLCPP_INFO(this->node().get_logger(),
                    "Locked velocity rotation: drone startup yaw = %.2f deg.",
                    _initial_yaw_ned * 180.0f / static_cast<float>(M_PI));
      });
  }

  void onActivate() override
  {
    RCLCPP_INFO(
      node().get_logger(),
      "VelocityMode activated; listening on '/offboard/velocity' (LIO-SAM map FLU -> PX4 NED).");
  }

  void updateSetpoint(float /*dt_s*/) override
  {
    _velocity_setpoint->update(_velocity_ned);
  }

private:
  Eigen::Vector3f _velocity_ned{0.f, 0.f, 0.f};
  Eigen::Matrix3f _R_ned_from_flu{Eigen::Matrix3f::Identity()};
  float _initial_yaw_ned{0.f};
  bool _initial_yaw_set{false};

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _velocity_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _twist_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _attitude_sub;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<px4_ros2::NodeWithMode<VelocityMode>>("velocityToDroneLocalGlobal", true));
  rclcpp::shutdown();
  return 0;
}
