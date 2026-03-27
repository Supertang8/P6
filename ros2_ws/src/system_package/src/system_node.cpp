#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <memory>
#include <atomic>
#include <sstream>
#include <vector>
#include <chrono>

using Trigger = std_srvs::srv::Trigger;

class SystemNode : public rclcpp::Node {
public:
  SystemNode() : Node("system_node"), drone_cloud_received_(false), rover_cloud_received_(false) {
    // Declare parameters
    this->declare_parameter<std::string>("drone_aggregated_topic", "/drone/aggregated_pointcloud");
    this->declare_parameter<std::string>("rover_aggregated_topic", "/rover/aggregated_pointcloud");
    this->declare_parameter<std::string>("drone_trigger_service", "/drone/trigger_accumulation");
    this->declare_parameter<std::string>("rover_trigger_service", "/rover/trigger_accumulation");
    this->declare_parameter<double>("request_retry_timeout_sec", 6.0);
    this->declare_parameter<std::vector<double>>(
      "initial_guess_drone_to_rover",
      std::vector<double>{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
      });

    // Get parameter values
    const auto initial_guess = this->get_parameter("initial_guess_drone_to_rover").as_double_array();
    if (initial_guess.size() == 16) {
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          initial_guess_drone_to_rover_(r, c) = static_cast<float>(initial_guess[r * 4 + c]);
        }
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter 'initial_guess_drone_to_rover' must contain 16 values. Using identity.");
    }

    drone_aggregated_topic_ = this->get_parameter("drone_aggregated_topic").as_string();
    rover_aggregated_topic_ = this->get_parameter("rover_aggregated_topic").as_string();
    drone_trigger_service_ = this->get_parameter("drone_trigger_service").as_string();
    rover_trigger_service_ = this->get_parameter("rover_trigger_service").as_string();
    request_retry_timeout_sec_ = this->get_parameter("request_retry_timeout_sec").as_double();

    RCLCPP_INFO(this->get_logger(), "Drone aggregated topic: %s", drone_aggregated_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Rover aggregated topic: %s", rover_aggregated_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Drone trigger service: %s", drone_trigger_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Rover trigger service: %s", rover_trigger_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Retry timeout: %.2f s", request_retry_timeout_sec_);

    // Subscribe to aggregated pointcloud topics from the two remote aggregators.
    auto aggregated_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    aggregated_qos.reliable();
    aggregated_qos.transient_local();
    sub_drone_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      drone_aggregated_topic_,
      aggregated_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_drone_cloud(msg); });

    sub_rover_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      rover_aggregated_topic_,
      aggregated_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_rover_cloud(msg); });

    // Clients used to request one accumulated cloud from each machine.
    drone_client_ = this->create_client<Trigger>(drone_trigger_service_);
    rover_client_ = this->create_client<Trigger>(rover_trigger_service_);

    request_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { request_accumulations(); });

    RCLCPP_INFO(this->get_logger(), "System node initialized. Waiting for trigger services and aggregated clouds...");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_drone_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rover_;
  rclcpp::Client<Trigger>::SharedPtr drone_client_;
  rclcpp::Client<Trigger>::SharedPtr rover_client_;
  rclcpp::TimerBase::SharedPtr request_timer_;

  std::atomic<bool> drone_cloud_received_;
  std::atomic<bool> rover_cloud_received_;
  bool drone_requested_ = false;
  bool rover_requested_ = false;
  bool drone_request_in_flight_ = false;
  bool rover_request_in_flight_ = false;
  bool icp_done_ = false;
  rclcpp::Time drone_last_request_time_;
  rclcpp::Time rover_last_request_time_;

  std::string drone_aggregated_topic_;
  std::string rover_aggregated_topic_;
  std::string drone_trigger_service_;
  std::string rover_trigger_service_;
  double request_retry_timeout_sec_ = 6.0;

  Eigen::Matrix4f initial_guess_drone_to_rover_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f drone_lidar_to_rover_lidar_transform_ = Eigen::Matrix4f::Identity();

  pcl::PointCloud<pcl::PointXYZ> drone_lidar_cloud_;
  pcl::PointCloud<pcl::PointXYZ> rover_lidar_cloud_;

  void request_accumulations() {
    if (icp_done_) {
      return;
    }

    const auto now = this->now();

    const bool drone_timed_out = drone_requested_ && !drone_cloud_received_ &&
      (now - drone_last_request_time_).seconds() > request_retry_timeout_sec_;
    const bool rover_timed_out = rover_requested_ && !rover_cloud_received_ &&
      (now - rover_last_request_time_).seconds() > request_retry_timeout_sec_;

    if (drone_timed_out) {
      drone_requested_ = false;
      RCLCPP_WARN(this->get_logger(), "No drone cloud received in time, retrying trigger request.");
    }
    if (rover_timed_out) {
      rover_requested_ = false;
      RCLCPP_WARN(this->get_logger(), "No rover cloud received in time, retrying trigger request.");
    }

    if (!drone_cloud_received_ && !drone_requested_ && !drone_request_in_flight_) {
      if (!drone_client_->wait_for_service(std::chrono::milliseconds(200))) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Waiting for drone trigger service: %s", drone_trigger_service_.c_str());
      } else {
        drone_request_in_flight_ = true;
        drone_last_request_time_ = now;
        auto req = std::make_shared<Trigger::Request>();
        drone_client_->async_send_request(
          req,
          [this](rclcpp::Client<Trigger>::SharedFuture future) {
            drone_request_in_flight_ = false;
            const auto response = future.get();
            if (response->success) {
              drone_requested_ = true;
              RCLCPP_INFO(this->get_logger(), "Drone accumulation requested: %s", response->message.c_str());
            } else {
              drone_requested_ = false;
              RCLCPP_WARN(this->get_logger(), "Drone accumulation request rejected: %s", response->message.c_str());
            }
          });
      }
    }

    if (!rover_cloud_received_ && !rover_requested_ && !rover_request_in_flight_) {
      if (!rover_client_->wait_for_service(std::chrono::milliseconds(200))) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Waiting for rover trigger service: %s", rover_trigger_service_.c_str());
      } else {
        rover_request_in_flight_ = true;
        rover_last_request_time_ = now;
        auto req = std::make_shared<Trigger::Request>();
        rover_client_->async_send_request(
          req,
          [this](rclcpp::Client<Trigger>::SharedFuture future) {
            rover_request_in_flight_ = false;
            const auto response = future.get();
            if (response->success) {
              rover_requested_ = true;
              RCLCPP_INFO(this->get_logger(), "Rover accumulation requested: %s", response->message.c_str());
            } else {
              rover_requested_ = false;
              RCLCPP_WARN(this->get_logger(), "Rover accumulation request rejected: %s", response->message.c_str());
            }
          });
      }
    }
  }

  void on_drone_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!icp_done_) {
      pcl::fromROSMsg(*msg, drone_lidar_cloud_);
      drone_cloud_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received drone aggregated cloud with %zu points", drone_lidar_cloud_.size());
      run_icp_if_ready();
    }
  }

  void on_rover_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!icp_done_) {
      pcl::fromROSMsg(*msg, rover_lidar_cloud_);
      rover_cloud_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Received rover aggregated cloud with %zu points", rover_lidar_cloud_.size());
      run_icp_if_ready();
    }
  }

  void run_icp_if_ready() {
    if (icp_done_ || !drone_cloud_received_ || !rover_cloud_received_) return;

    if (drone_lidar_cloud_.empty() || rover_lidar_cloud_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot run ICP: one or both accumulated clouds are empty.");
      icp_done_ = true;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Running ICP on aggregated clouds...");
    RCLCPP_INFO(this->get_logger(), "Drone cloud: %zu points, Rover cloud: %zu points",
                drone_lidar_cloud_.size(), rover_lidar_cloud_.size());

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(drone_lidar_cloud_.makeShared());
    icp.setInputTarget(rover_lidar_cloud_.makeShared());

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess_drone_to_rover_);
    icp_done_ = true;

    if (!icp.hasConverged()) {
      RCLCPP_ERROR(this->get_logger(), "ICP did not converge.");
      return;
    }

    request_timer_->cancel();

    drone_lidar_to_rover_lidar_transform_ = icp.getFinalTransformation();
    std::ostringstream oss;
    oss << drone_lidar_to_rover_lidar_transform_;
    RCLCPP_INFO(this->get_logger(), "ICP converged. Fitness: %.8f", icp.getFitnessScore());
    RCLCPP_INFO(this->get_logger(), "Transform (drone cloud -> rover cloud):\n%s", oss.str().c_str());

    pcl::PointCloud<pcl::PointXYZ> drone_in_rover_cloud_initial_guess;
    pcl::PointCloud<pcl::PointXYZ> drone_in_rover_cloud_icp;
    pcl::transformPointCloud(drone_lidar_cloud_, drone_in_rover_cloud_initial_guess, initial_guess_drone_to_rover_);
    pcl::transformPointCloud(drone_lidar_cloud_, drone_in_rover_cloud_icp, drone_lidar_to_rover_lidar_transform_);

    if (pcl::io::savePCDFileBinary("rover_in_rover_frame.pcd", rover_lidar_cloud_) != 0 ||
        pcl::io::savePCDFileBinary("drone_in_drone_frame.pcd", drone_lidar_cloud_) != 0 ||
        pcl::io::savePCDFileBinary("drone_in_rover_frame_initial_guess.pcd", drone_in_rover_cloud_initial_guess) != 0 ||
        pcl::io::savePCDFileBinary("drone_in_rover_frame_icp.pcd", drone_in_rover_cloud_icp) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save one or more output PCD files.");
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Saved PCDs: %s, %s, %s, %s",
      "rover_in_rover_frame.pcd",
      "drone_in_drone_frame.pcd",
      "drone_in_rover_frame_initial_guess.pcd",
      "drone_in_rover_frame_icp.pcd");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
