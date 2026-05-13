#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <cmath>
#include <memory>

using CustomMsg = livox_ros_driver2::msg::CustomMsg;

class PointcloudAggregatorNode : public rclcpp::Node {
public:
  PointcloudAggregatorNode() 
    : Node("pointcloud_aggregator_node"),
      lidar_online_(false),
      collecting_(false) {
    
    // Declare parameters
    this->declare_parameter<std::string>("lidar_topic", "livox/lidar");
    this->declare_parameter<std::string>("aggregated_topic", "aggregated_pointcloud");
    this->declare_parameter<std::string>("trigger_service", "trigger_accumulation");
    this->declare_parameter<std::string>("output_frame_id", "livox_frame");
    this->declare_parameter<std::string>("shutdown_topic", "shutdown");
    this->declare_parameter<int>("messages_to_accumulate", 10);
    this->declare_parameter<float>("downsample_leaf_size", 0.05f);
    this->declare_parameter<float>("min_dist", 1.0f);
    this->declare_parameter<float>("max_dist", 10.0f);

    // Get parameter values
    lidar_topic_ = this->get_parameter("lidar_topic").as_string();
    aggregated_topic_ = this->get_parameter("aggregated_topic").as_string();
    trigger_service_ = this->get_parameter("trigger_service").as_string();
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();
    const int requested = this->get_parameter("messages_to_accumulate").as_int();
    messages_to_accumulate_ = static_cast<size_t>(requested > 0 ? requested : 1);
    downsample_leaf_size_ = static_cast<float>(this->get_parameter("downsample_leaf_size").as_double());
    min_dist_ = static_cast<float>(this->get_parameter("min_dist").as_double());
    max_dist_ = static_cast<float>(this->get_parameter("max_dist").as_double());

    if (min_dist_ < 0.0f) {
      RCLCPP_WARN(this->get_logger(), "min_dist < 0 (%.4f). Clamping to 0.", min_dist_);
      min_dist_ = 0.0f;
    }
    if (max_dist_ <= min_dist_) {
      const float adjusted = min_dist_ + 0.1f;
      RCLCPP_WARN(
        this->get_logger(),
        "max_dist (%.4f) must be greater than min_dist (%.4f). Adjusting max_dist to %.4f.",
        max_dist_, min_dist_, adjusted);
      max_dist_ = adjusted;
    }
    min_dist_sq_ = min_dist_ * min_dist_;
    max_dist_sq_ = max_dist_ * max_dist_;

    RCLCPP_INFO(this->get_logger(), "Input lidar topic: %s", lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output aggregated topic: %s", aggregated_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Trigger service: %s", trigger_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Messages to accumulate: %zu, downsample leaf: %.4f",
      messages_to_accumulate_, downsample_leaf_size_);
    RCLCPP_INFO(this->get_logger(), "Distance crop: min_dist=%.3f m, max_dist=%.3f m",
      min_dist_, max_dist_);

    sub_lidar_ = this->create_subscription<CustomMsg>(
      lidar_topic_,
      rclcpp::SensorDataQoS(),
      [this](const CustomMsg::SharedPtr msg) { on_lidar(msg); });

    const std::string shutdown_topic = this->get_parameter("shutdown_topic").as_string();
    sub_shutdown_ = this->create_subscription<std_msgs::msg::Empty>(
      shutdown_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Empty::UniquePtr) {
        RCLCPP_INFO(this->get_logger(), "Received shutdown signal; exiting.");
        rclcpp::shutdown();
      });

    auto aggregated_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    aggregated_qos.reliable();
    aggregated_qos.transient_local();
    pub_aggregated_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      aggregated_topic_, aggregated_qos);

    srv_trigger_ = this->create_service<std_srvs::srv::Trigger>(
      trigger_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          (void)request;
          this->handle_trigger(response);
      });

    RCLCPP_INFO(this->get_logger(), "Pointcloud aggregator node initialized.");
  }

private:
  rclcpp::Subscription<CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_shutdown_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aggregated_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;

  std::atomic<bool> lidar_online_;
  bool collecting_ = false;

  size_t messages_to_accumulate_ = 10;
  size_t lidar_msgs_ = 0;
  float downsample_leaf_size_ = 0.05f;
  float min_dist_ = 1.0f;
  float max_dist_ = 10.0f;
  float min_dist_sq_ = 0.0f;
  float max_dist_sq_ = 10000.0f;
  size_t points_received_ = 0;
  size_t points_in_range_ = 0;
  std::string lidar_topic_;
  std::string aggregated_topic_;
  std::string trigger_service_;
  std::string output_frame_id_;

  pcl::PointCloud<pcl::PointXYZ> lidar_cloud_;

  void on_lidar(const CustomMsg::SharedPtr msg) {
    if (!lidar_online_.exchange(true)) {
      RCLCPP_INFO(this->get_logger(), "LiDAR online on topic: %s", lidar_topic_.c_str());
    }

    if (!collecting_) {
      return;
    }

    points_received_ += msg->points.size();
    for (const auto & p : msg->points) {
      const float dist_sq = (p.x * p.x) + (p.y * p.y) + (p.z * p.z);
      if (dist_sq >= min_dist_sq_ && dist_sq <= max_dist_sq_) {
        lidar_cloud_.push_back(pcl::PointXYZ{p.x, p.y, p.z});
        ++points_in_range_;
      }
    }

    ++lidar_msgs_;
    if (lidar_msgs_ >= messages_to_accumulate_) {
      publish_aggregated_cloud();
    }
  }

  void handle_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) {
    if (!lidar_online_) {
      response->success = false;
      response->message = "LiDAR topic has not produced data yet.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (collecting_) {
      response->success = false;
      response->message = "Accumulation already in progress.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    lidar_cloud_.clear();
    lidar_msgs_ = 0;
    points_received_ = 0;
    points_in_range_ = 0;
    collecting_ = true;

    response->success = true;
    response->message = "Accumulation started.";
    RCLCPP_INFO(this->get_logger(), "%s (%zu messages)", response->message.c_str(), messages_to_accumulate_);
  }

  void publish_aggregated_cloud() {
    collecting_ = false;

    if (lidar_cloud_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Accumulated cloud is empty; skipping publish.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> downsampled;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(lidar_cloud_.makeShared());
    vg.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
    vg.filter(downsampled);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(downsampled, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = output_frame_id_;
    pub_aggregated_->publish(cloud_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "Published aggregated cloud on %s: %zu input points, %zu within [%.2f, %.2f] m -> %zu downsampled points",
      aggregated_topic_.c_str(),
      points_received_,
      points_in_range_,
      min_dist_,
      max_dist_,
      downsampled.size());

    // Delay shutdown so gather_aggregated_clouds has time to receive and save
    // the cloud over the network before this node's transient_local publisher dies.
    shutdown_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      [this]() { rclcpp::shutdown(); });
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointcloudAggregatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
