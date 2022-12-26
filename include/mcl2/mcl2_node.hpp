#ifndef MCL2__MCL2_NODE_HPP_
#define MCL2__MCL2_NODE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mcl2
{
class Mcl2Node : public rclcpp::Node
{
public:
  explicit Mcl2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Mcl2Node();

private:
  // パブリッシャ・サブスクライバ初期化用
  void initPubSub();

  // サブスクライバの登録
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr scan_sub_;

  // パブリッシャの登録
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr likelihood_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marginal_likelihood_publisher_;
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
    maximum_likelihood_particles_publisher_;

  void receiveInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void receiveMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void receiveScan(sensor_msgs::msg::LaserScan::SharedPtr msg);
};
}  // namespace mcl2

#endif  // MCL2__MCL2_NODE_HPP_