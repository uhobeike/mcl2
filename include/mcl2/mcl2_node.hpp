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
#include "mcl2/mcl/mcl.hpp"
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

  void receiveInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);  //初期位置の受取
  void receiveMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // 尤度場作成用のマップの受取
  void receiveScan(sensor_msgs::msg::LaserScan::SharedPtr msg);  // LiDARからのデータの受取

  void initPubSub();  // パブリッシャ・サブスクライバ初期化用
  void initTf();      //tf関連の初期化
  void initMcl(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);  //Mclの初期化
  void loopMcl();                                                               //Mclのループ

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform latest_tf_;

  nav2_msgs::msg::ParticleCloud pc_;  //パーティクル群
  bool initialpose_receive_;          //初期位置を受け取ったかのフラグ

  // Mcl2用のパラメータ
  double alpha1_, alpha2_, alpha3_, alpha4_;  //動作モデル用の誤差パラメータ
  double particle_size_;
  double likelihood_dist_;

  std::shared_ptr<mcl::Mcl> mcl_;
};
}  // namespace mcl2

#endif  // MCL2__MCL2_NODE_HPP_