#include "mcl2/mcl2_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/string_utils.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace mcl2
{

Mcl2Node::Mcl2Node(const rclcpp::NodeOptions & options) : Node("mcl2_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Start Mcl2Node");
}
Mcl2Node::~Mcl2Node() {}

void Mcl2Node::initPubSub()
{
  RCLCPP_INFO(get_logger(), "initPubSub");

  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", 2);

  likelihood_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("likelihood_map", 2);

  marker_array_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("mcl_match", 2);

  marginal_likelihood_publisher_ =
    create_publisher<std_msgs::msg::String>("marginal_likelihood", 2);

  maximum_likelihood_particles_publisher_ =
    create_publisher<nav2_msgs::msg::ParticleCloud>("maximum_likelihood_particles", 2);

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&Mcl2Node::receiveInitialPose, this, std::placeholders::_1));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 1, std::bind(&Mcl2Node::receiveMap, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 1, std::bind(&Mcl2Node::receiveScan, this, std::placeholders::_1));
}

void Mcl2Node::receiveInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){};
void Mcl2Node::receiveMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg){};
void Mcl2Node::receiveScan(sensor_msgs::msg::LaserScan::SharedPtr msg){};

}  // namespace mcl2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mcl2::Mcl2Node)
