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
#include "nav_msgs/srv/get_map.hpp"
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

Mcl2Node::Mcl2Node(const rclcpp::NodeOptions & options)
: Node("mcl2_node", options), ros_clock_(RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "Run Mcl2Node");
  initPubSub();
}
Mcl2Node::~Mcl2Node() {}

void Mcl2Node::initPubSub()
{
  RCLCPP_INFO(get_logger(), "Run initPubSub");

  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", 2);
  likelihood_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("likelihood_map", 2);
  marker_array_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("mcl_match", 2);
  marginal_likelihood_publisher_ =
    create_publisher<std_msgs::msg::String>("marginal_likelihood", 2);
  maximum_likelihood_particles_publisher_ =
    create_publisher<nav2_msgs::msg::ParticleCloud>("maximum_likelihood_particles", 2);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&Mcl2Node::receiveMap, this, std::placeholders::_1));
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&Mcl2Node::receiveInitialPose, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 1, std::bind(&Mcl2Node::receiveScan, this, std::placeholders::_1));
}

void Mcl2Node::receiveInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Run receiveInitialPose");

  if (not initialpose_receive_) {
    initialpose_receive_ = true;

    initTf();
    initMcl(msg);
    loopMcl();
    // getOdom2RobotPose(current_pose_);
    // old_pose_ = current_pose_;

  } else {
    mcl_->initParticles(
      msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation),
      particle_size_);
  }
};

void Mcl2Node::receiveScan(sensor_msgs::msg::LaserScan::SharedPtr msg){};
void Mcl2Node::receiveMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = *msg; }

void Mcl2Node::initTf()
{
  RCLCPP_INFO(get_logger(), "Run initTf");

  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface(),
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  latest_tf_ = tf2::Transform::getIdentity();
}

void Mcl2Node::initMcl(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
  RCLCPP_INFO(get_logger(), "Run initMcl");

  alpha1_ = 0.2;
  alpha2_ = 0.2;
  alpha3_ = 0.2;
  alpha4_ = 0.03;

  particle_size_ = 500;

  likelihood_dist_ = 20.0;

  mcl_.reset();
  mcl_ = std::make_shared<mcl::Mcl>(
    pose->pose.pose.position.x, pose->pose.pose.position.y, 0.0, alpha1_, alpha2_, alpha3_, alpha4_,
    particle_size_, likelihood_dist_, map_.info.width, map_.info.height, map_.info.resolution,
    map_.data);

  mcl_->likelihood_field_->getLikelihoodField(map_.data);
  likelihood_map_pub_->publish(map_);
}

void Mcl2Node::mcl_to_ros2()
{
  nav2_msgs::msg::ParticleCloud particles;
  setParticles(particles);
  publishParticles(particles);
}

void Mcl2Node::setParticles(nav2_msgs::msg::ParticleCloud & particles)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.nanosec = ros_clock_.now().nanoseconds();
  particles.set__header(header);

  particles.particles.resize(mcl_->particles_.size());
  for (auto i = 0; i < mcl_->particles_.size(); ++i) {
    particles.particles[i].pose.position.x = mcl_->particles_[i].pose.position.x;
    particles.particles[i].pose.position.y = mcl_->particles_[i].pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, mcl_->particles_[i].pose.euler.yaw);
    particles.particles[i].pose.orientation = tf2::toMsg(q);

    particles.particles[i].weight = mcl_->particles_[i].weight;
  }
}

void Mcl2Node::loopMcl()
{
  rclcpp::WallRate loop_rate(100ms);
  while (rclcpp::ok() && initialpose_receive_) {
    RCLCPP_INFO(get_logger(), "Run loop_mcl");
    mcl_->motion_model_->update(mcl_->particles_, 0, 0.001, 0.001, 0);
    mcl_->observation_model_;
    mcl_->resampling_;
    mcl_to_ros2();
    loop_rate.sleep();
  }
}
}  // namespace mcl2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mcl2::Mcl2Node)
