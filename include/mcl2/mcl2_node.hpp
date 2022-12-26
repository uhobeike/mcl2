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

protected:
};
}  // namespace mcl2

#endif  // MCL2__MCL2_NODE_HPP_