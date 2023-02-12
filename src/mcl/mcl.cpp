#include "mcl2/mcl/mcl.hpp"

namespace mcl
{
Mcl::Mcl(
  double ini_pose_x, double ini_pose_y, double ini_pose_yaw, double alpha_trans_trans,
  double alpha_trans_rotate, double alpha_rotate_trans, double alpha_rotate_rotate,
  int particle_size, double likelihood_dist, uint32_t map_width, uint32_t map_height,
  double map_resolution, double map_origin_x, double map_origin_y, std::vector<int8_t> map_data,
  float scan_angle_min, float scan_angle_max, float scan_angle_increment, float scan_range_min,
  float scan_range_max)
{
  initParticles(ini_pose_x, ini_pose_y, ini_pose_yaw, particle_size);

  release_pointers();

  likelihood_field_ = std::make_unique<LikelihoodField>(
    likelihood_dist, map_width, map_height, map_resolution, map_origin_x, map_origin_y, map_data);
  motion_model_ = std::make_unique<MotionModel>(
    alpha_trans_trans, alpha_trans_rotate, alpha_rotate_trans, alpha_rotate_rotate);
  observation_model_ = std::make_unique<ObservationModel>(
    std::move(likelihood_field_), scan_angle_min, scan_angle_max, scan_angle_increment,
    scan_range_min, scan_range_max);
  resampling_ = std::make_unique<Resampling>();
}

Mcl::~Mcl() { release_pointers(); }

void Mcl::initParticles(
  double ini_pose_x, double ini_pose_y, double ini_pose_yaw, int particle_size)
{
  Particle p;
  p.pose.position.x = ini_pose_x;
  p.pose.position.y = ini_pose_y;
  p.pose.euler.yaw = ini_pose_yaw;

  particles_.clear();
  particles_.resize(particle_size);

  for (auto i = 0; i < particle_size; i++) {
    particles_[i] = p;
    particles_[i].weight = 1. / particle_size;
  }
}

void Mcl::release_pointers()
{
  likelihood_field_.reset();
  motion_model_.reset();
  observation_model_.reset();
  resampling_.reset();
}

}  // namespace mcl