#include "mcl2/mcl/mcl.hpp"

#include <iostream>

namespace mcl
{
Mcl::Mcl(
  double ini_pose_x, double ini_pose_y, double ini_pose_yaw, double alpha_trans_trans,
  double alpha_trans_rotate, double alpha_rotate_trans, double alpha_rotate_rotate,
  int particle_size, double likelihood_dist)
{
  initParticles(ini_pose_x, ini_pose_y, ini_pose_yaw, particle_size);
  std::cout << particles_.size() << "\n";
}

Mcl::~Mcl() {}

void Mcl::initParticles(
  double ini_pose_x, double ini_pose_y, double ini_pose_yaw, int particle_size)
{
  Particle p;
  p.pose.position.x = ini_pose_x;
  p.pose.position.y = ini_pose_y;
  p.pose.euler.yaw = ini_pose_yaw;

  particles_.resize(particle_size);

  for (auto i = 0; i < particle_size; i++) {
    particles_[i] = p;
  }
}

}  // namespace mcl