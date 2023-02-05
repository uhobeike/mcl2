#include "mcl2/mcl/mcl.hpp"

namespace mcl
{
Mcl::Mcl(
  double ini_pose_x, double ini_pose_y, double ini_pose_yaw, double alpha_trans_trans,
  double alpha_trans_rotate, double alpha_rotate_trans, double alpha_rotate_rotate,
  int particle_size, double likelihood_dist)
{
  initParticles(ini_pose_x, ini_pose_y, ini_pose_yaw, particle_size);

  release_pointers();

  likelihood_field_ = std::make_shared<LikelihoodField>(likelihood_dist);
  motion_model_update_ = std::make_shared<MotionModelUpdate>();
  observation_model_update_ = std::make_shared<ObservationModelUpdate>();
  resampling_ = std::make_shared<Resampling>();
}

Mcl::~Mcl() { release_pointers(); }

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

void Mcl::release_pointers()
{
  likelihood_field_.reset();
  motion_model_update_.reset();
  observation_model_update_.reset();
  resampling_.reset();
}

}  // namespace mcl