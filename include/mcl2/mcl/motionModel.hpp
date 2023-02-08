#ifndef MCL2__MOTIONMODEL_HPP_
#define MCL2__MOTIONMODEL_HPP_

#include <random>
#include <vector>

#include "mcl2/mcl/particle.hpp"

namespace mcl
{
class MotionModel
{
public:
  MotionModel(
    double alpha_trans_trans, double alpha_trans_rotate, double alpha_rotate_trans,
    double alpha_rotate_rotate);
  ~MotionModel();

  void update(
    std::vector<Particle> & particles, double current_pose_yaw, double delta_x, double delta_y,
    double delta_yaw);
  double sumpleNoise(double sigma);
  double diffMinAngle(double angle1, double angle2);
  inline double normalizeAngle(double yaw) { return atan2(sin(yaw), cos(yaw)); }
  inline void normalizeAngle(double & yaw, bool no_return) { yaw = atan2(sin(yaw), cos(yaw)); }

  double alpha_trans_trans_, alpha_trans_rotate_, alpha_rotate_trans_, alpha_rotate_rotate_;

  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace mcl

#endif