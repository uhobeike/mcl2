#include <vector>

#include "mcl2/mcl/particle.hpp"

namespace mcl
{
class Mcl
{
public:
  Mcl(
    double ini_pose_x, double ini_pose_y, double ini_pose_yaw, double alpha_trans_trans,
    double alpha_trans_rotate, double alpha_rotate_trans, double alpha_rotate_rotate,
    int particle_size, double likelihood_dist);
  ~Mcl();

  void initParticles(double ini_pose_x, double ini_pose_y, double ini_pose_yaw, int particle_size);

  std::vector<Particle> particles_;
};
}  // namespace mcl