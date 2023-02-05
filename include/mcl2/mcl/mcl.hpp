#include <memory>
#include <vector>

#include "mcl2/mcl/likelihoodField.hpp"
#include "mcl2/mcl/motionModelUpdate.hpp"
#include "mcl2/mcl/observationModelUpdate.hpp"
#include "mcl2/mcl/particle.hpp"
#include "mcl2/mcl/resampling.hpp"

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

  void release_pointers();

  std::shared_ptr<LikelihoodField> likelihood_field_;
  std::shared_ptr<MotionModelUpdate> motion_model_update_;
  std::shared_ptr<ObservationModelUpdate> observation_model_update_;
  std::shared_ptr<Resampling> resampling_;

  void initParticles(double ini_pose_x, double ini_pose_y, double ini_pose_yaw, int particle_size);

  std::vector<Particle> particles_;
};
}  // namespace mcl