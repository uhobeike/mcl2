#include <memory>
#include <vector>

#include "mcl2/mcl/likelihoodField.hpp"
#include "mcl2/mcl/motionModel.hpp"
#include "mcl2/mcl/observationModel.hpp"
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
    int particle_size, double likelihood_dist, uint32_t map_width, uint32_t map_height,
    double map_resolution, double map_origin_x, double map_origin_y, std::vector<int8_t> map_data,
    float scan_angle_min, float scan_angle_max, float scan_angle_increment, float scan_range_min,
    float scan_range_max);
  ~Mcl();

  void release_pointers();

  std::unique_ptr<LikelihoodField> likelihood_field_;
  std::unique_ptr<MotionModel> motion_model_;
  std::unique_ptr<ObservationModel> observation_model_;
  std::unique_ptr<Resampling> resampling_;

  void initParticles(double ini_pose_x, double ini_pose_y, double ini_pose_yaw, int particle_size);

  std::vector<Particle> particles_;
};
}  // namespace mcl