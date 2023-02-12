#ifndef MCL2__RESAMPLING_HPP_
#define MCL2__RESAMPLING_HPP_

#include <random>
#include <vector>

#include "mcl2/mcl/particle.hpp"

namespace mcl
{
class Resampling
{
public:
  Resampling(int particle_size_);
  ~Resampling();

  void resampling(std::vector<Particle> & particles);

  void systematicSampling(std::vector<Particle> & particles);
  double calculateSystematicSamplingStep(double particles_weight_median);
  void normalize(std::vector<Particle> & particles);

  std::random_device seed_gen_;
  std::mt19937 engine_;

  int particle_size_;
};
}  // namespace mcl

#endif