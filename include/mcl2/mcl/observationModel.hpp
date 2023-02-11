#ifndef MCL2__OBSERVATIONMODEL_HPP_
#define MCL2__OBSERVATIONMODEL_HPP_

#include <cmath>
#include <memory>

#include "mcl2/mcl/likelihoodField.hpp"
#include "mcl2/mcl/particle.hpp"
#include "mcl2/mcl/scan.hpp"

namespace mcl
{
class ObservationModel
{
public:
  ObservationModel(
    std::unique_ptr<mcl::LikelihoodField> likelihood_field, float angle_min, float angle_max,
    float angle_increment, float range_min, float range_max);
  ~ObservationModel();

  void initScan(
    float angle_min, float angle_max, float angle_increment, float range_min, float range_max);
  void setScan(std::vector<float> & scan_data);

  void update(std::vector<Particle> & particles, std::vector<float> scan_data);
  double calculateParticleWeight(const Particle p);
  double getProbFromLikelihoodMap(double x, double y);

  inline double getRadian(double degree) { return degree * M_PI / 180; }

  std::unique_ptr<mcl::LikelihoodField> likelihood_field_;
  Scan scan_;
};
}  // namespace mcl

#endif