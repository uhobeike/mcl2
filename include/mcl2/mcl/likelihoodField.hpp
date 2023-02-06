#ifndef MCL2__LIKELIHOODFIELD_HPP_
#define MCL2__LIKELIHOODFIELD_HPP_

#include <iostream>
#include <vector>

namespace mcl
{
class LikelihoodField
{
public:
  LikelihoodField(
    double likelihood_dist, uint32_t width, uint32_t height, double resolution,
    std::vector<int8_t> data);
  ~LikelihoodField();

  void createLikelihoodField();
  void calculateLikelihood(uint32_t map_x, uint32_t map_y);
  double calculatePdf(double stochastic_variable, double likelihood_dist);
  double normalizePdf(double max_pdf, double pdf);

  bool getLikelihoodField(std::vector<int8_t> & data);

  double likelihood_dist_;
  uint32_t width_;
  uint32_t height_;
  double resolution_;
  std::vector<int8_t> data_;

  bool create_likelihood_field_;
};
}  // namespace mcl

#endif