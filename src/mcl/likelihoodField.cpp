// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "mcl2/mcl/likelihoodField.hpp"

#include <cmath>

namespace mcl
{
LikelihoodField::LikelihoodField(
  double likelihood_dist, uint32_t width, uint32_t height, double resolution, double origin_x,
  double origin_y, std::vector<int8_t> data)
: likelihood_dist_(likelihood_dist),
  width_(width),
  height_(height),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  data_(data)
{
  std::cout << "Create LikelihoodField."
            << "\n";

  createLikelihoodField();

  std::cout << "Done Create LikelihoodField."
            << "\n";
};
LikelihoodField::~LikelihoodField(){};

void LikelihoodField::createLikelihoodField()
{
  for (uint32_t y = 0; y < width_; y++)
    for (uint32_t x = 0; x < height_; x++)
      if (data_[width_ * (height_ - y - 1) + x] == 100) {
        calculateLikelihood(x, y);
      }
}

void LikelihoodField::calculateLikelihood(uint32_t map_x, uint32_t map_y)
{
  int sub_map_x_start, sub_map_y_start, sub_map_x_end, sub_map_y_end;
  sub_map_x_start = map_x - likelihood_dist_;
  sub_map_y_start = map_y - likelihood_dist_;
  sub_map_x_end = map_x + likelihood_dist_;
  sub_map_y_end = map_y + likelihood_dist_;

  for (auto y = sub_map_y_start - likelihood_dist_; y < sub_map_y_end; y++)
    for (auto x = sub_map_x_start - likelihood_dist_; x < sub_map_x_end; x++)
      if (hypot(x - map_x, y - map_y) < likelihood_dist_)
        if (
          normalizePdf(
            calculateProb(0, likelihood_dist_),
            calculateProb(hypot(x - map_x, y - map_y), likelihood_dist_)) >
          data_[width_ * (height_ - y - 1) + x])
          data_[width_ * (height_ - y - 1) + x] = normalizePdf(
            calculateProb(0, likelihood_dist_),
            calculateProb(hypot(x - map_x, y - map_y), likelihood_dist_));
};

double LikelihoodField::calculateProb(double stochastic_variable, double likelihood_dist)
{
  double sigma = likelihood_dist / 3;
  double pdf = 1. / std::sqrt(2. * M_PI * sigma * sigma) *
               std::exp(-stochastic_variable * stochastic_variable / (2 * sigma * sigma));
  return pdf;
}

double LikelihoodField::normalizePdf(double max_pdf, double pdf) { return (pdf / max_pdf) * 100; }

void LikelihoodField::getLikelihoodField(std::vector<int8_t> & data) { data = data_; }

}  // namespace mcl