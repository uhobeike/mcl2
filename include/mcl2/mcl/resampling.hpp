// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

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

  void resampling(std::vector<Particle> & particles);  // リサンプリングをする

  void systematicSampling(std::vector<Particle> & particles);  // 系統サンプリングをする
  double calculateSystematicSamplingStep(
    double particles_weight_median);  // 系統サンプリング用のstepを計算する
  void normalize(std::vector<Particle> & particles);  // パーティクルの重みを正規化をする

  std::random_device seed_gen_;
  std::mt19937 engine_;

  long unsigned int particle_size_;
};
}  // namespace mcl

#endif