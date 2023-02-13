// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

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
    std::shared_ptr<mcl::LikelihoodField> likelihood_field, float angle_min, float angle_max,
    float angle_increment, float range_min, float range_max);
  ~ObservationModel();

  void initScan(
    float angle_min, float angle_max, float angle_increment, float range_min,
    float range_max);  // スキャンに関するパラメータをセットする
  void setScan(std::vector<float> & scan_data);  // ROS 2のスキャンをMClのスキャンとしてセットする

  void update(std::vector<Particle> & particles, std::vector<float> scan_data);  // 観測モデルの更新
  double calculateParticleWeight(const Particle p);  // パーティクルの重みを計算する
  double getProbFromLikelihoodMap(double x, double y);  // 尤度場から確率を取得する

  inline double getRadian(double degree) { return degree * M_PI / 180; }  // ラジアンに変換する

  std::shared_ptr<mcl::LikelihoodField> likelihood_field_;
  Scan scan_;
};
}  // namespace mcl

#endif