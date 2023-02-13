// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

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
    double delta_yaw);                                // 動作モデルの更新
  double drawNoise(double sigma);                     // ノイズを一様分布からドロー
  double diffMinAngle(double angle1, double angle2);  // 小さい回転の方を渡す
  void getDelta(
    double & delta_x, double & delta_y, double & delta_yaw, double current_x, double past_x,
    double current_y, double past_y, double current_yaw,
    double past_yaw);  // 現在の姿勢と現在の姿勢を比較したときの各差分を計算する
  inline double normalizeAngle(double yaw) { return atan2(sin(yaw), cos(yaw)); }
  inline void normalizeAngle(double & yaw, bool no_return)
  {
    yaw = atan2(sin(yaw), cos(yaw));
  }  // 回転の正規化を行う（θ ∈ [-π,π)）

  double alpha_trans_trans_, alpha_trans_rotate_, alpha_rotate_trans_,
    alpha_rotate_rotate_;  // 動作モデル用の誤差

  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace mcl

#endif