// SPDX-FileCopyrightText: 2023 Tatuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef MCL2__PARTICLE_HPP_
#define MCL2__PARTICLE_HPP_

struct Euler
{
  double yaw;
};

struct Point
{
  double x;
  double y;
};

struct Pose
{
  Point position;
  Euler euler;
};

struct Particle
{
  Pose pose;

  double weight;
};

#endif  // MCL2__PARTICLE_HPP_