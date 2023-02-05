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