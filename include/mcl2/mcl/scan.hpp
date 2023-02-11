#ifndef MCL2__SCAN_HPP_
#define MCL2__SCAN_HPP_

#include <iostream>
#include <vector>

struct Scan
{
  float angle_min;
  float angle_max;
  float angle_increment;
  float range_min;
  float range_max;
  std::vector<float> ranges;
};

#endif  // MCL2__SCAN_HPP_