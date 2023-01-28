#pragma once
#include "depth_image_to_mavlink/common.h"

namespace depth_image_to_mavlink {

struct candidateDirection {
  float cost;
  float elevation_angle;
  float azimuth_angle;

  candidateDirection(float c, float e, float z) : cost(c), elevation_angle(e), azimuth_angle(z){};

  bool operator<(const candidateDirection& y) const { return cost < y.cost; }

  bool operator>(const candidateDirection& y) const { return cost > y.cost; }

  PolarPoint toPolar(float r) const { return PolarPoint(elevation_angle, azimuth_angle, r); }
};
}
