#pragma once

namespace px4_reference_generator
{

struct TrajectorySample
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double ax{0.0};
  double ay{0.0};
  double az{0.0};
};

struct HoverTarget
{
  double x{0.0};
  double y{0.0};
  double z_ned{ -5.0};
};

}  // namespace px4_reference_generator
