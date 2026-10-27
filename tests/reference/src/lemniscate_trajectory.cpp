#include "px4_reference_generator/lemniscate_trajectory.hpp"

#include <cmath>

namespace px4_reference_generator
{

LemniscateTrajectory::LemniscateTrajectory(
  double a, double omega, double z_ned, double center_x, double center_y,
  double z_amp, double z_omega)
: a_(a), omega_(omega), z_ned_(z_ned), center_x_(center_x), center_y_(center_y),
  z_amp_(z_amp), z_omega_(z_omega)
{
}

TrajectorySample LemniscateTrajectory::sample(double t) const
{
  TrajectorySample out;
  const double theta = omega_ * t - M_PI_2;
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  const double s2 = s * s;
  const double c2 = c * c;

  const double D = 1.0 + s2;
  const double D2 = D * D;
  const double D3 = D2 * D;

  out.x = center_x_ + a_ * c / D;
  out.y = center_y_ + a_ * s * c / D;
  out.z = z_ned_ + z_amp_ * std::sin(z_omega_ * t);
  out.vx = -a_ * omega_ * s * (2.0 + c2) / D2;
  out.vy = a_ * omega_ * (c2 - 2.0 * s2) / D2;
  out.vz = z_amp_ * z_omega_ * std::cos(z_omega_ * t);
  out.ax = -a_ * omega_ * omega_ * c * (s2 * s2 - 12.0 * s2 + 3.0) / D3;
  out.ay = a_ * omega_ * omega_ * s * c * (6.0 * s2 - 10.0) / D3;
  out.az = -z_amp_ * z_omega_ * z_omega_ * std::sin(z_omega_ * t);
  return out;
}

TrajectorySample hover_sample(const HoverTarget & target)
{
  TrajectorySample out;
  out.x = target.x;
  out.y = target.y;
  out.z = target.z_ned;
  return out;
}

TrajectorySample circle_sample(
  double radius, double omega, double z_ned, double anchor_x, double anchor_y,
  double z_amp, double z_omega, double t)
{
  TrajectorySample out;
  const double th = omega * t - M_PI_2;
  const double c = std::cos(th);
  const double s = std::sin(th);
  // centre offset so that t=0 sits on the anchor
  const double cx = anchor_x;
  const double cy = anchor_y + radius;
  out.x = cx + radius * c;
  out.y = cy + radius * s;
  out.z = z_ned + z_amp * std::sin(z_omega * t);
  out.vx = -radius * omega * s;
  out.vy = radius * omega * c;
  out.vz = z_amp * z_omega * std::cos(z_omega * t);
  out.ax = -radius * omega * omega * c;
  out.ay = -radius * omega * omega * s;
  out.az = -z_amp * z_omega * z_omega * std::sin(z_omega * t);
  return out;
}

}  // namespace px4_reference_generator
