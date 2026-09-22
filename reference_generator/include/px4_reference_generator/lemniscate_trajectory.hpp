#pragma once

#include "px4_reference_generator/trajectory_sample.hpp"

namespace px4_reference_generator
{

/** Bernoulli lemniscate (figure-8) with -pi/2 phase so t=0 passes through center.
 *  x = a cos th / (1 + sin^2 th), y = a sin th cos th / (1 + sin^2 th), th = omega t - pi/2;
 *  satisfies (x^2+y^2)^2 = a^2 (x^2-y^2). Verified numerically to 8.5e-14 (2026-07-26).
 *  Curvature kappa = 3 rho / a^2, maximal at the tips (rho = a): kappa_max = 3/a. */
class LemniscateTrajectory
{
public:
  LemniscateTrajectory(
    double a, double omega, double z_ned, double center_x, double center_y,
    double z_amp = 0.0, double z_omega = 0.0);

  TrajectorySample sample(double t) const;

  double a() const { return a_; }
  double omega() const { return omega_; }

private:
  double a_;
  double omega_;
  double z_ned_;
  double center_x_;
  double center_y_;
  double z_amp_{0.0};
  double z_omega_{0.0};
};

TrajectorySample hover_sample(const HoverTarget & target);

/** Constant-curvature (circular) reference. The circle is placed so that t=0 lies on
 *  the anchor, matching the lemniscate's entry condition; radius is the INSCRIBED
 *  line radius when used for the certified-corridor experiment. */
TrajectorySample circle_sample(
  double radius, double omega, double z_ned, double anchor_x, double anchor_y,
  double z_amp, double z_omega, double t);

}  // namespace px4_reference_generator
