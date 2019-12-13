/*
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Dense>

namespace utils
{

/** Polynomial function.
 *
 */
template<typename T>
struct PolynomialBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Get the value of the polynomial at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  virtual T pos(double t) const = 0;

  /** Get the value of the first-order derivative (velocity) at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  virtual T vel(double t) const = 0;

  /** Get the value of the second-order derivative (acceleration) at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  virtual T accel(double t) const = 0;

  /** Get the value of the tangent vector at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  T tangent(double t) const
  {
    T v = this->vel(t);
    return v / v.norm();
  }

  /** Compute the arc length between two points of the polynomial curve.
   *
   * \param t_start Index of start point.
   *
   * \param t_end Index of end point.
   *
   * \returns length Arc length of the curve between ``P(t_start)`` and
   * ``P(t_end)``.
   *
   * \note This function uses 5-point Gauss-Legendre quadrature [1]. It is
   * adapted from the Unreal Engine's ``GetSplineLength`` [2] pointed out in
   * ``jj``'s post on Medium [3].
   *
   * [1] https://en.wikipedia.org/wiki/Gaussian_quadrature
   * [2] https://api.unrealengine.com/INT/BlueprintAPI/Spline/GetSplineLength/
   * [3] https://medium.com/@all2one/how-to-compute-the-length-of-a-spline-e44f5f04c40
   *
   */
  double arcLength(double t_start, double t_end) const
  {
    struct GaussLengendreCoefficient
    {
      double t; /**< value of polynomial variable */
      double w; /**< corresponding Gauss-Legendre weight */
    };
    static constexpr GaussLengendreCoefficient coeffs[] = {{0.0, 128. / 225.},
                                                           {-0.53846931010568311, 0.47862867049936647},
                                                           {+0.53846931010568311, 0.47862867049936647},
                                                           {-0.90617984593866396, 0.23692688505618908},
                                                           {+0.90617984593866396, 0.23692688505618908}};
    double a = (t_end - t_start) / 2.;
    double b = (t_end + t_start) / 2.;
    double length = 0.;
    for(auto coeff : coeffs)
    {
      length += coeff.w * this->vel(a * coeff.t + b).norm();
    }
    return a * length;
  }

  /** Inverse of the arc length function.
   *
   * \param t_start Start index on polynomial curve.
   *
   * \param length Desired arc length.
   *
   * \param t_guess Initial guess for the value of the desired output
   * (optional).
   *
   * \returns t Value of the polynomial argument such that
   * ``arcLength(t_start, t) == length``.
   *
   */
  double arcLengthInverse(double t_start, double length, double t_guess = -1.) const
  {
    constexpr double PRECISION = 1e-3;
    if(t_guess < 0.)
    {
      t_guess = t_start + 1.;
    }
    double l_guess = arcLength(t_start, t_guess);
    if(std::abs(l_guess - length) < PRECISION)
    {
      return t_guess;
    }
    else if(l_guess < length)
    {
      return arcLengthInverse(t_guess, length - l_guess, 2. * t_guess - t_start);
    }
    else // (l_guess > length)
    {
      return arcLengthInverse(t_start, length, (t_guess + t_start) / 2.);
    }
  }
};

/** Cubic polynomial curve.
 *
 */
template<typename T>
struct CubicPolynomialBase : PolynomialBase<T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Build a new curve from its monomial vector coefficients.
   *
   * \param C0 Zero-order coefficient.
   *
   * \param C1 First-order coefficient.
   *
   * \param C2 Second-order coefficient.
   *
   * \param C3 Third-order coefficient.
   *
   */
  CubicPolynomialBase(const T & C0, const T & C1, const T & C2, const T & C3) : C0_(C0), C1_(C1), C2_(C2), C3_(C3) {}

  /** Get the value of the polynomial at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  T pos(double t) const
  {
    return C0_ + t * (C1_ + t * (C2_ + t * C3_));
  }

  /** Get the value of the first-order derivative (velocity) at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  T vel(double t) const
  {
    return C1_ + t * (2 * C2_ + 3 * t * C3_);
  }

  /** Get the value of the second-order derivative (acceleration) at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  T accel(double t) const
  {
    return 2 * C2_ + 6 * t * C3_;
  }

protected:
  T C0_;
  T C1_;
  T C2_;
  T C3_;
};

template<typename T>
struct CubicPolynomial : CubicPolynomialBase<T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Return T's zero.
   *
   */
  T zero()
  {
    return T::Zero();
  }

  /** Empty constructor.
   *
   */
  CubicPolynomial() : CubicPolynomialBase<T>(T::Zero(), T::Zero(), T::Zero(), T::Zero()) {}
};

template<>
struct CubicPolynomial<double> : CubicPolynomialBase<double>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Zero function used for double specialization.
   *
   */
  double zero()
  {
    return 0.;
  }

  /** Empty constructor.
   *
   */
  CubicPolynomial() : CubicPolynomialBase<double>(0., 0., 0., 0.) {}
};

/** Cubic Hermite polynomial.
 *
 */
template<typename T>
struct CubicHermitePolynomial : public CubicPolynomial<T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Empty constructor.
   *
   */
  CubicHermitePolynomial()
  {
    CubicHermitePolynomial(this->zero(), this->zero(), this->zero(), this->zero());
  }

  /** Build a new cubic Hermite polynomial.
   *
   * \param initPos Position at t=0.
   *
   * \param initVel Velocity at t=0.
   *
   * \param targetPos Position at t=1.
   *
   * \param targetVel Velocity at t=1.
   *
   */
  CubicHermitePolynomial(const T & initPos, const T & initVel, const T & targetPos, const T & targetVel)
  {
    reset(initPos, initVel, targetPos, targetVel);
  }

  /** Reset boundaries.
   *
   * \param initPos Initial position.
   *
   * \param initVel Initial tangent vector.
   *
   * \param targetPos Target position.
   *
   * \param targetVel Target tangent vector.
   *
   */
  void reset(const T & initPos, const T & initVel, const T & targetPos, const T & targetVel)
  {
    initPos_ = initPos;
    initVel_ = initVel;
    targetPos_ = targetPos;
    targetVel_ = targetVel;
    reset();
  }

  /** Reset boundaries with zero tangents.
   *
   * \param initPos Position at t=0.
   *
   * \param targetPos Position at t=1.
   *
   */
  void reset(const T & initPos, const T & targetPos)
  {
    initPos_ = initPos;
    initVel_ = this->zero();
    targetPos_ = targetPos;
    targetVel_ = this->zero();
    reset();
  }

  /** Reset underlying cubic polynomial coefficients.
   *
   */
  virtual void reset()
  {
    this->C0_ = initPos_;
    this->C1_ = initVel_;
    this->C2_ = 3. * (targetPos_ - initPos_) - 2. * initVel_ - targetVel_;
    this->C3_ = -2. * (targetPos_ - initPos_) + initVel_ + targetVel_;
  }

protected:
  T initPos_;
  T initVel_;
  T targetPos_;
  T targetVel_;
};

/** Hermite polynomial with Overall Uniformly-Bounded Accelerations (HOUBA).
 *
 * \note See <https://hal.archives-ouvertes.fr/hal-01363757/document> for
 * details on the definition and derivation of these polynomials.
 *
 */
template<typename T>
struct HoubaPolynomial : CubicHermitePolynomial<T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Bring overloads down from parent class (otherwise C++ doesn't go up the
   * inheritance tree to find them).
   *
   */
  using CubicHermitePolynomial<T>::reset;

  /** Rescale boundary velocities, then reset as Hermite polynomial.
   *
   */
  void reset() override
  {
    T Delta = this->targetPos_ - this->initPos_;
    double Delta_v0 = Delta.dot(this->initVel_);
    double Delta_v1 = Delta.dot(this->targetVel_);
    double v0_v0 = this->initVel_.dot(this->initVel_);
    double v0_v1 = this->initVel_.dot(this->targetVel_);
    double v1_v1 = this->targetVel_.dot(this->targetVel_);
    double houbaInitVelScaling =
        6. * (3. * Delta_v0 * v1_v1 - 2. * Delta_v1 * v0_v1) / (9. * v0_v0 * v1_v1 - 4. * v0_v1 * v0_v1);
    if(houbaInitVelScaling < 0.)
    {
      houbaInitVelScaling *= -1.;
    }
    double houbaTargetVelScaling =
        6. * (-2. * Delta_v0 * v0_v1 + 3. * Delta_v1 * v0_v0) / (9. * v0_v0 * v1_v1 - 4. * v0_v1 * v0_v1);
    if(houbaTargetVelScaling < 0.)
    {
      houbaTargetVelScaling *= -1.;
    }
    this->initVel_ *= houbaInitVelScaling;
    this->initVel_ *= extraInitVelScaling_;
    this->targetVel_ *= houbaTargetVelScaling;
    this->targetVel_ *= extraTargetVelScaling_;
    CubicHermitePolynomial<T>::reset();
  }

  /** Get extra initial velocity scaling.
   *
   */
  double extraInitVelScaling()
  {
    return extraInitVelScaling_;
  }

  /** Add extra initial velocity scaling to the HOUBA one.
   *
   */
  void extraInitVelScaling(double scaling)
  {
    extraInitVelScaling_ = scaling;
  }

  /** Get extra target velocity scaling.
   *
   */
  double extraTargetVelScaling()
  {
    return extraTargetVelScaling_;
  }

  /** Add extra target velocity scaling to the HOUBA one.
   *
   */
  void extraTargetVelScaling(double scaling)
  {
    extraTargetVelScaling_ = scaling;
  }

protected:
  double extraInitVelScaling_ = 1.0;
  double extraTargetVelScaling_ = 1.0;
};

/** Quintic polynomial.
 *
 */
template<typename T>
struct QuinticPolynomialBase : PolynomialBase<T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Build a new curve from its monomial vector coefficients.
   *
   * \param C0 Zero-order coefficient.
   *
   * \param C1 First-order coefficient.
   *
   * \param C2 Second-order coefficient.
   *
   * \param C3 Third-order coefficient.
   *
   * \param C4 Fourth-order coefficient.
   *
   * \param C5 Fifth-order coefficient.
   *
   */
  QuinticPolynomialBase(const T & C0, const T & C1, const T & C2, const T & C3, const T & C4, const T & C5)
  : C0_(C0), C1_(C1), C2_(C2), C3_(C3), C4_(C4), C5_(C5)
  {
  }

  /** Get the value of the polynomial at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  T pos(double t) const
  {
    return C0_ + t * (C1_ + t * (C2_ + t * (C3_ + t * (C4_ + t * C5_))));
  }

  /** Get the value of the first-order derivative (velocity) at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  T vel(double t) const
  {
    return C1_ + t * (2 * C2_ + t * (3 * C3_ + t * (4 * C4_ + t * 5 * C5_)));
  }

  /** Get the value of the second-order derivative (acceleration) at time t.
   *
   * \param t Value of the polynomial argument.
   *
   */
  T accel(double t) const
  {
    return 2 * C2_ + t * (6 * C3_ + t * (12 * C4_ + t * 20 * C5_));
  }

protected:
  T C0_;
  T C1_;
  T C2_;
  T C3_;
  T C4_;
  T C5_;
};

/** Quintic polynomial over vectors.
 *
 */
template<typename T>
struct QuinticPolynomial : QuinticPolynomialBase<T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Return T's zero.
   *
   */
  T zero()
  {
    return T::Zero();
  }

  /** Empty constructor.
   *
   */
  QuinticPolynomial() : QuinticPolynomialBase<T>(T::Zero(), T::Zero(), T::Zero(), T::Zero(), T::Zero(), T::Zero()) {}
};

/** Quintic polynomial over floating-point numbers.
 *
 */
template<>
struct QuinticPolynomial<double> : QuinticPolynomialBase<double>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Zero function used for double specialization.
   *
   */
  double zero()
  {
    return 0.;
  }

  /** Empty constructor.
   *
   */
  QuinticPolynomial() : QuinticPolynomialBase<double>(0., 0., 0., 0., 0., 0.) {}
};

/** Quintic polynomial with zero velocity and zero acceleration at 0 and 1.
 *
 */
template<typename T>
struct QuinticHermitePolynomial : public QuinticPolynomial<T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Empty constructor.
   *
   */
  QuinticHermitePolynomial()
  {
    QuinticHermitePolynomial(this->zero(), this->zero(), this->zero(), this->zero());
  }

  /** Build a new cubic Hermite polynomial.
   *
   * \param initPos Position at t=0.
   *
   * \param initVel Velocity at t=0.
   *
   * \param targetPos Position at t=1.
   *
   * \param targetVel Velocity at t=1.
   *
   */
  QuinticHermitePolynomial(const T & initPos, const T & initVel, const T & targetPos, const T & targetVel)
  {
    reset(initPos, initVel, targetPos, targetVel);
  }

  /** Reset boundary values.
   *
   * \param initPos Position at t=0.
   *
   * \param initVel Velocity at t=0.
   *
   * \param initAccel Acceleration at t=0.
   *
   * \param targetPos Position at t=1.
   *
   * \param targetVel Velocity at t=1.
   *
   * \param targetAccel Acceleration at t=1.
   *
   */
  void reset(const T & initPos,
             const T & initVel,
             const T & initAccel,
             const T & targetPos,
             const T & targetVel,
             const T & targetAccel)
  {
    initPos_ = initPos;
    initVel_ = initVel;
    initAccel_ = initAccel;
    targetPos_ = targetPos;
    targetVel_ = targetVel;
    targetAccel_ = targetAccel;
    reset();
  }

  /** Reset boundaries with zero boundary velocities.
   *
   * \param initPos Position at t=0.
   *
   * \param initVel Velocity at t=0.
   *
   * \param targetPos Position at t=1.
   *
   * \param targetVel Velocity at t=1.
   *
   */
  void reset(const T & initPos, const T & initVel, const T & targetPos, const T & targetVel)
  {
    initPos_ = initPos;
    initVel_ = initVel;
    initAccel_ = this->zero();
    targetPos_ = targetPos;
    targetVel_ = targetVel;
    targetAccel_ = this->zero();
    reset();
  }

  /** Reset boundaries with zero tangents.
   *
   * \param initPos Position at t=0.
   *
   * \param targetPos Position at t=1.
   *
   */
  void reset(const T & initPos, const T & targetPos)
  {
    initPos_ = initPos;
    initVel_ = this->zero();
    initAccel_ = this->zero();
    targetPos_ = targetPos;
    targetVel_ = this->zero();
    targetAccel_ = this->zero();
    reset();
  }

  /** Reset underlying cubic polynomial coefficients.
   *
   */
  void reset()
  {
    auto Delta = targetPos_ - initPos_;
    auto Sigma = initVel_ + targetVel_;
    auto Gamma = 0.5 * (targetAccel_ - initAccel_);
    auto kappa = initVel_ + 0.5 * initAccel_;
    this->C0_ = initPos_;
    this->C1_ = initVel_;
    this->C2_ = 0.5 * initAccel_;
    this->C3_ = 10 * Delta - 4 * Sigma + Gamma - 2 * kappa;
    this->C4_ = -15 * Delta + 7 * Sigma - 2 * Gamma + kappa;
    this->C5_ = 6 * Delta - 3 * Sigma + Gamma;
  }

protected:
  T initPos_;
  T initVel_;
  T initAccel_;
  T targetPos_;
  T targetVel_;
  T targetAccel_;
};

/** Polynomial whose argument \f$s \in [0, 1]\f$ is retimed to \f$t \in [0,
 * T]\f$ by \f$ s(t) = t / T \f$.
 *
 */
template<template<class> class Polynomial, typename T>
struct RetimedPolynomial
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Empty constructor.
   *
   */
  RetimedPolynomial() : duration_(0.) {}

  /** Constructor.
   *
   * \param poly Polynomial whose argument \f$s \in [0, 1]\f$.
   *
   * \param duration New duration T of the retiming \f$t \in [0, T]\f$.
   *
   */
  RetimedPolynomial(Polynomial<T> poly, double duration) : poly_(poly), duration_(duration) {}

  /** Get trajectory duration.
   *
   */
  double duration() const
  {
    return duration_;
  }

  /** Reset duration and boundary positions.
   *
   * \param initPos Initial position.
   *
   * \param targetPos Target position.
   *
   * \param duration New duration T of the trajectory.
   *
   */
  void reset(const T & initPos, const T & targetPos, double duration)
  {
    duration_ = duration;
    poly_.reset(initPos, targetPos);
  }

  /** Reset duration and boundary positions and velocities.
   *
   * \param initPos Initial position.
   *
   * \param initVel Initial velocity.
   *
   * \param targetPos Target position.
   *
   * \param targetVel Target velocity.
   *
   * \param duration New duration T of the trajectory.
   *
   */
  void reset(const T & initPos, const T & initVel, const T & targetPos, const T & targetVel, double duration)
  {
    duration_ = duration;
    poly_.reset(initPos, duration * initVel, targetPos, duration * targetVel);
  }

  /** Reset duration and boundary positions, velocities and accelerations.
   *
   * \param initPos Initial position.
   *
   * \param initVel Initial velocity.
   *
   * \param initAccel Initial acceleration.
   *
   * \param targetPos Target position.
   *
   * \param targetVel Target velocity.
   *
   * \param targetAccel Target acceleration.
   *
   * \param duration New duration T of the trajectory.
   *
   */
  void reset(const T & initPos,
             const T & initVel,
             const T & initAccel,
             const T & targetPos,
             const T & targetVel,
             const T & targetAccel,
             double duration)
  {
    duration_ = duration;
    double duration2 = duration * duration;
    poly_.reset(initPos, duration * initVel, duration2 * initAccel, targetPos, duration * targetVel,
                duration2 * targetAccel);
  }

  /** Mapping from \f$t \in [0, T]\f$ to \f$s(t) \in [0, 1]\f$.
   *
   */
  double s(double t) const
  {
    return (t < 0.) ? 0. : (t > duration_) ? 1. : t / duration_;
  }

  /** Mapping from \f$t \in [0, T]\f$ to
   * \f$\dot{s}(t) = \frac{{\rm d}(s(t))}{{\rm d}t}\f$.
   *
   */
  double sd(double t) const
  {
    return (t < 0.) ? 0. : (t > duration_) ? 0. : 1. / duration_;
  }

  /** Position along the retimed trajectory.
   *
   * \param t Time \f$t \in [0, T]\f$.
   *
   */
  T pos(double t) const
  {
    return poly_.pos(s(t));
  }

  /** Velocity along the retimed trajectory.
   *
   * \param t Time t \f$\in [0, T]\f$.
   *
   */
  T vel(double t) const
  {
    return sd(t) * poly_.vel(s(t));
  }

  /** Acceleration along the retimed trajectory.
   *
   * \param t Time \f$t \in [0, T]\f$.
   *
   */
  T accel(double t) const
  {
    return std::pow(sd(t), 2) * poly_.accel(s(t));
  }

protected:
  double duration_;
  Polynomial<T> poly_;
};

} // namespace utils

using utils::CubicHermitePolynomial;
using utils::CubicPolynomial;
using utils::HoubaPolynomial;
using utils::QuinticHermitePolynomial;
using utils::QuinticPolynomial;
using utils::RetimedPolynomial;
