/* Copyright 2018 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of lipm_walking_controller.
 *
 * lipm_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * lipm_walking_controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lipm_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#pragma once

/** Cubic polynomial curve.
 *
 */
template <typename T>
struct CubicPolynomialBase
{
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
  CubicPolynomialBase(const T & C0, const T & C1, const T & C2, const T & C3)
    : C0_(C0), C1_(C1), C2_(C2), C3_(C3)
  {
  }

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

template <typename T>
struct CubicPolynomial : CubicPolynomialBase<T>
{
  /** Return T's zero.
   *
   */
  inline T zero()
  {
    return T::Zero();
  }

  /** Empty constructor.
   *
   */
  CubicPolynomial()
    : CubicPolynomialBase<T>(T::Zero(), T::Zero(), T::Zero(), T::Zero())
  {
  }
};

template <>
struct CubicPolynomial<double> : CubicPolynomialBase<double>
{
  /** Zero function used for double specialization.
   *
   */
  inline double zero()
  {
    return 0.;
  }

  /** Empty constructor.
   *
   */
  CubicPolynomial()
    : CubicPolynomialBase<double>(0., 0., 0., 0.)
  {
  }
};

/** Cubic Hermite polynomial.
 *
 */
template <typename T>
struct CubicHermitePolynomial : public CubicPolynomial<T>
{
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
  void reset()
  {
    this->C0_ = initPos_;
    this->C1_ = initVel_;
    this->C2_ = 3. * (targetPos_ - initPos_) - 2. * initVel_ - targetVel_;
    this->C3_ = -2. * (targetPos_ - initPos_) + initVel_ + targetVel_;
  }

private:
  T initPos_;
  T initVel_;
  T targetPos_;
  T targetVel_;
};

/** Quintic polynomial.
 *
 */
template <typename T>
struct QuinticPolynomialBase
{
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
template <typename T>
struct QuinticPolynomial : QuinticPolynomialBase<T>
{
  /** Return T's zero.
   *
   */
  inline T zero()
  {
    return T::Zero();
  }

  /** Empty constructor.
   *
   */
  QuinticPolynomial()
    : QuinticPolynomialBase<T>(T::Zero(), T::Zero(), T::Zero(), T::Zero(), T::Zero(), T::Zero())
  {
  }
};

/** Quintic polynomial over floating-point numbers.
 *
 */
template <>
struct QuinticPolynomial<double> : QuinticPolynomialBase<double>
{
  /** Zero function used for double specialization.
   *
   */
  inline double zero()
  {
    return 0.;
  }

  /** Empty constructor.
   *
   */
  QuinticPolynomial()
    : QuinticPolynomialBase<double>(0., 0., 0., 0., 0., 0.)
  {
  }
};

/** Quintic polynomial with zero velocity and zero acceleration at 0 and 1.
 *
 */
template <typename T>
struct QuinticHermitePolynomial : public QuinticPolynomial<T>
{
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
  void reset(const T & initPos, const T & initVel, const T & initAccel, const T & targetPos, const T & targetVel, const T & targetAccel)
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

private:
  T initPos_;
  T initVel_;
  T initAccel_;
  T targetPos_;
  T targetVel_;
  T targetAccel_;
};

/** Polynomial whose argument s \in [0, 1] is retimed to t \in [0, T] by
 *
 *  s(t) = t / T
 *
 */
template <template <class> class Polynomial, typename T>
struct RetimedPolynomial
{
  /** Empty constructor.
   *
   */
  RetimedPolynomial()
    : duration_(0.)
  {
  }

  /** Constructor.
   *
   * \param poly Polynomial whose argument s \in [0, 1].
   *
   * \param duration New duration T of the retiming t \in [0, T].
   *
   */
  RetimedPolynomial(Polynomial<T> poly, double duration)
    : poly_(poly),
      duration_(duration)
  {
  }

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
  void reset(const T & initPos, const T & initVel, const T & initAccel, const T & targetPos, const T & targetVel, const T & targetAccel, double duration)
  {
    duration_ = duration;
    double duration2 = duration * duration;
    poly_.reset(initPos, duration * initVel, duration2 * initAccel, targetPos, duration * targetVel, duration2 * targetAccel);
  }

  /** Mapping from t \in [0, T] to s(t) \in [0, 1].
   *
   */
  double s(double t) const
  {
    return (t < 0.) ? 0. : (t > duration_) ? 1. : t / duration_;
  }

  /** Mapping from t \in [0, T] to sd(t) = d(s(t)) / dt.
   *
   */
  double sd(double t) const
  {
    return (t < 0.) ? 0. : (t > duration_) ? 0. : 1. / duration_;
  }

  /** Position along the retimed trajectory.
   *
   * \param t Time t \in [0, T].
   *
   */
  T pos(double t) const
  {
    return poly_.pos(s(t));
  }

  /** Velocity along the retimed trajectory.
   *
   * \param t Time t \in [0, T].
   *
   */
  T vel(double t) const
  {
    return sd(t) * poly_.vel(s(t));
  }

  /** Acceleration along the retimed trajectory.
   *
   * \param t Time t \in [0, T].
   *
   */
  T accel(double t) const
  {
    return std::pow(sd(t), 2) * poly_.accel(s(t));
  }

private:
  double duration_;
  Polynomial<T> poly_;
};
