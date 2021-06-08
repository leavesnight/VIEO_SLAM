//
// Created by leavesnight on 2021/3/29.
//

#ifndef VIEO_SLAM_SO3_EXTRA_H
#define VIEO_SLAM_SO3_EXTRA_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#ifdef USE_SOPHUS_NEWEST
#include "sophus/so3.hpp"
#endif

namespace Sophus {
template <class Scalar, int Options = 0>
class SO3ex : public SO3<Scalar, Options> {
  const static double SMALL_EPS;

 protected:
  using Base = SO3<Scalar, Options>;
  using Base::unit_quaternion_;
  using Base::unit_quaternion_nonconst;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using QuaternionMember = typename Base::QuaternionMember;  // Eigen::Quaternion<Scalar, Options>
  using Tangent = typename Base::Tangent;                    // Eigen::Matrix<Scalar, 3, 1>
  using Point = typename Base::Point;                        // Vector3
  using Transformation = typename Base::Transformation;      // Eigen::Matrix<Scalar, 3, 3>
  // here Eigen::ScalarBinaryOpTraits<> in 3.3.7 and sophus (Wed Apr 21 18:12:08 2021 -0700) should only accept same
  // scalar or one is complex
  template <typename OtherDerived>
  using ReturnScalar = typename Base::template ReturnScalar<OtherDerived>;
  template <typename OtherDerived>
  using SO3exProduct = SO3ex<ReturnScalar<OtherDerived>>;
  template <typename PointDerived>
  using PointProduct = typename Base::template PointProduct<PointDerived>;
  using Base::unit_quaternion;

  // SOPHUS_FUNC now is EIGEN_DEVICE_FUNC for CUDA usage: __host__ __device__ means cpu&&gpu both make this func
  SOPHUS_FUNC SO3ex()
#ifdef USE_SOPHUS_NEWEST
      : Base() {
  }
  template <class OtherDerived>
  SOPHUS_FUNC SO3ex(SO3Base<OtherDerived> const& other) : Base(other) {}
#else
  {
    unit_quaternion_.setIdentity();
  }
#endif
  SOPHUS_FUNC SO3ex(const SO3ex& other)
#ifdef USE_SOPHUS_NEWEST
      : Base(other) {
  }
#else
  {
    unit_quaternion_ = other.unit_quaternion_;
    // if we ensure all SO3ex changing func will ensure the unit property, this normalize can be omitted
    unit_quaternion_.normalize();
  }
#endif
  // so3 will assert identity of R, but for convenience, we nomralize all R here
  SOPHUS_FUNC SO3ex(Transformation const& R) {
    this->unit_quaternion_ = QuaternionMember(R);
    this->unit_quaternion_.normalize();
  }
  template <class D>
  SOPHUS_FUNC explicit SO3ex(Eigen::QuaternionBase<D> const& quat)
#ifdef USE_SOPHUS_NEWEST
      : Base(quat) {
  }
#else
  {
    unit_quaternion_ = quat;
    static_assert(std::is_same<typename Eigen::QuaternionBase<D>::Scalar, Scalar>::value,
                  "Input must be of same scalar type");
    assert(unit_quaternion_.squaredNorm() > SMALL_EPS);
    unit_quaternion_.normalize();
  }
#endif

#ifndef USE_SOPHUS_NEWEST
  template <class OtherDerived>
  SOPHUS_FUNC SO3ex& operator=(SO3Base<OtherDerived> const& other) {
    this->unit_quaternion_ = other.unit_quaternion();
    return *this;
  }
  template <typename OtherDerived>
  SOPHUS_FUNC SO3exProduct<OtherDerived> operator*(SO3Base<OtherDerived> const& other) const {
    // here Eigen 3.3.7 and sophus (Wed Apr 21 18:12:08 2021 -0700) only support
    // SO3Base<Derived<Scalar>>*SO3Base<Derived<Scalar>>
    SO3ex<Scalar> result(*this);
    result *= other;
    return result;
  }
  // typename without name could help confirm different SO3ex<Scalar> *= op. at compiling stage
  template <typename OtherDerived,
            typename = typename std::enable_if<std::is_same<Scalar, ReturnScalar<OtherDerived>>::value>::type>
  SOPHUS_FUNC SO3ex& operator*=(SO3Base<OtherDerived> const& other) {
    unit_quaternion_ *= other.unit_quaternion();
    unit_quaternion_.normalize();
    return *this;
  }
  template <typename PointDerived, typename = typename std::enable_if<IsFixedSizeVector<PointDerived, 3>::value>::type>
  SOPHUS_FUNC PointProduct<PointDerived> operator*(Eigen::MatrixBase<PointDerived> const& p) const {
    return unit_quaternion_._transformVector(p);  // only suitable for SO3ex<Scalar> * Vector3<Scalar>
  }
  SOPHUS_FUNC SO3ex inverse() const { return SO3ex(unit_quaternion_.conjugate()); }
  SOPHUS_FUNC Transformation matrix() const { return unit_quaternion_.toRotationMatrix(); }
  SOPHUS_FUNC static Transformation hat(Tangent const& omega) {
    Transformation Omega;
    Omega << Scalar(0), -omega(2), omega(1), omega(2), Scalar(0), -omega(0), -omega(1), omega(0), Scalar(0);
    return Omega;
  }
  SOPHUS_FUNC static Tangent vee(Transformation const& Omega) {
    assert(fabs(Omega(2, 1) + Omega(1, 2)) < SMALL_EPS);
    assert(fabs(Omega(0, 2) + Omega(2, 0)) < SMALL_EPS);
    assert(fabs(Omega(1, 0) + Omega(0, 1)) < SMALL_EPS);
    return Tangent(Omega(2, 1), Omega(0, 2), Omega(1, 0));
  }
  SOPHUS_FUNC QuaternionMember const& unit_quaternion() const { return unit_quaternion_; }
#endif

  SOPHUS_FUNC static SO3ex exp(Tangent const& omega) {
    Scalar theta_impl = 0;
    Scalar* theta = &theta_impl;
    *theta = omega.norm();
    Scalar half_theta = 0.5 * (*theta);

    Scalar imag_factor;
    Scalar real_factor;
    if ((*theta) < SMALL_EPS) {
      double theta_sq = (*theta) * (*theta);
      //    double theta_po4 = theta_sq*theta_sq;
      imag_factor = 0.5 - theta_sq / 48.;  // + theta_po4 / 3840.;//Taylor expansion of sin(x/2)/x
      real_factor = 1.0 - theta_sq / 8.;   // + theta_po4 / 384.;
    } else {
      Scalar sin_half_theta = sin(half_theta);
      imag_factor = sin_half_theta / (*theta);
      real_factor = cos(half_theta);
    }

    return SO3ex(
        QuaternionMember(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z()));
  }
  // range[-pi,pi) is mainly designed for residual error(abs or pow 2), (-pi,pi) could ensure uniqueness, but when
  // theta = -pi this log() cannot ensure uniqueness, except we define sth. like
  // if abs(theta)=pi {suppose q=(w,xyz) then theta=-pi, ensure z>=0{(if z<0:xyz=-xyz), if (z==0) ensure y>=0{if (y ==
  // 0) ensure x>=0}}}, g2o's se3quat also has such problem, which could limit the interpolation op. of system's state
  // one way is to use Eigen's slerp(<=3.3.7 confirmed), which will limit the angle between 2 qs <= pi/2, suitable for
  // system's state's interpolation and won't require the uniqueness of q but if you use linear interpolation here, you
  // should ensure the input q's uniqueness problem)
  // Also we should notice interpolation op. on angular velocity(which should have no limit on its range, so usually
  // it's interpolated through simple linear one instead of angular linear one like slerp)
  SOPHUS_FUNC Tangent log() const {
    const SO3ex& other = *this;
    Scalar n = other.unit_quaternion_.vec().norm();  // sin(theta/2)
    Scalar w = other.unit_quaternion_.w();           // cos(theta/2)
    Scalar squared_w = w * w;

    Scalar two_atan_nbyw_by_n;
    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    // small variable approximation is used for speed but keep the max or reasonable accuracy
    // (so3.cpp here choose the max or double(1) + double(2^(-52)))
    if (n < SMALL_EPS) {
      // If quaternion is normalized and n=1, then w should be 1;
      // w=0 should never happen here!
      assert(fabs(w) > SMALL_EPS);

      two_atan_nbyw_by_n = 2. / w - 2. / 3 * (n * n) / (w * squared_w);  // TODO: try right Taylor 2./3
    } else {
      if (fabs(w) < SMALL_EPS) {  // notice atan(x) = pi/2 - atan(1/x)
        if (w > 0) {              // notice for range[-pi,pi), atan(x) = pi/2 - atan(1/x) for x>0
          two_atan_nbyw_by_n = M_PI / n;
        } else                             // w=0 corresponds to theta = Pi or -Pi, here choose -Pi
        {                                  // notice for range[-pi,pi), atan(x) = -pi/2 - atan(1/x) for x<=0
          two_atan_nbyw_by_n = -M_PI / n;  // theta belongs to [-Pi,Pi)=>theta/2 in [-Pi/2,Pi/2)
        }
        double n_pow2 = n * n;
        double n_pow4 = n_pow2 * n_pow2;
        two_atan_nbyw_by_n -= 2 * w / n_pow2 - 2. / 3 * (w * squared_w) / n_pow4;
      } else
        two_atan_nbyw_by_n = 2 * atan(n / w) / n;  // theta/sin(theta/2)
    }

    return two_atan_nbyw_by_n * other.unit_quaternion_.vec();
  }

  // Jr, right jacobian of SO(3)
  SOPHUS_FUNC static Transformation JacobianR(const Tangent& w);
  // Jr^(-1)
  SOPHUS_FUNC static Transformation JacobianRInv(const Tangent& w);
  // Jl, left jacobian of SO(3), Jl(x) = Jr(-x)
  SOPHUS_FUNC static Transformation JacobianL(const Tangent& w) { return JacobianR(-w); }
  // Jl^(-1)
  SOPHUS_FUNC static Transformation JacobianLInv(const Tangent& w) { return JacobianRInv(-w); }
};

template <class Scalar, int Options>
const double SO3ex<Scalar, Options>::SMALL_EPS = 1e-5;

template <class Scalar, int Options>
typename SO3ex<Scalar, Options>::Transformation SO3ex<Scalar, Options>::JacobianR(const Tangent& w) {
  Transformation Jr = Transformation::Identity();
  Scalar theta = w.norm();
  if (theta < 1e-5) {
    Transformation Omega = Base::hat(w);
    Transformation Omega2 = Omega * Omega;
    // omit 3rd order eps & more for 1e-5 (accuracy:e-10), similar to omit >=1st order (Jl=I/R) for 1e-10
    // the one more order term is theta*theta*Omega/24. < 2^(-52), where 1 + it is useless
    return Jr - 0.5 * Omega + Omega2 / 6.;
  } else {
    Tangent k = w.normalized();  // k - unit direction vector of w
    Transformation K = Base::hat(k);
    Jr = Transformation::Identity() - (1 - cos(theta)) / theta * K + (1 - sin(theta) / theta) * K * K;
  }
  return Jr;
}
template <class Scalar, int Options>
typename SO3ex<Scalar, Options>::Transformation SO3ex<Scalar, Options>::JacobianRInv(const Tangent& w) {
  Transformation Jrinv = Transformation::Identity();
  Scalar theta = w.norm();
  Transformation Omega = Base::hat(w);

  // very small angle
  if (theta < 1e-5) {
    // limit(theta->0)((1-theta/(2*tan(theta/2)))/theta^2)~=(omit theta^5&&less)=1/12
    return Jrinv + 0.5 * Omega + (1. / 12.) * (Omega * Omega);
  } else {
    Tangent k = w.normalized();  // k - unit direction vector of w
    Transformation K = Base::hat(k);
    Jrinv = Transformation::Identity() + 0.5 * Omega + (1.0 - (1.0 + cos(theta)) * theta / (2.0 * sin(theta))) * K * K;
  }

  return Jrinv;
}

typedef SO3ex<double> SO3exd;
typedef SO3ex<float> SO3exf;

}  // namespace Sophus

#endif  // VIEO_SLAM_SO3_EXTRA_H
