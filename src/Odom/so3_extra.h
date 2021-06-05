//
// Created by leavesnight on 2021/3/29.
//

#ifndef VIEO_SLAM_SO3_EXTRA_H
#define VIEO_SLAM_SO3_EXTRA_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

//#include "sophus/so3.hpp"

namespace Sophus {
//template <class Scalar, int Options = 0>
//class SO3ex : public SO3<Scalar, Options> {
//  const static double SMALL_EPS;
//
// protected:
//  using Base = SO3<Scalar, Options>;
//  using Base::unit_quaternion_;
//
// public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//  using Matrix3 = Matrix<Scalar, 3, 3>;
//  using Vector3 = Matrix<Scalar, 3, 1>;
//  using QuaternionMember = typename Base::QuaternionMember;
//  using typename Base::Tangent;
//  using typename Base::Transformation;
//  using typename Base::Point;
//
//  SOPHUS_FUNC SO3ex()
//  //          : Base() {}
//  {
//    unit_quaternion_.setIdentity();
//  }
//  SOPHUS_FUNC SO3ex(const Base& base) : Base(base){};
//  SOPHUS_FUNC SO3ex(const SO3ex & other)
//  {
//    unit_quaternion_ = other.unit_quaternion_;
//    unit_quaternion_.normalize();
//  }
//  SOPHUS_FUNC SO3ex(Transformation const& R) {
//    this->unit_quaternion_ = QuaternionMember(R);
//    this->unit_quaternion_.normalize();
//  }
//  SOPHUS_FUNC explicit SO3ex(QuaternionMember const& quat)
//      //            : Base(quat){
//  {
//    unit_quaternion_ = quat;
//    assert(unit_quaternion_.squaredNorm() > SMALL_EPS);
//    unit_quaternion_.normalize();
//  }
//  SOPHUS_FUNC SO3ex operator*(SO3ex const& other) const {
////    return Base::operator*(other);
//    SO3ex<Scalar> result(*this);
//    result *= other;
//    return result;
//  }
//  SOPHUS_FUNC SO3ex& operator*=(SO3ex const& other) {
////    return Base::operator*=(other);
//    unit_quaternion_ *= other.unit_quaternion_;
//    unit_quaternion_.normalize();
//  }
//  SOPHUS_FUNC Point operator*(Point const& p) const {
////    return Base::operator*(p);//TODO:check if right
//    return unit_quaternion_._transformVector(p);
//  }
//  SOPHUS_FUNC void operator=(const SO3ex & other)
//  {
//    this->unit_quaternion_ = other.unit_quaternion_;
//  }
//
//  // Jr, right jacobian of SO(3)
//  static Matrix3 JacobianR(const Vector3& w);
//  // Jr^(-1)
//  static Matrix3 JacobianRInv(const Vector3& w);
//  // Jl, left jacobian of SO(3), Jl(x) = Jr(-x)
//  static Matrix3 JacobianL(const Vector3& w) { return JacobianR(-w); }
//  // Jl^(-1)
//  static Matrix3 JacobianLInv(const Vector3& w) { return JacobianRInv(-w); }
//
//  SOPHUS_FUNC static SO3ex<Scalar> exp(Tangent const& omega) {
//    //          return Base::exp(omega);
//    Scalar theta_impl = 0;
//    Scalar* theta = &theta_impl;
//    *theta = omega.norm();
//    Scalar half_theta = 0.5 * (*theta);
//
//    Scalar imag_factor;
//    Scalar real_factor = cos(half_theta);
//    if ((*theta) < SMALL_EPS) {
//      Scalar theta_sq = (*theta) * (*theta);
//      Scalar theta_po4 = theta_sq * theta_sq;
//      imag_factor = 0.5 - theta_sq / 48. + theta_po4 / 3840.0;  // Taylor expansion of sin(x/2)/x
//    } else {
//      Scalar sin_half_theta = sin(half_theta);
//      imag_factor = sin_half_theta / (*theta);
//    }
//
//    return SO3ex(QuaternionMember(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z()));
//  }
//  SOPHUS_FUNC Tangent log() const {
//    //          return Base::log();
//    const SO3ex& other = *this;
//    //          Scalar *theta;
//    Scalar n = other.unit_quaternion_.vec().norm();  // sin(theta/2)
//    Scalar w = other.unit_quaternion_.w();           // cos(theta/2)
//    Scalar squared_w = w * w;
//
//    Scalar two_atan_nbyw_by_n;
//    // Atan-based log thanks to
//    //
//    // C. Hertzberg et al.:
//    // "Integrating Generic Sensor Fusion Algorithms with Sound State
//    // Representation through Encapsulation of Manifolds"
//    // Information Fusion, 2011
//
//    if (n < SMALL_EPS) {
//      // If quaternion is normalized and n=1, then w should be 1;
//      // w=0 should never happen here!
//      assert(fabs(w) > SMALL_EPS);
//
//      two_atan_nbyw_by_n = 2. / w - 2. * (n * n) / (w * squared_w);  // TODO: try right Taylor 2./3
//    } else {
//      if (fabs(w) < SMALL_EPS) {
//        if (w > 0) {
//          two_atan_nbyw_by_n = M_PI / n;
//        } else  // w=0 corresponds to theta= Pi or -Pi, here choose -Pi
//        {
//          two_atan_nbyw_by_n = -M_PI / n;  // theta belongs to [-Pi,Pi)=>theta/2 in [-Pi/2,Pi/2)
//        }
//      } else
//        two_atan_nbyw_by_n = 2 * atan(n / w) / n;  // theta/sin(theta/2)
//    }
//
//    //          *theta = two_atan_nbyw_by_n*n;
//    return two_atan_nbyw_by_n * other.unit_quaternion_.vec();
//  }
//};
//
//template <class Scalar, int Options>
//const double SO3ex<Scalar, Options>::SMALL_EPS = 1e-10;
//
//template <class Scalar, int Options>
//typename SO3ex<Scalar, Options>::Matrix3 SO3ex<Scalar, Options>::JacobianR(const Vector3& w) {
//  Matrix3 Jr = Matrix3::Identity();
//  Scalar theta = w.norm();
//  if (theta < 1e-5) {
//    Matrix3 Omega = Base::hat(w);
//    Matrix3 Omega2 = Omega * Omega;
//    // omit 3rd order eps & more for 1e-5 (accuracy:e-10), similar to omit >=1st order (Jl=I/R) for 1e-10
//    return Jr - 0.5 * Omega + Omega2 / 6.;
//  } else {
//    Vector3 k = w.normalized();  // k - unit direction vector of w
//    Matrix3 K = Base::hat(k);
//    Jr = Matrix3::Identity() - (1 - cos(theta)) / theta * K + (1 - sin(theta) / theta) * K * K;
//  }
//  return Jr;
//}
//template <class Scalar, int Options>
//typename SO3ex<Scalar, Options>::Matrix3 SO3ex<Scalar, Options>::JacobianRInv(const Vector3& w) {
//  Matrix3 Jrinv = Matrix3::Identity();
//  Scalar theta = w.norm();
//  Matrix3 Omega = Base::hat(w);
//
//  // very small angle
//  if (theta < 1e-5) {
//    // limit(theta->0)((1-theta/(2*tan(theta/2)))/theta^2)~=(omit theta^5&&less)=1/12
//    return Jrinv + 0.5 * Omega + (1. / 12.) * (Omega * Omega);
//  } else {
//    Vector3 k = w.normalized();  // k - unit direction vector of w
//    Matrix3 K = Base::hat(k);
//    Jrinv = Matrix3::Identity() + 0.5 * Omega + (1.0 - (1.0 + cos(theta)) * theta / (2.0 * sin(theta))) * K * K;
//  }
//
//  return Jrinv;
//}
//
//typedef SO3ex<double> SO3exd;
//typedef SO3ex<float> SO3exf;


using namespace Eigen;

const double SMALL_EPS = 1e-10;

class SO3
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Jr, right jacobian of SO(3)
  static Matrix3d JacobianR(const Vector3d& w);
  // Jr^(-1)
  static Matrix3d JacobianRInv(const Vector3d& w);
  // Jl, left jacobian of SO(3), Jl(x) = Jr(-x)
  static Matrix3d JacobianL(const Vector3d& w);
  // Jl^(-1)
  static Matrix3d JacobianLInv(const Vector3d& w);

  // ----------------------------------------

  SO3                        ();//no rotation

  SO3                        (const SO3 & other);

  explicit
  SO3                        (const Matrix3d & _R);

  explicit
  SO3                        (const Quaterniond & unit_quaternion);

  SO3                        (double rot_x,
                              double rot_y,
                              double rot_z);//RPY
  void
  operator=                  (const SO3 & so3);

  SO3
  operator*                  (const SO3 & so3) const;

  void
  operator*=                 (const SO3 & so3);

  Vector3d
  operator*                  (const Vector3d & xyz) const;

  SO3
  inverse                    () const;

  Matrix3d
  matrix                     () const;

  Matrix3d
  Adj                        () const;

  Matrix3d
  generator                  (int i);

  Vector3d
  log                        () const;

  static SO3
  exp                        (const Vector3d & omega);

  static SO3
  expAndTheta                (const Vector3d & omega,
                              double * theta);
  static Vector3d
  log                        (const SO3 & so3);

  static Vector3d
  logAndTheta                (const SO3 & so3,
                              double * theta);

  static Matrix3d
  hat                        (const Vector3d & omega);

  static Vector3d
  vee                        (const Matrix3d & Omega);

  static Vector3d
  lieBracket                 (const Vector3d & omega1,
                              const Vector3d & omega2);

  static Matrix3d
  d_lieBracketab_by_d_a      (const Vector3d & b);

  void
  setQuaternion              (const Quaterniond& quaternion);

  const Quaterniond & unit_quaternion() const
  {
    return unit_quaternion_;
  }

  static const int DoF = 3;

 protected:
  Quaterniond unit_quaternion_;
};

typedef SO3 SO3exd;

}  // namespace Sophus

#endif  // VIEO_SLAM_SO3_EXTRA_H
