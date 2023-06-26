// created by zzh, inspired by JingWang
#ifndef ODOMSTATE_H
#define ODOMSTATE_H

#include "Eigen/Core"
#include "common/so3_extra.h"

#define USE_P_PLUS_RDP

namespace VIEO_SLAM {

using namespace Eigen;

typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;

// navigation state xj for IMU, and my encoder state is included
class NavState {  // refer to the JW's NavState.h, used in PR&V/PVR order, we will use w means world Frame=C0/c0/0th
                  // frame/KF of Camera Frame in the whole project when no specially explained
  // cv::Mat mTbc,mTbo;//Tbc is from IMU frame to camera frame;Tbo is from IMU frame to encoder frame(the centre of 2
  // driving wheels, +x pointing to forward,+z pointing up)

 public:
  typedef double Tdata;
  typedef double Tcalc;
  using Vector3c = Matrix<Tcalc, 3, 1>;  // TODO make this class template
  using SO3c = Sophus::SO3ex<Tcalc>;
  Sophus::SO3exd mRwb;  // rotation Rwbj=Rwb(tj)=qwbj=wqwb(tj)=Phiwbj=wPhiwb(tj), public for g2otypes
  Vector3d mpwb;        // position pwbj=wpwb(tj) or twbj
  Vector3d mvwb;        // velocity vwbj=wvwb(tj)
  // keep unchanged during optimization
  Vector3d mbg;  // bias of gyroscope bgj_bar=bg(tj)_bar
  Vector3d mba;  // bias of accelerometer baj_bar=ba(tj)_bar
  // update below term during optimization
  Vector3d mdbg;  // delta bias of gyroscope, correction term computed in optimization, d means small delta
  Vector3d mdba;  // delta bias of accelerometer

  NavState() : mpwb(0, 0, 0), mvwb(0, 0, 0), mbg(0, 0, 0), mba(0, 0, 0), mdbg(0, 0, 0), mdba(0, 0, 0) {}
  // for Eigen has deep copy, we don't define default copy constructor and operator= and we don't need ~NavState(),
  // so move copy constructor and move operator= will also be default

  Matrix3d getRwb() const { return mRwb.matrix(); }  // get rotation matrix of Rwbj, const is very important!
  // if SO3exd could be used, it will be safer here
  void setRwb(const Matrix3d &Rwb) { mRwb = Sophus::SO3exd(Rwb); }  // implicitly use SO3(Matrix3d)

  // incremental addition, dx = [dP, dV, dPhi, dBa, dBg] for oplusImpl(), see Manifold paper (70)
  void IncSmall(const Eigen::Map<const Matrix<Tcalc, 6, 1>> &dPR) {  // also inline
    Vector3c pwb = mpwb.template cast<Tcalc>();
    SO3c Rwb = mRwb.template cast<Tcalc>();
#ifdef USE_P_PLUS_RDP
    pwb += Rwb * dPR.template segment<3>(0);  // here dp<-p+R*dp(in paper On-Manifold Preintegration)
#else
    pwb += dPR.template segment<3>(0);  // here p<-p+dp, dp=R*dp(in paper On-Manifold Preintegration)
#endif
    Rwb *= SO3c::exp(dPR.template segment<3>(3));  // right distrubance model
    mpwb = pwb.template cast<Tdata>();
    mRwb = Rwb.template cast<Tdata>();
  }
  void IncSmall(const Eigen::Map<const Vector3c> &dV) {  // use overload to implement part specialization
    Vector3c vwb = mvwb.template cast<Tcalc>();
    vwb += dV;
    mvwb = vwb.template cast<Tdata>();
  }
  void IncSmall(const Eigen::Map<const Matrix<Tcalc, 9, 1>> &dPVR) {  // TODO: delte this func for code simplicity
    Vector3c pwb = mpwb.template cast<Tcalc>();
    SO3c Rwb = mRwb.template cast<Tcalc>();
#ifdef USE_P_PLUS_RDP
    pwb += Rwb * dPVR.template segment<3>(0);  // here dp<-p+R*dp(in paper On-Manifold Preintegration)
#else
    pwb += dPVR.template segment<3>(0);
#endif
    Vector3c vwb = mvwb.template cast<Tcalc>();
    vwb += dPVR.template segment<3>(3);
    Rwb *= SO3c::exp(dPVR.template segment<3>(6));
    mpwb = pwb.template cast<Tdata>();
    mvwb = vwb.template cast<Tdata>();
    mRwb = Rwb.template cast<Tdata>();
  }
  inline void IncSmallBias(const Vector6d &dBias) {
    mdbg += dBias.segment<3>(0);
    mdba += dBias.segment<3>(3);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // for quaterniond in SO3
};

}  // namespace VIEO_SLAM

#endif
