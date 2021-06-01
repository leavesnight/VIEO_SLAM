//created by zzh, inspired by JingWang
#ifndef ODOMSTATE_H
#define ODOMSTATE_H

#include "Eigen/Core"
#include "so3_extra.h"

namespace VIEO_SLAM{

using namespace Eigen;

typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 6, 1> Vector6d;

//navigation state xj for IMU, and my encoder state is included
class NavState{//refer to the JW's NavState.h, used in PR&V/PVR order, we will use w means world Frame=C0/c0/0th frame/KF of Camera Frame in the whole project when no specially explained
  //cv::Mat mTbc,mTbo;//Tbc is from IMU frame to camera frame;Tbo is from IMU frame to encoder frame(the centre of 2 driving wheels, +x pointing to forward,+z pointing up)
  
public:
  Sophus::SO3exd mRwb;	// rotation Rwbj=Rwb(tj)=qwbj=wqwb(tj)=Phiwbj=wPhiwb(tj), public for g2otypes
  Vector3d mpwb;	// position pwbj=wpwb(tj) or twbj
  Vector3d mvwb;	// velocity vwbj=wvwb(tj)
  // keep unchanged during optimization
  Vector3d mbg;		// bias of gyroscope bgj_bar=bg(tj)_bar
  Vector3d mba;		// bias of accelerometer baj_bar=ba(tj)_bar
  // update below term during optimization
  Vector3d mdbg;  	// delta bias of gyroscope, correction term computed in optimization, d means small delta
  Vector3d mdba;  	// delta bias of accelerometer
  
  NavState():mpwb(0,0,0),mvwb(0,0,0),mbg(0,0,0),mba(0,0,0),mdbg(0,0,0),mdba(0,0,0){}
  NavState(const NavState &x):mpwb(x.mpwb),mvwb(x.mvwb),mRwb(x.mRwb),mbg(x.mbg),mba(x.mba),mdbg(x.mdbg),mdba(x.mdba){}//though Eigen has deep copy, use initialization list to speed up

  Matrix3d getRwb() const{return mRwb.matrix();}//get rotation matrix of Rwbj, const is very important!
  //if SO3exd could be used, it will be safer here
  void setRwb(const Matrix3d &Rwb){mRwb=Sophus::SO3exd(Rwb);}//implicitly use SO3(Matrix3d)
  
  // incremental addition, dx = [dP, dV, dPhi, dBa, dBg] for oplusImpl(), see Manifold paper (70)
  template <int D>//default is for 6, IncSmallPR
  void IncSmall(const Matrix<double,D,1> &dPR){//also inline
//     mpwb+=mRwb*dPR.template segment<3>(0);//here dp<-p+R*dp(in paper On-Manifold Preintegration)
    mpwb+=dPR.template segment<3>(0);//here p<-p+dp, dp=R*dp(in paper On-Manifold Preintegration)
    mRwb*=Sophus::SO3exd::exp(dPR.template segment<3>(3));//right distrubance model
  }
  inline void IncSmallBias(const Vector6d &dBias){
    mdbg+=dBias.segment<3>(0);mdba+=dBias.segment<3>(3);
  }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW//for quaterniond in SO3
};
//specialized function when using Matrix<double,D,1> I've to defined in .h with inline/static(not good), when it's defined in NavState.cpp, it enters the undefined reference problem? Though I try using Matrix<double,D,1,0> can overcome this problem
template<>
inline void NavState::IncSmall<3>(const Vector3d &dV){
  mvwb+=dV;
}//IncSmallV
template<>
inline void NavState::IncSmall<9>(const Vector9d &dPVR){
//   mpwb+=mRwb*dPVR.segment<3>(0);//here dp<-p+R*dp(in paper On-Manifold Preintegration)
  mpwb+=dPVR.segment<3>(0);//we don't discover improvement using p<-p+R*dp, so we prefer the simple form p<-p+dp
  mvwb+=dPVR.segment<3>(3);
  mRwb*=Sophus::SO3exd::exp(dPVR.segment<3>(6));
}//IncSmallPVR

}

#endif
