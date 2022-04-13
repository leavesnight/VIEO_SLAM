#include "g2otypes.h"

namespace g2o {
using namespace VIEO_SLAM;

// return normalized v(0:1)
Vector2d project2d(const Vector3d& v) {
  Vector2d res;
  res(0) = v(0) / v(2);
  res(1) = v(1) / v(2);
  return res;
}

void EdgeNavStateBias::computeError() {
  const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[0]);
  const VertexNavStateBias* vBiasj = static_cast<const VertexNavStateBias*>(_vertices[1]);
  const NavState& NSi = vBiasi->estimate();
  const NavState& NSj = vBiasj->estimate();

  // rbij/eb=bj-bi see VIORBSLAM paper (6) or Manifold paper (48)
  // residual error of Gyroscope's bias, Forster 15'RSS
  Vector3d rBiasG = (NSj.mbg + NSj.mdbg) - (NSi.mbg + NSi.mdbg);
  // residual error of Accelerometer's bias, Forster 15'RSS
  Vector3d rBiasA = (NSj.mba + NSj.mdba) - (NSi.mba + NSi.mdba);  // here is x(x)xa!!!
  Vector6d err(Vector6d::Zero());
  // 6-Dim error vector order: deltabiasGyr_i;deltabiasAcc_i, rBiasGi;rBiasAi
  err.segment<3>(0) = rBiasG;
  err.segment<3>(3) = rBiasA;
  _error = err;
}
void EdgeNavStateBias::linearizeOplus() {
  _jacobianOplusXi = -Matrix<double, 6, 6>::Identity();  // J_eb_bi=-I for eb=bj-bi
  _jacobianOplusXj = Matrix<double, 6, 6>::Identity();   // J_eb_bj=I
}

typedef Matrix<double, 15, 1> Vector15d;
void EdgeNavStatePriorPRVBias::computeError() {
  const VertexNavStatePR* vNSPR = static_cast<const VertexNavStatePR*>(_vertices[0]);        // PRj
  const VertexNavStateV* vNSV = static_cast<const VertexNavStateV*>(_vertices[1]);           // Vj
  const VertexNavStateBias* vNSBias = static_cast<const VertexNavStateBias*>(_vertices[2]);  // Bj
  const NavState& nsPRest = vNSPR->estimate();
  const NavState& nsVest = vNSV->estimate();
  const NavState& nsBiasest = vNSBias->estimate();

  // P R V bg=bg_bar+dbg ba=ba_bar+dba, see VIORBSLAM paper (8)
  Vector15d err = Vector15d::Zero();
  const Matrix3d Rbw_bar = _measurement.getRwb().transpose();  //_measurement.mRwb.inverse();
  // NOTICE! VIORBSLAM paper wrong in er's sign with the left, so orb3 fix this problem, while vieoslam old version
  // missed er.t*Omega*ep term in H/prior's Info so it seems no problem, but it'll have problem in fast motion dataset!
#ifdef USE_P_PLUS_RDP
  // Rwb.t() for twb<-twb+Rwb*dtwb, where H is for dtwb
  err.segment<3>(0) = Rbw_bar * (nsPRest.mpwb - _measurement.mpwb);  // ep=Rbw(pwbj-pwbj_bar)
#else
  err.segment<3>(0) = nsPRest.mpwb - _measurement.mpwb;  // ep=pwbj-pwbj_bar
#endif
  err.segment<3>(3) = Sophus::SO3exd::Log(Rbw_bar * nsPRest.getRwb());  // eR=Log(Rwbj_bar.t()*Rwbj)
  err.segment<3>(6) = nsVest.mvwb - _measurement.mvwb;                  // ev=vwbj-vwbj_bar
  err.segment<3>(9) = nsBiasest.mbg + nsBiasest.mdbg - (_measurement.mbg + _measurement.mdbg);  // eb=bj-bj_bar
  err.segment<3>(12) = nsBiasest.mba + nsBiasest.mdba - (_measurement.mba + _measurement.mdba);
  _error = err;
}
void EdgeNavStatePriorPRVBias::linearizeOplus() {
  // J_ej_dPRj=[I Jrinv(phi_eRj)](p<-p+dp); [Rjbar.t*Rj ...](p<-p+R*dp)
  Matrix<double, 15, 6> _jacobianOplusPR = Matrix<double, 15, 6>::Zero();

#ifdef USE_P_PLUS_RDP
  const VertexNavStatePR* vNSPR = static_cast<const VertexNavStatePR*>(_vertices[0]);                   // PRj
  _jacobianOplusPR.block<3, 3>(0, 0) = _measurement.getRwb().transpose() * vNSPR->estimate().getRwb();  // p<-p+R*dp
#else
  _jacobianOplusPR.block<3, 3>(0, 0) = Matrix3d::Identity();  // p<-p+dp
#endif

  _jacobianOplusPR.block<3, 3>(3, 3) = Sophus::SO3exd::JacobianRInv(_error.segment<3>(3));  // P(R)VBgBa
  Matrix<double, 15, 3> _jacobianOplusV = Matrix<double, 15, 3>::Zero();                    // J_ej_dv=I
  _jacobianOplusV.block<3, 3>(6, 0) = Matrix3d::Identity();                                 // wrong 6 in JingWang code
  Matrix<double, 15, 6> _jacobianOplusBias = Matrix<double, 15, 6>::Zero();                 // j_ej_db=I
  _jacobianOplusBias.block<3, 3>(9, 0) = Matrix3d::Identity();
  _jacobianOplusBias.block<3, 3>(12, 3) = Matrix3d::Identity();

  _jacobianOplus[0] = _jacobianOplusPR;
  _jacobianOplus[1] = _jacobianOplusV;
  _jacobianOplus[2] = _jacobianOplusBias;
}
void EdgeNavStatePriorPVRBias::computeError() {
  const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);     // PVRj
  const VertexNavStateBias* vNSBias = static_cast<const VertexNavStateBias*>(_vertices[1]);  // Bj
  const NavState& nsPVRest = vNSPVR->estimate();
  const NavState& nsBiasest = vNSBias->estimate();
  // P V R bg=bg_bar+dbg ba=ba_bar+dba
  Vector15d err = Vector15d::Zero();
  const Matrix3d Rbw_bar = _measurement.getRwb().transpose();  //_measurement.mRwb.inverse();
  // NOTICE! VIORBSLAM paper wrong in er's sign with the left, so orb3 fix this problem, while vieoslam old version
  // missed er.t*Omega*ep term in H/prior's Info so it seems no problem, but it'll have problem in fast motion dataset!
#ifdef USE_P_PLUS_RDP
  // Rwb.t() for twb<-twb+Rwb*dtwb, where H is for dtwb
  err.segment<3>(0) = Rbw_bar * (nsPVRest.mpwb - _measurement.mpwb);  // ep=Rbw(pwbj-pwbj_bar)
//  err.segment<3>(0) = _measurement.mRwb.inverse() * (nsPVRest.mpwb - _measurement.mpwb);  // ep=Rbw(pwbj-pwbj_bar)
#else
  err.segment<3>(0) = nsPVRest.mpwb - _measurement.mpwb;  // ep=pwbj-pwbj_bar
#endif
  err.segment<3>(6) = Sophus::SO3exd::Log(Rbw_bar * nsPVRest.getRwb());  // eR=Log(Rwbj_bar.t()*Rwbj)
  //  err.segment<3>(6) = (_measurement.mRwb.inverse() * nsPVRest.mRwb).log();  // eR=Log(Rwbj_bar.t()*Rwbj)
  err.segment<3>(3) = nsPVRest.mvwb - _measurement.mvwb;                                        // ev=vwbj-vwbj_bar
  err.segment<3>(9) = nsBiasest.mbg + nsBiasest.mdbg - (_measurement.mbg + _measurement.mdbg);  // eb=bj-bj_bar
  err.segment<3>(12) = nsBiasest.mba + nsBiasest.mdba - (_measurement.mba + _measurement.mdba);
  _error = err;
}
void EdgeNavStatePriorPVRBias::linearizeOplus() {
  _jacobianOplusXi = Matrix<double, 15, 9>::Zero();  // J_ej_dPVRj=[-I -I Jrinv(phi_eRj)](p<-p+dp); [-Rj ...](p<-p+R*dp)

#ifdef USE_P_PLUS_RDP
  const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);                 // PVRj
  _jacobianOplusXi.block<3, 3>(0, 0) = _measurement.getRwb().transpose() * vNSPVR->estimate().getRwb();  // p<-p+R*dp
#else
  _jacobianOplusXi.block<3, 3>(0, 0) = Matrix3d::Identity();  // p<-p+dp
#endif

  _jacobianOplusXi.block<3, 3>(3, 3) = Matrix3d::Identity();
  _jacobianOplusXi.block<3, 3>(6, 6) = Sophus::SO3exd::JacobianRInv(_error.segment<3>(6));  // PV(R)BgBa
  _jacobianOplusXj = Matrix<double, 15, 6>::Zero();                                         // J_ej_db=I
  _jacobianOplusXj.block<3, 3>(9, 0) = Matrix3d::Identity();
  _jacobianOplusXj.block<3, 3>(12, 3) = Matrix3d::Identity();
}

void EdgeGyrBiasPrior::computeError() {
  const VertexGyrBias* vBiasi = static_cast<const VertexGyrBias*>(_vertices[0]);
  const VertexGyrBias* vBiasj = static_cast<const VertexGyrBias*>(_vertices[1]);
  const Vector3d& bgi = vBiasi->estimate();
  const Vector3d& bgj = vBiasj->estimate();

  _error = bgj - bgi;
}
void EdgeGyrBiasPrior::linearizeOplus() {
  _jacobianOplusXi = -Matrix3d::Identity();  // J_eb_bi=-I for eb=bj-bi
  _jacobianOplusXj = Matrix3d::Identity();   // J_eb_bj=I
}

}  // namespace g2o
