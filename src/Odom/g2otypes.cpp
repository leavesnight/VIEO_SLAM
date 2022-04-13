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

#include "Converter.h"
using VIEO_SLAM::Converter;

ImuCamPose::ImuCamPose(KeyFrame* pF) : its(0) {
  // Load IMU pose
  auto ns = pF->GetNavState();
  twb = ns.mpwb;
  Rwb = ns.getRwb();

  // Load camera poses
  int num_cams;
  if (pF->mpCameras.size())
    num_cams = pF->mpCameras.size();
  else
    num_cams = 1;

  tcw.resize(num_cams);
  Rcw.resize(num_cams);
  tcb.resize(num_cams);
  Rcb.resize(num_cams);
  Rbc.resize(num_cams);
  tbc.resize(num_cams);
  pCamera.resize(num_cams);

  // Left camera
  tcw[0] = pF->GetTcw().translation();
  Rcw[0] = pF->GetTcw().rotationMatrix();
  tcb[0] = pF->meigtcb;
  Rcb[0] = pF->meigRcb;
  Rbc[0] = Rcb[0].transpose();
  tbc[0] = Converter::toVector3d(pF->mTbc.rowRange(0, 3).col(3));
  CV_Assert(pF->mpCameras.size());
  pCamera[0] = pF->mpCameras[0];
  bf = pF->mbf;

  if (num_cams > 1) {
    Eigen::Matrix4d Trl = Converter::toMatrix4d(pF->mpCameras[1]->GetTcr());
    Rcw[1] = Trl.block<3, 3>(0, 0) * Rcw[0];
    tcw[1] = Trl.block<3, 3>(0, 0) * tcw[0] + Trl.block<3, 1>(0, 3);
    tcb[1] = Trl.block<3, 3>(0, 0) * tcb[0] + Trl.block<3, 1>(0, 3);
    Rcb[1] = Trl.block<3, 3>(0, 0) * Rcb[0];
    Rbc[1] = Rcb[1].transpose();
    tbc[1] = -Rbc[1] * tcb[1];
    pCamera[1] = pF->mpCameras[1];
  }

  // For posegraph 4DoF
  Rwb0 = Rwb;
  DR.setIdentity();
}
ImuCamPose::ImuCamPose(Frame* pF) : its(0) {
  // Load IMU pose
  twb = pF->GetNavStateRef().mpwb;
  Rwb = pF->GetNavStateRef().getRwb();

  // Load camera poses
  int num_cams;
  if (pF->mpCameras.size())
    num_cams = pF->mpCameras.size();
  else
    num_cams = 1;

  tcw.resize(num_cams);
  Rcw.resize(num_cams);
  tcb.resize(num_cams);
  Rcb.resize(num_cams);
  Rbc.resize(num_cams);
  tbc.resize(num_cams);
  pCamera.resize(num_cams);

  // Left camera
  tcw[0] = Converter::toVector3d(pF->GetTcwRef().rowRange(0, 3).col(3));
  Rcw[0] = Converter::toMatrix3d(pF->GetTcwRef().rowRange(0, 3).colRange(0, 3));
  tcb[0] = pF->meigtcb;
  Rcb[0] = pF->meigRcb;
  Rbc[0] = Rcb[0].transpose();
  tbc[0] = Converter::toVector3d(pF->mTbc.rowRange(0, 3).col(3));
  CV_Assert(pF->mpCameras.size());
  pCamera[0] = pF->mpCameras[0];
  bf = pF->mbf;

  if (num_cams > 1) {
    Eigen::Matrix4d Trl = Converter::toMatrix4d(pF->mpCameras[1]->GetTcr());
    Rcw[1] = Trl.block<3, 3>(0, 0) * Rcw[0];
    tcw[1] = Trl.block<3, 3>(0, 0) * tcw[0] + Trl.block<3, 1>(0, 3);
    tcb[1] = Trl.block<3, 3>(0, 0) * tcb[0] + Trl.block<3, 1>(0, 3);
    Rcb[1] = Trl.block<3, 3>(0, 0) * Rcb[0];
    Rbc[1] = Rcb[1].transpose();
    tbc[1] = -Rbc[1] * tcb[1];
    pCamera[1] = pF->mpCameras[1];
  }

  // For posegraph 4DoF
  Rwb0 = Rwb;
  DR.setIdentity();
}
void ImuCamPose::Update(const double* pu) {
  Eigen::Vector3d ur, ut;
  ur << pu[0], pu[1], pu[2];
  ut << pu[3], pu[4], pu[5];

  // Update body pose
  twb += Rwb * ut;
  Rwb = Rwb * Sophus::SO3exd::Exp(ur);

  // Normalize rotation after 5 updates
  its++;
  if (its >= 1) {  // 3) {
    Sophus::SO3exd::NormalizeRotation(Rwb);
    its = 0;
  }

  // Update camera poses
  const Eigen::Matrix3d Rbw = Rwb.transpose();
  const Eigen::Vector3d tbw = -Rbw * twb;

  for (int i = 0; i < pCamera.size(); i++) {
    Rcw[i] = Rcb[i] * Rbw;
    tcw[i] = Rcb[i] * tbw + tcb[i];
  }
}
VertexVelocity::VertexVelocity(KeyFrame* pF) { setEstimate(pF->GetNavState().mvwb); }
VertexVelocity::VertexVelocity(Frame* pF) { setEstimate(pF->GetNavStateRef().mvwb); }
Eigen::Vector2d ImuCamPose::Project(const Eigen::Vector3d& Xw, int cam_idx) const {
  Eigen::Vector3d Xc = Rcw[cam_idx] * Xw + tcw[cam_idx];

  return pCamera[cam_idx]->project(Xc);
}
bool ImuCamPose::isDepthPositive(const Eigen::Vector3d& Xw, int cam_idx) const {
  return (Rcw[cam_idx].row(2) * Xw + tcw[cam_idx](2)) > 0.0;
}
void EdgeMonoOnlyPose::linearizeOplus() {
  const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

  const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
  const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
  const Eigen::Vector3d Xc = Rcw * Xw + tcw;
  const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
  const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

  Eigen::Matrix<double, 2, 3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

  Eigen::Matrix<double, 3, 6> SE3deriv;
  double x = Xb(0);
  double y = Xb(1);
  double z = Xb(2);
  SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0, 0.0, 0.0, 1.0;
  _jacobianOplusXi = proj_jac * Rcb * SE3deriv;  // symbol different becasue of update mode
}
using g2o::VertexGyrBias;
using Matrix9d = Eigen::Matrix<double, 9, 9>;
EdgeInertial::EdgeInertial(VIEO_SLAM::IMUPreintegrator* pInt, const Vector3d& g_in, const Vector3d& bg_in,
                           const Vector3d& ba_in)
    : mbg(bg_in),
      mba(ba_in),
      JRg(pInt->mJgRij),
      JVg(pInt->mJgvij),
      JPg(pInt->mJgpij),
      JVa(pInt->mJavij),
      JPa(pInt->mJapij),
      mpInt(pInt),
      dt(pInt->mdeltatij) {
  // This edge links 6 vertices
  resize(6);
  g = g_in;
  Matrix9d Info = pInt->mSigmaijPRV;
  //  cout << "SigmaPRV="<<Info<<endl;
  Matrix3d pp = Info.block<3, 3>(0, 0);
  Matrix<double, 3, 6> prv = Info.block<3, 6>(0, 3);
  Matrix<double, 6, 3> rvp = Info.block<6, 3>(3, 0);
  Info.block<6, 6>(0, 0) = Info.block<6, 6>(3, 3).eval();
  Info.block<3, 3>(6, 6) = pp;
  Info.block<6, 3>(0, 6) = rvp;
  Info.block<3, 6>(6, 0) = prv;
  //  cout << "SigmaRVP="<<Info<<endl;
  Info = Info.inverse();  //.rowRange(0, 9).colRange(0, 9).inv(cv::DECOMP_SVD);
  // PRV2RVP
  Info = (Info + Info.transpose()) / 2;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9> > es(Info);
  Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
  for (int i = 0; i < 9; i++)
    if (eigs[i] < 1e-12) eigs[i] = 0;
  Info = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
  setInformation(Info);
  //  cout << "check imu edge:"<<endl;
  //  cout << "g="<<g.transpose()<<endl;
  //  cout << "JRg="<<JRg<<endl;
  //  cout << "imu JRg="<<mpInt->mJgRij<<endl;
  //  cout << "Info="<<information()<<endl;
  //  cout << "dt="<<dt<<endl;
  //  cout << endl;
  //  cout << "bg ba="<<mbg<<","<<mba<<endl;
}
Matrix3d EdgeInertial::GetDeltaRotation(const Vector3d& bg) {
  // std::unique_lock<std::mutex> lock(mMutex);
  return Sophus::SO3exd::NormalizeRotation(mpInt->mRij * Sophus::SO3exd::Exp(JRg * (bg - mbg)));
}
Vector3d EdgeInertial::GetDeltaVelocity(const Vector3d& bg, const Vector3d& ba) {
  // std::unique_lock<std::mutex> lock(mMutex);
  Vector3d dbg = bg - mbg, dba = ba - mba;
  return mpInt->mvij + JVg * dbg + JVa * dba;
}
Vector3d EdgeInertial::GetDeltaPosition(const Vector3d& bg, const Vector3d& ba) {
  // std::unique_lock<std::mutex> lock(mMutex);
  Vector3d dbg = bg - mbg, dba = ba - mba;
  return mpInt->mpij + JPg * dbg + JPa * dba;
}
void EdgeInertial::computeError() {
  // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too
  // big
  const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyrBias* VG1 = static_cast<const VertexGyrBias*>(_vertices[2]);
  const VertexGyrBias* VA1 = static_cast<const VertexGyrBias*>(_vertices[3]);
  const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
  const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const Vector3d bg(VG1->estimate()[0], VG1->estimate()[1], VG1->estimate()[2]),
      ba(VA1->estimate()[0], VA1->estimate()[1], VA1->estimate()[2]);
  const Eigen::Matrix3d dR = GetDeltaRotation(bg);
  const Eigen::Vector3d dV = GetDeltaVelocity(bg, ba);
  const Eigen::Vector3d dP = GetDeltaPosition(bg, ba);

  const Eigen::Vector3d er =
      Sophus::SO3exd::Log(dR.transpose() * VP1->estimate().Rwb.transpose() * VP2->estimate().Rwb);
  const Eigen::Vector3d ev = VP1->estimate().Rwb.transpose() * (VV2->estimate() - VV1->estimate() - g * dt) - dV;
  const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose() *
                                 (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt - g * dt * dt / 2) -
                             dP;

  _error << er, ev, ep;
}
//#define USE_ORB3_MATH
#ifdef USE_ORB3_MATH
Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z) {
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);

  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5) {
    return Eigen::Matrix3d::Identity();
  } else {
    return Eigen::Matrix3d::Identity() - W * (1.0 - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
  }
}
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z) {
  const double d2 = x * x + y * y + z * z;
  const double d = sqrt(d2);

  Eigen::Matrix3d W;
  W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
  if (d < 1e-5)
    return Eigen::Matrix3d::Identity();
  else
    return Eigen::Matrix3d::Identity() + W / 2 + W * W * (1.0 / d2 - (1.0 + cos(d)) / (2.0 * d * sin(d)));
}
#endif
Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v) {
#ifndef USE_ORB3_MATH
  return Sophus::SO3exd::JacobianRInv(v);
#else
  return InverseRightJacobianSO3(v[0], v[1], v[2]);
#endif
}
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v) {
#ifndef USE_ORB3_MATH
  return Sophus::SO3exd::JacobianR(v);
#else
  return RightJacobianSO3(v[0], v[1], v[2]);
#endif
}
void EdgeInertial::linearizeOplus() {
  const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyrBias* VG1 = static_cast<const VertexGyrBias*>(_vertices[2]);
  const VertexGyrBias* VA1 = static_cast<const VertexGyrBias*>(_vertices[3]);
  const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
  const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const Vector3d bg(VG1->estimate()[0], VG1->estimate()[1], VG1->estimate()[2]);
  const Vector3d dbg = bg - mbg;

  const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
  const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
  const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;

  const Eigen::Matrix3d dR = GetDeltaRotation(bg);
  const Eigen::Matrix3d eR = dR.transpose() * Rbw1 * Rwb2;
  const Eigen::Vector3d er = Sophus::SO3exd::Log(eR);
  const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

  // Jacobians wrt Pose 1
  _jacobianOplus[0].setZero();
  // rotation
  _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;                                          // OK
  _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3exd::hat(Rbw1 * (VV2->estimate() - VV1->estimate() - g * dt));  // OK
  _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3exd::hat(
      Rbw1 * (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt - 0.5 * g * dt * dt));  // OK
  // translation
  _jacobianOplus[0].block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();  // OK

  // Jacobians wrt Velocity 1
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(3, 0) = -Rbw1;       // OK
  _jacobianOplus[1].block<3, 3>(6, 0) = -Rbw1 * dt;  // OK

  // Jacobians wrt Gyro 1
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(0, 0) = -invJr * eR.transpose() * RightJacobianSO3(JRg * dbg) * JRg;  // OK
  _jacobianOplus[2].block<3, 3>(3, 0) = -JVg;                                                         // OK
  _jacobianOplus[2].block<3, 3>(6, 0) = -JPg;                                                         // OK

  // Jacobians wrt Accelerometer 1
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(3, 0) = -JVa;  // OK
  _jacobianOplus[3].block<3, 3>(6, 0) = -JPa;  // OK

  // Jacobians wrt Pose 2
  _jacobianOplus[4].setZero();
  // rotation
  _jacobianOplus[4].block<3, 3>(0, 0) = invJr;  // OK
  // translation
  _jacobianOplus[4].block<3, 3>(6, 3) = Rbw1 * Rwb2;  // OK

  // Jacobians wrt Velocity 2
  _jacobianOplus[5].setZero();
  _jacobianOplus[5].block<3, 3>(3, 0) = Rbw1;  // OK
}
EdgePriorPoseImu::EdgePriorPoseImu(ConstraintPoseImu* c) {
  resize(4);
  Rwb = c->Rwb;
  twb = c->twb;
  vwb = c->vwb;
  bg = c->bg;
  ba = c->ba;
  setInformation(c->H);
}
void EdgePriorPoseImu::computeError() {
  const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
  const VertexVelocity* VV = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyrBias* VG = static_cast<const VertexGyrBias*>(_vertices[2]);
  const VertexGyrBias* VA = static_cast<const VertexGyrBias*>(_vertices[3]);

  const Eigen::Vector3d er = Sophus::SO3exd::Log(Rwb.transpose() * VP->estimate().Rwb);
  // Rwb.t() for twb<-twb+Rwb*dtwb, where H is for dtwb
  const Eigen::Vector3d et = Rwb.transpose() * (VP->estimate().twb - twb);
  const Eigen::Vector3d ev = VV->estimate() - vwb;
  const Eigen::Vector3d ebg = VG->estimate() - bg;
  const Eigen::Vector3d eba = VA->estimate() - ba;

  _error << er, et, ev, ebg, eba;
}
void EdgePriorPoseImu::linearizeOplus() {
  const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
  const Eigen::Vector3d er = Sophus::SO3exd::Log(Rwb.transpose() * VP->estimate().Rwb);
  _jacobianOplus[0].setZero();
  _jacobianOplus[0].block<3, 3>(0, 0) = InverseRightJacobianSO3(er);
  _jacobianOplus[0].block<3, 3>(3, 3) = Rwb.transpose() * VP->estimate().Rwb;
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
}
