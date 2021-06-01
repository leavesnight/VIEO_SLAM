#include "g2otypes.h"

namespace g2o {
    using namespace VIEO_SLAM;

    void EdgeNavStateBias::computeError() {
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[0]);
        const VertexNavStateBias *vBiasj = static_cast<const VertexNavStateBias *>(_vertices[1]);
        const NavState &NSi = vBiasi->estimate();
        const NavState &NSj = vBiasj->estimate();

        //rbij/eb=bj-bi see VIORBSLAM paper (6) or Manifold paper (48)
        // residual error of Gyroscope's bias, Forster 15'RSS
        Vector3d rBiasG = (NSj.mbg + NSj.mdbg) - (NSi.mbg + NSi.mdbg);
        // residual error of Accelerometer's bias, Forster 15'RSS
        Vector3d rBiasA = (NSj.mba + NSj.mdba) - (NSi.mba + NSi.mdba);//here is x(x)xa!!!
        Vector6d err(Vector6d::Zero());
        // 6-Dim error vector order: deltabiasGyr_i;deltabiasAcc_i, rBiasGi;rBiasAi
        err.segment<3>(0) = rBiasG;
        err.segment<3>(3) = rBiasA;
        _error = err;
    }
    void EdgeNavStateBias::linearizeOplus() {
        _jacobianOplusXi = -Matrix<double, 6, 6>::Identity();//J_eb_bi=-I for eb=bj-bi
        _jacobianOplusXj = Matrix<double, 6, 6>::Identity();//J_eb_bj=I
    }

    typedef Matrix<double, 15, 1> Vector15d;
void EdgeNavStatePriorPRVBias::computeError()
{
    const VertexNavStatePR* vNSPR=static_cast<const VertexNavStatePR*>(_vertices[0]);//PRj
    const VertexNavStateV* vNSV=static_cast<const VertexNavStateV*>(_vertices[1]);//Vj
    const VertexNavStateBias* vNSBias=static_cast<const VertexNavStateBias*>(_vertices[2]);//Bj
    const NavState& nsPRest=vNSPR->estimate();const NavState& nsVest=vNSV->estimate();const NavState& nsBiasest=vNSBias->estimate();
    
    // P R V bg=bg_bar+dbg ba=ba_bar+dba, see VIORBSLAM paper (8)
    Vector15d err=Vector15d::Zero();
    err.segment<3>(0)=_measurement.mpwb-nsPRest.mpwb;//ep=pwbj_bar-pwbj
    err.segment<3>(3)=(_measurement.mRwb.inverse()*nsPRest.mRwb).log();//eR=Log(Rwbj_bar.t()*Rwbj)
    err.segment<3>(6)=_measurement.mvwb-nsVest.mvwb;//ev=vwbj_bar-vwbj
    err.segment<3>(9)=_measurement.mbg+_measurement.mdbg-(nsBiasest.mbg+nsBiasest.mdbg);//eb=bj_bar-bj
    err.segment<3>(12)=_measurement.mba+_measurement.mdba-(nsBiasest.mba+nsBiasest.mdba);
    _error = err;
}
void EdgeNavStatePriorPRVBias::linearizeOplus()
{
    Matrix<double,15,6> _jacobianOplusPR = Matrix<double,15,6>::Zero();//J_ej_dPRj=[-I Jrinv(phi_eRj)](p<-p+dp); [-Rj ...](p<-p+R*dp)
    
//     const VertexNavStatePR* vNSPR=static_cast<const VertexNavStatePR*>(_vertices[0]);//PRj
//     _jacobianOplusPR.block<3,3>(0,0)= - vNSPR->estimate().getRwb();//p<-p+R*dp
    
    _jacobianOplusPR.block<3,3>(0,0)= - Matrix3d::Identity();//p<-p+dp
    
    _jacobianOplusPR.block<3,3>(3,3)=Sophus::SO3exd::JacobianRInv( _error.segment<3>(3));//P(R)VBgBa
    Matrix<double,15,3> _jacobianOplusV = Matrix<double,15,3>::Zero();//J_ej_dv=-I
    _jacobianOplusV.block<3,3>(6,0)= - Matrix3d::Identity();//wrong 6 in JingWang code
    Matrix<double,15,6> _jacobianOplusBias = Matrix<double,15,6>::Zero();//j_ej_db=-I
    _jacobianOplusBias.block<3,3>(9,0)= - Matrix3d::Identity();_jacobianOplusBias.block<3,3>(12,3)= - Matrix3d::Identity();

    _jacobianOplus[0] = _jacobianOplusPR;_jacobianOplus[1] = _jacobianOplusV;
    _jacobianOplus[2] = _jacobianOplusBias;
}
void EdgeNavStatePriorPVRBias::computeError()
{
    const VertexNavStatePVR* vNSPVR=static_cast<const VertexNavStatePVR*>(_vertices[0]);//PVRj
    const VertexNavStateBias* vNSBias=static_cast<const VertexNavStateBias*>(_vertices[1]);//Bj
    const NavState& nsPVRest=vNSPVR->estimate();const NavState& nsBiasest=vNSBias->estimate();
    // P V R bg=bg_bar+dbg ba=ba_bar+dba
    Vector15d err=Vector15d::Zero();
    err.segment<3>(0)=_measurement.mpwb-nsPVRest.mpwb;//ep=pwbj_bar-pwbj
    err.segment<3>(6)=(_measurement.mRwb.inverse()*nsPVRest.mRwb).log();//eR=Log(Rwbj_bar.t()*Rwbj)
    err.segment<3>(3)=_measurement.mvwb-nsPVRest.mvwb;//ev=vwbj_bar-vwbj
    err.segment<3>(9)=_measurement.mbg+_measurement.mdbg-(nsBiasest.mbg+nsBiasest.mdbg);//eb=bj_bar-bj
    err.segment<3>(12)=_measurement.mba+_measurement.mdba-(nsBiasest.mba+nsBiasest.mdba);
    _error = err;
}
void EdgeNavStatePriorPVRBias::linearizeOplus()
{
    _jacobianOplusXi= Matrix<double,15,9>::Zero();//J_ej_dPVRj=[-I -I Jrinv(phi_eRj)](p<-p+dp); [-Rj ...](p<-p+R*dp)
    
//     const VertexNavStatePVR* vNSPVR=static_cast<const VertexNavStatePVR*>(_vertices[0]);//PVRj
//     _jacobianOplusXi.block<3,3>(0,0)= - vNSPVR->estimate().getRwb();//p<-p+R*dp
    
    _jacobianOplusXi.block<3,3>(0,0)= - Matrix3d::Identity();//p<-p+dp
    
    _jacobianOplusXi.block<3,3>(3,3)= - Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(6,6)=Sophus::SO3exd::JacobianRInv( _error.segment<3>(6));//PV(R)BgBa
    _jacobianOplusXj = Matrix<double,15,6>::Zero();//J_ej_db=-I
    _jacobianOplusXj.block<3,3>(9,0)= - Matrix3d::Identity();_jacobianOplusXj.block<3,3>(12,3)= - Matrix3d::Identity();
}

void EdgeEnc::computeError()
{
  const VertexSE3Expmap* vi=static_cast<const VertexSE3Expmap*>(_vertices[0]);
  const VertexSE3Expmap* vj=static_cast<const VertexSE3Expmap*>(_vertices[1]);
  Quaterniond qRiw=vi->estimate().rotation(),qRjw=vj->estimate().rotation();
  Vector3d piw=vi->estimate().translation(),pjw=vj->estimate().translation();
  Sophus::SO3exd so3Reiej=Sophus::SO3exd(qRce.conjugate()*qRiw*qRjw.conjugate()*qRce);
  _error.segment<3>(0)=Sophus::SO3exd(Sophus::SO3exd::exp(_measurement.segment<3>(0)).inverse()*so3Reiej).log();//Log(delta~Rij.t()*Reiej)
  Vector3d deltapij=qRce.conjugate()*(piw-qRiw*qRjw.conjugate()*pjw-pce+qRiw*qRjw.conjugate()*pce);//Rec*[pciw-Rciw*Rcjw.t()*pcjw-pce+Rciw*Rcjw.t()*pce]
  _error.segment<3>(3)=deltapij-_measurement.segment<3>(3);//deltapij-delta~pij
}

void EdgeEnc::linearizeOplus()
{
  const VertexSE3Expmap* vi=static_cast<const VertexSE3Expmap*>(_vertices[0]);
  const VertexSE3Expmap* vj=static_cast<const VertexSE3Expmap*>(_vertices[1]);
  Quaterniond qRiw=vi->estimate().rotation(),qRjw=vj->estimate().rotation();
  Vector3d piw=vi->estimate().translation(),pjw=vj->estimate().translation();
  
  //calculate Je_dxi xi=ksi=(phii,rhoi)
  Matrix3d Rec=qRce.conjugate().toRotationMatrix();
  Quaterniond qRij=qRiw*qRjw.conjugate();
  Vector3d eR=_error.segment<3>(0);
  Matrix3d JeR_dphii=Sophus::SO3exd::JacobianRInv(eR)*Rec*qRij.conjugate().toRotationMatrix();//JeR_dphii=Jrinv(eR)*(Rciw*Rwcj*Rce).t()
  Matrix3d O3x3=Matrix3d::Zero();//JeR_drhoi/j=0
  Matrix3d Jep_dphii=Rec*Sophus::SO3exd::hat(qRij*(pjw-pce)-piw);//Jep_dphii=Rec*[Rciw*Rwcj*(pcjw-pce)-pciw]^
  Matrix3d Jep_drhoi=Rec;//Jep_drhoi=Rec
  //calculate Je_dxj xj=ksj=(phij,rhoj)
  Matrix3d JeR_dphij=-Sophus::SO3exd::JacobianRInv(eR)*Rec;//JeR_dphij=-Jrinv(eR)*Rec
  Quaterniond qRecRij=qRce.conjugate()*qRij;
  Matrix3d Jep_dphij=qRecRij.toRotationMatrix()*Sophus::SO3exd::hat(pce);//Jep_dphij=Rec*Rciw*Rwcj*pce^
  Matrix3d Jep_drhoj=-qRecRij.toRotationMatrix();//Jep_drhoj=-Rec*Rciw*Rcjw.t()
  
  _jacobianOplusXi.block<3,3>(0,0)=JeR_dphii;
  _jacobianOplusXi.block<3,3>(0,3)=O3x3;//JeR_drhoi
  _jacobianOplusXi.block<3,3>(3,0)=Jep_dphii;
  _jacobianOplusXi.block<3,3>(3,3)=Jep_drhoi;
  _jacobianOplusXj.block<3,3>(0,0)=JeR_dphij;
  _jacobianOplusXj.block<3,3>(0,3)=O3x3;//JeR_drhoj
  _jacobianOplusXj.block<3,3>(3,0)=Jep_dphij;
  _jacobianOplusXj.block<3,3>(3,3)=Jep_drhoj;
}

}
