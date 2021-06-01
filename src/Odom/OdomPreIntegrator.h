//created by zzh, inspired by Jing Wang
#ifndef ODOMPREINTEGRATOR_H
#define ODOMPREINTEGRATOR_H

#include <list>
#include "OdomData.h"
#include "so3_extra.h"//for IMUPreIntegratorBase::PreIntegration

#include <iostream>

namespace VIEO_SLAM{

    using Eigen::Quaterniond;
    using Eigen::Matrix;
  
template<class _OdomData>
class OdomPreIntegratorBase{//base class
  OdomPreIntegratorBase(const OdomPreIntegratorBase &pre){}//don't want the list to be copied (e.g. by derived class)
  OdomPreIntegratorBase& operator=(const OdomPreIntegratorBase &other){;return *this;}//do nothing, don't want the list to be assigned in any situation, this makes the derived class unable to use default =!
  
protected:
  listeig(_OdomData) mlOdom;//for IMUPreIntegrator: IMU list
  
public:
  double mdeltatij;//0 means not preintegrated
  
  OdomPreIntegratorBase():mdeltatij(0){}
  //though copy constructor/operator = already deep, please don't copy the list when preintegration is not related to the statei...j
  virtual ~OdomPreIntegratorBase(){}
  // Odom PreIntegration List Setting
  virtual void SetPreIntegrationList(const typename listeig(_OdomData)::const_iterator &begin,typename listeig(_OdomData)::const_iterator pback){
    mlOdom.clear();
    mlOdom.insert(mlOdom.begin(),begin,++pback);
  }
  const listeig(_OdomData)& getlOdom(){return mlOdom;}//the list of Odom, for KFCulling()
  // Odom PreIntegration
  virtual void PreIntegration(const double timeStampi,const double timeStampj){assert(0&&"You called an empty virtual function!!!");}//cannot use =0 for we allow transformed in derived class
  
  // normalize to avoid numerical error accumulation
  inline Quaterniond normalizeRotationQ(const Quaterniond& r) const
  {
    Quaterniond _r(r);
    if (_r.w()<0)//is this necessary?
    {
	_r.coeffs() *= -1;
    }
    return _r.normalized();
  }
  inline Matrix3d normalizeRotationM(const Matrix3d& R) const
  {
    Quaterniond qr(R);
    return normalizeRotationQ(qr).toRotationMatrix();
  }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Eigen::Matrix<double, 6, 1> Vector6d;

//next derived classes don't use operator=!
class EncPreIntegrator:public OdomPreIntegratorBase<EncData>{
  //mlOdom: mlOdomEnc list for vl,vr& its own timestamp
public:
  Vector6d mdelxEij;// delta~Phiij(3*1),delta~pij(3*1) from Encoder PreIntegration, 6*1*float
  Matrix6d mSigmaEij;// by Enc, 6*6*float
  
  EncPreIntegrator():mdelxEij(Vector6d::Zero()),mSigmaEij(Matrix6d::Zero()){}
  EncPreIntegrator(const EncPreIntegrator &pre):mdelxEij(pre.mdelxEij),mSigmaEij(pre.mSigmaEij){mdeltatij=pre.mdeltatij;}//don't copy list!
  EncPreIntegrator& operator=(const EncPreIntegrator &pre){
    mdeltatij=pre.mdeltatij;//don't copy list!
    mdelxEij=pre.mdelxEij;mSigmaEij=pre.mSigmaEij;
    return *this;
  }
  void PreIntegration(const double &timeStampi,const double &timeStampj,
		      const listeig(EncData)::const_iterator &iterBegin,const listeig(EncData)::const_iterator &iterEnd);//rewrite
  void PreIntegration(const double &timeStampi,const double &timeStampj){PreIntegration(timeStampi,timeStampj,mlOdom.begin(),mlOdom.end());}//rewrite, inline
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

template<class IMUDataBase>
class IMUPreIntegratorBase:public OdomPreIntegratorBase<IMUDataBase>{//refer the IMUPreintergrator.cpp by JingWang, so use PVR/PRV Cov.
public:
  Matrix3d mRij;//deltaR~ij(bgi_bar) by awIMU, 3*3*float/delta~Rbibj
  Vector3d mvij,mpij;//deltav~ij,deltap~ij(bi_bar)
  Matrix9d mSigmaijPRV;//Cov_p_Phi_v_ij, a bit different with paper for convenience
  Matrix9d mSigmaij;//Cov_p_v_Phi_ij, opposite order with the paper
  // jacobian of delta measurements w.r.t bias of gyro/acc
  Matrix3d mJgpij;	// position / gyro, Jgdeltapij(bi_bar) in VIORBSLAM paper, par(deltapij)/par(bgi).t()(bi_bar) in Preintegration Paper, we may drop .t() later
  Matrix3d mJapij;	// position / acc
  Matrix3d mJgvij;	// velocity / gyro
  Matrix3d mJavij;	// velocity / acc
  Matrix3d mJgRij;	// rotation / gyro
  //Vector3d mbgij,mbaij;//bg(ti)=b_bar_gi+db_gi, bg~ij not exists, neither do ba~ij
  //Matrix3d mSigmabgd,mSigmabad;//should be deltatij*Cov(eta_bg),deltatij*Cov(eta_ba)
  //for lower imu frequency test
    Matrix3d mRij_hf;//deltaR~ij(bgi_bar) by awIMU, 3*3*float/delta~Rbibj
    Vector3d mvij_hf,mpij_hf;//deltav~ij,deltap~ij(bi_bar)
    double mdt_hf, mdt_hf_ref;
  
  IMUPreIntegratorBase():mRij(Matrix3d::Identity()),mvij(0,0,0),mpij(0,0,0),mSigmaijPRV(Matrix9d::Zero()),mSigmaij(Matrix9d::Zero()){
    mJgpij.setZero();mJapij.setZero();mJgvij.setZero();mJavij.setZero();mJgRij.setZero();
  }
  IMUPreIntegratorBase(const IMUPreIntegratorBase &pre):mRij(pre.mRij),mvij(pre.mvij),mpij(pre.mpij),mSigmaijPRV(pre.mSigmaijPRV),mSigmaij(pre.mSigmaij),
  mJgpij(pre.mJgpij),mJapij(pre.mJapij),mJgvij(pre.mJgvij),mJavij(pre.mJavij),mJgRij(pre.mJgRij){
    this->mdeltatij=pre.mdeltatij;//2-phase name lookup used in Derived template class
  }//don't copy list!
  IMUPreIntegratorBase& operator=(const IMUPreIntegratorBase &pre){
    this->mdeltatij=pre.mdeltatij;//don't copy list!
    this->mRij=pre.mRij,this->mvij=pre.mvij,this->mpij=pre.mpij,this->mSigmaijPRV=pre.mSigmaijPRV,this->mSigmaij=pre.mSigmaij,
    this->mJgpij=pre.mJgpij,this->mJapij=pre.mJapij,this->mJgvij=pre.mJgvij,this->mJavij=pre.mJavij,this->mJgRij=pre.mJgRij;
    return *this;
  }
  virtual ~IMUPreIntegratorBase(){}
  
  void PreIntegration(const double &timeStampi,const double &timeStampj,const Vector3d &bgi_bar,const Vector3d &bai_bar,
		      const typename listeig(IMUDataBase)::const_iterator &iterBegin,const typename listeig(IMUDataBase)::const_iterator &iterEnd);//rewrite, like override but different
  void PreIntegration(const double &timeStampi,const double &timeStampj,const Vector3d &bgi_bar,const Vector3d &bai_bar){//inline
    PreIntegration(timeStampi,timeStampj,bgi_bar,bai_bar,this->mlOdom.begin(),this->mlOdom.end());
  }//rewrite
  // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
  void update(const Vector3d& omega, const Vector3d& acc, const double& dt);//don't allow dt<0!
  void update_highfreq(const Vector3d& omega, const Vector3d& acc, const double& dt);//don't allow dt<0!
  
  // reset to initial state
  void reset(){
    mRij.setIdentity();mvij.setZero();mpij.setZero();mSigmaijPRV.setZero();mSigmaij.setZero();
    mJgpij.setZero();mJapij.setZero();mJgvij.setZero();mJavij.setZero();mJgRij.setZero();
    this->mdeltatij=0;//very important!
      mRij_hf.setIdentity();mvij_hf.setZero();mpij_hf.setZero();
      mdt_hf = 0;
      mdt_hf_ref = 1. / 105;//60;//
  }
  
  // exponential map from vec3 to mat3x3 (Rodrigues formula)
  static Matrix3d Expmap(const Vector3d& v){//here is inline, but defined in .cpp is ok for efficiency due to copy elision(default gcc -O2 uses it) when return a temporary variable(NRVO/URVO)
    return Sophus::SO3exd::exp(v).matrix();//here is URVO
  }
};
//when template<>: specialized definition should be defined in .cpp(avoid redefinition) or use inline/static(not good) in .h and template func. in template class can't be specialized(only fully) when its class is not fully specialized
template<class IMUDataBase>
void IMUPreIntegratorBase<IMUDataBase>::PreIntegration(const double &timeStampi,const double &timeStampj,const Vector3d &bgi_bar,const Vector3d &bai_bar,
						       const typename listeig(IMUDataBase)::const_iterator &iterBegin,const typename listeig(IMUDataBase)::const_iterator &iterEnd){
  //TODO: refer to the code by JingWang
  if (iterBegin!=iterEnd&&timeStampi<timeStampj){//default parameter = !mlOdom.empty(); timeStampi may >=timeStampj for Map Reuse
    // Reset pre-integrator first
    reset();
    // remember to consider the gap between the last KF and the first IMU
    // integrate each imu
    IMUDataBase imu_last;
    double t_last;
    for (typename listeig(IMUDataBase)::const_iterator iterj=iterBegin;iterj!=iterEnd;){
      typename listeig(IMUDataBase)::const_iterator iterjm1=iterj++;//iterj-1
      
      // delta time
      double dt,tj,tj_1;
      if (iterjm1==iterBegin) tj_1=timeStampi; else tj_1=iterjm1->mtm;
      if (iterj==iterEnd) tj=timeStampj; else{ tj=iterj->mtm;assert(tj-tj_1>=0);}
      dt=tj-tj_1;
      if (dt==0) continue;//for we use [nearest imu data at timeStampi, nearest but <=timeStampj] or [/(timeStampi,timeStampj], when we concate them in KeyFrameCulling(), dt may be 0
      if (dt>1.5){ this->mdeltatij=0;std::cout<<"CheckIMU!!!"<<std::endl;return;}//for Map Reuse, the edge between last KF of the map and 0th KF of 2nd SLAM should have no odom info (20frames,>=10Hz, 1.5s<=2s is enough for not using MAP_REUSE_RELOC)
      
      //selete/design measurement_j-1
      const IMUDataBase& imu=*iterjm1;//imuj-1 for w~j-1 & a~j-1 chooses imu(tj-1), maybe u can try (imu(tj-1)+imu(tj))/2 or other filter here
//      IMUDataBase imu=*iterjm1, imu_now = iterj != iterEnd ? *iterj : iterjm1 != iterBegin ? imu_last : imu;//(interplot)
//      if (iterj == iterEnd && iterjm1 != iterBegin) {
////          Eigen::AngleAxisd ang_last(imu.mw.norm(),imu.mw.normalized()), ang_now(imu_now.mw.norm(),imu_now.mw.normalized());
//          double rat = (tj - tj_1)/(t_last - tj_1);
//          if (rat > 0 ) {//we could use imu to preintegrate [timeStampi,imu]
////              Eigen::AngleAxisd ang_mid(Eigen::Quaterniond(ang_last).slerp(rat, Eigen::Quaterniond(ang_now)));
////              imu_now.mw = ang_mid.angle() * ang_mid.axis();
//              imu_now.mw = (1 - rat) * imu.mw + rat * imu_now.mw;
//              imu_now.ma = (1 - rat) * imu.ma + rat * imu_now.ma;
//          }
//      }
//      if (iterjm1 == iterBegin && iterj != iterEnd) {
////          Eigen::AngleAxisd ang_last(imu.mw.norm(),imu.mw.normalized()), ang_now(imu_now.mw.norm(),imu_now.mw.normalized());
//          double rat = (tj_1 - iterjm1->mtm)/(tj - iterjm1->mtm);
//          if (rat > 0 ) {
////              Eigen::AngleAxisd ang_mid(Eigen::Quaterniond(ang_last).slerp(rat, Eigen::Quaterniond(ang_now)));
////              imu.mw = ang_mid.angle() * ang_mid.axis();
//              imu.mw = (1 - rat) * imu.mw + rat * imu_now.mw;
//              imu.ma = (1 - rat) * imu.ma + rat * imu_now.ma;
//          }
//      }

      const bool test_lower_freq = false;//true;//
      if (test_lower_freq) {
          if (iterjm1 == iterBegin)
              imu_last = imu;
          mdt_hf += dt;
          update_highfreq(imu.mw - bgi_bar, imu.ma - bai_bar, dt);
          if (mdt_hf >= mdt_hf_ref || iterj == iterEnd) {
              if (iterj == iterEnd)
                std::cout<<"mdt_hf="<<mdt_hf<<std::endl;
              IMUDataBase imu_fake;
              double dt_fake = 1./200;

//              int n = int(mdt_hf / dt_fake) + 1;
//              Eigen::Vector3d alphadt, a0;
//              double coeff[4];//vij=c0*a0+c1*alpha*dt;pij=a0*c2+alpha*dt*c3;
//              double D = 0;
//              if (n > 0) {
//                  double dtf_fake = mdt_hf - (n-1) * dt_fake;
//                  double dt2_fake = dt_fake * dt_fake, dtf2_fake = dtf_fake * dtf_fake, dtdtf_fake = dt_fake * dtf_fake;
//                  double n2 = n * n;
//                  coeff[0] = (n - 1) * dt_fake + dtf_fake;
//                  coeff[1] = (n2 - 3 * n + 2) * dt_fake / 2 + (n - 1) * dtf_fake;
//                  coeff[2] = (n2 - 2 * n + 1) * dt2_fake / 2 + dtf2_fake / 2 + (n - 1) * dtdtf_fake;
//                  coeff[3] = ((2 * n2 * n - 9 * n2 + 13 * n - 6) * dt2_fake / 6 + (n - 1) * dtf2_fake +
//                              (n - 2) * (n - 1) * dtdtf_fake) / 2;
//                  D = coeff[0] * coeff[3] - coeff[1] * coeff[2];
//              } else {
//                  std::cout<<"Error: n=0"<<std::endl;
//              }
//              if (!D) {
//                  alphadt.setZero();
//                  a0 = mvij_hf / mdt_hf;
//              } else {
//                  a0 = (coeff[3] * mvij_hf - coeff[1] * mpij_hf) / D;
//                  alphadt = (coeff[0] * mpij_hf - coeff[2] * mvij_hf) / D;
//              }

              imu_fake.mw = Sophus::SO3exd(mRij_hf).log() / mdt_hf;
//              imu_fake.ma = a0;
              imu_fake.ma = mvij_hf / mdt_hf;
//              imu_fake.ma = mpij_hf/(mdt_hf*mdt_hf/2);
              std::cout<<"ma = "<< mvij_hf / mdt_hf << " map = "<<mpij_hf/(mdt_hf*mdt_hf/2)<<std::endl;
              std::cout<<"before i_p_i_j = Rij-1 * j-1(j)_p_j-1_j + p_i_j-1(j) = "<< (mRij * mpij_hf + mpij + mvij * mdt_hf).transpose() << std::endl;
              std::cout<<"i_v_i_j = "<<(mRij * mvij_hf + mvij).transpose()<<std::endl;
              std::cout<<mRij * mRij_hf<<std::endl;

//              double t_tmp = 0;
//              Matrix3d mRij_1_hf = mRij;
//              while (t_tmp < mdt_hf) {
//                  if (t_tmp + dt_fake >= mdt_hf)
//                      dt_fake = mdt_hf - t_tmp;
//                  update(imu_fake.mw, mRij.transpose() * mRij_1_hf * imu_fake.ma, dt_fake);
//                  t_tmp += dt_fake;
////                  imu_fake.ma += alphadt;
//              }
              update(imu_fake.mw, imu_fake.ma, mdt_hf);
//              update((imu_last.mw + imu.mw)/2 - bgi_bar, (imu_last.ma + imu.ma)/2 - bai_bar, mdt_hf);
//              update(imu.mw - bgi_bar, imu.ma - bai_bar, mdt_hf);
              imu_last = imu;

              std::cout<<"after i_p_i_j = "<< mpij.transpose() <<std::endl;
              std::cout<<"i_v_i_j = "<< mvij.transpose()<<std::endl;
              std::cout<<mRij<<std::endl;
              mRij_hf.setIdentity();mvij_hf.setZero();mpij_hf.setZero();
              mdt_hf = 0;
          }
          continue;
      }

      // update pre-integrator(interplot)
//      if (iterjm1 == iterBegin) {//we could use imu to preintegrate [timeStampi,imu]
//          double dt_comple = iterjm1->mtm - timeStampi;
//          if (dt_comple > 0) {
//              update(imu.mw - bgi_bar, imu.ma - bai_bar, dt_comple);
//              dt -= dt_comple;
//              if (!dt) continue;
//          }
//      }
//      if (iterj == iterEnd) {
//          if (dt > 0) {
//              imu_now = imu;
//          }
//      }//end speical constant process
//      update((imu_now.mw + imu.mw) / 2 - bgi_bar, (imu_now.ma + imu.ma) / 2 - bai_bar, dt);
//      imu_last = imu;
//      t_last = tj_1;
      // update pre-integrator
      update(imu.mw-bgi_bar,imu.ma-bai_bar,dt);
    }
  }
}
template<class IMUDataBase>
void IMUPreIntegratorBase<IMUDataBase>::update(const Vector3d& omega, const Vector3d& acc, const double& dt){
  using namespace Sophus;
  using namespace Eigen;
  double dt2div2=dt*dt/2;
  Matrix3d dR=Expmap(omega*dt);//Exp((w~j-1 - bgi_bar)*dtj-1j)=delta~Rj-1j
  Matrix3d Jr=SO3exd::JacobianR(omega*dt);//Jrj-1=Jr(dtj-1j*(w~j-1 - bgi_bar))
  Matrix3d skewa=SO3exd::hat(acc);//(~aj-1 - bai_bar)^

  //see paper On-Manifold Preintegration (63), notice PRV is different from paper RVP, but the BgBa is the same(A change row&col, B just change row)
  // err_k+1 = A*err_k + Bg*err_gyro + Ba*err_acc; or Bj-1=[Bg Ba],Aj-1=A 
  Matrix3d I3x3 = Matrix3d::Identity();
  Matrix<double,9,9> A = Matrix9d::Identity();
  A.block<3,3>(3,3) = dR.transpose();
  A.block<3,3>(6,3) = -mRij*skewa*dt;
  A.block<3,3>(0,3) = -mRij*skewa*dt2div2;
  A.block<3,3>(0,6) = I3x3*dt;
  Matrix<double,9,3> Bg = Matrix<double,9,3>::Zero();
  Bg.block<3,3>(3,0) = Jr*dt;
  Matrix<double,9,3> Ba = Matrix<double,9,3>::Zero();
  Ba.block<3,3>(6,0) = mRij*dt;
  Ba.block<3,3>(0,0) = mRij*dt2div2;
    if (IMUDataBase::mdt_cov_noise_fixed)
        mSigmaijPRV=A*mSigmaijPRV*A.transpose()+Bg*IMUDataBase::mSigmag*Bg.transpose()+Ba*IMUDataBase::mSigmaa*Ba.transpose();//notice using Sigma_eta_g/a_d here
    else if (!IMUDataBase::mFreqRef || dt < 1.5 / IMUDataBase::mFreqRef)
        mSigmaijPRV=A*mSigmaijPRV*A.transpose()+Bg*(IMUDataBase::mSigmag/dt)*Bg.transpose()+Ba*(IMUDataBase::mSigmaa/dt)*Ba.transpose();
    else
        mSigmaijPRV=A*mSigmaijPRV*A.transpose()+Bg*(IMUDataBase::mSigmag*IMUDataBase::mFreqRef)*Bg.transpose()+Ba*(IMUDataBase::mSigmaa*IMUDataBase::mFreqRef)*Ba.transpose();
  //the order is opposite(row&col order) for A, opposite row for B
  A = Matrix9d::Identity();
  A.block<3,3>(6,6) = dR.transpose();
  A.block<3,3>(3,6) = -mRij*skewa*dt;
  A.block<3,3>(0,6) = -mRij*skewa*dt2div2;
  A.block<3,3>(0,3) = I3x3*dt;
  Bg = Matrix<double,9,3>::Zero();
  Bg.block<3,3>(6,0) = Jr*dt;
  Ba = Matrix<double,9,3>::Zero();
  Ba.block<3,3>(3,0) = mRij*dt;
  Ba.block<3,3>(0,0) = mRij*dt2div2;
  if (IMUDataBase::mdt_cov_noise_fixed)
    mSigmaij=A*mSigmaij*A.transpose()+Bg*IMUDataBase::mSigmag*Bg.transpose()+Ba*IMUDataBase::mSigmaa*Ba.transpose();
  else if (!IMUDataBase::mFreqRef || dt < 1.5 / IMUDataBase::mFreqRef)
    mSigmaij=A*mSigmaij*A.transpose()+Bg*(IMUDataBase::mSigmag/dt)*Bg.transpose()+Ba*(IMUDataBase::mSigmaa/dt)*Ba.transpose();
  else
    mSigmaij=A*mSigmaij*A.transpose()+Bg*(IMUDataBase::mSigmag*IMUDataBase::mFreqRef)*Bg.transpose()+Ba*(IMUDataBase::mSigmaa*IMUDataBase::mFreqRef)*Ba.transpose();
  
  //see the same paper (69) & use similar iterative rearrange method (59)
  // jacobian of delta measurements w.r.t bias of gyro/acc, for motion_update_with_dbi & residual error & J_error_dxi,xj calculation
  // update P first, then V, then R for using ij as ij-1 term
  mJapij += mJavij*dt - mRij*dt2div2;//mRij here is delta~Rij-1 before its updation
  mJgpij += mJgvij*dt - mRij*skewa*mJgRij*dt2div2;
  mJavij += -mRij*dt;
  mJgvij += -mRij*skewa*mJgRij*dt;//notice except mJgRij use dR, the other Jxxij use mRij!
  mJgRij = dR.transpose()*mJgRij - Jr*dt;//like (59): JgRij=delta~Rj-1j.t()*JgRij-1 - Jrj-1*dtj-1j, the left incremental formula is easy to get for there's no j label
  
  //see paper On-Manifold Preintegration (35~37)
  mpij+=mvij*dt+mRij*(acc*dt2div2);//delta~pij=delta~pij-1 + delta~vij-1*dtj-1j + 1/2*delta~Rij-1*(~aj-1 - bai_bar)*dtj-1j^2
  mvij+=mRij*(acc*dt);//here mRij=mRij-1, delta~vij=delta~vij-1 + delta~Rij-1 * (~aj-1 - bai_bar)*dtj-1j
  // normalize rotation, in case of numerical error accumulation
  mRij=this->normalizeRotationM(mRij*dR);//here omega=(w~k-bgi_bar)(k=j-1), deltaR~ij(bgi_bar)=deltaRij-1(bgi_bar) * Exp((w~j-1 - bgi_bar)*dtj-1j)
  
  this->mdeltatij+=dt;
}

    template<class IMUDataBase>
    void IMUPreIntegratorBase<IMUDataBase>::update_highfreq(const Vector3d& omega, const Vector3d& acc, const double& dt){
        using namespace Sophus;
        using namespace Eigen;
        double dt2div2=dt*dt/2;
        Matrix3d dR=Expmap(omega*dt);//Exp((w~j-1 - bgi_bar)*dtj-1j)=delta~Rj-1j

        //see paper On-Manifold Preintegration (35~37)
        mpij_hf+=mvij_hf*dt+mRij_hf*(acc*dt2div2);//delta~pij=delta~pij-1 + delta~vij-1*dtj-1j + 1/2*delta~Rij-1*(~aj-1 - bai_bar)*dtj-1j^2
        mvij_hf+=mRij_hf*(acc*dt);//here mRij=mRij-1, delta~vij=delta~vij-1 + delta~Rij-1 * (~aj-1 - bai_bar)*dtj-1j
        // normalize rotation, in case of numerical error accumulation
        mRij_hf=this->normalizeRotationM(mRij_hf*dR);//here omega=(w~k-bgi_bar)(k=j-1), deltaR~ij(bgi_bar)=deltaRij-1(bgi_bar) * Exp((w~j-1 - bgi_bar)*dtj-1j)
    }

class IMUPreIntegratorDerived:public IMUPreIntegratorBase<IMUDataDerived>{
public:
  Matrix3d mdelxRji;// delta~Rij.t() from qIMU PreIntegration, 3*3*float
  Matrix3d mSigmaPhiij;// SigmaPhiij by qIMU, 3*3*float

  IMUPreIntegratorDerived():mdelxRji(Matrix3d::Identity()),mSigmaPhiij(Matrix3d::Zero()){}
  void SetPreIntegrationList(const listeig(IMUDataDerived)::const_iterator &begin,const listeig(IMUDataDerived)::const_iterator &pback){//rewrite, will override the base class one
    this->mlOdom.clear();
    this->mlOdom.push_front(*begin);this->mlOdom.push_back(*pback);
  }
  void PreIntegration(const double &timeStampi,const double &timeStampj);//rewrite
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#ifndef TRACK_WITH_IMU
typedef IMUPreIntegratorDerived IMUPreintegrator;
#else
typedef IMUPreIntegratorBase<IMUDataBase> IMUPreintegrator;
#endif

}

#endif
