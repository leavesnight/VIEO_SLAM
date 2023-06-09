// created by zzh, inspired by Jing Wang
#ifndef ODOMPREINTEGRATOR_H
#define ODOMPREINTEGRATOR_H

#include <list>
#include "OdomData.h"
#include "common/so3_extra.h"  //for IMUPreIntegratorBase::PreIntegration

#include <iostream>

//#define USE_PREINT_EULA

namespace VIEO_SLAM {

using Eigen::Matrix;
using Eigen::Quaterniond;

template <class _OdomData>
class OdomPreIntegratorBase {  // base class
  // don't want the list to be copied (e.g. by derived class)
  OdomPreIntegratorBase(const OdomPreIntegratorBase &pre) {}
  // do nothing, don't want the list to be assigned in any situation, this makes the derived class unable to use
  // default =!
  OdomPreIntegratorBase &operator=(const OdomPreIntegratorBase &other) { return *this; }

 protected:
  listeig(_OdomData) mlOdom;  // for IMUPreIntegrator: IMU list

  virtual void reset() {
    mdeltatij = 0;  // very important!
  }

 public:
  template <class T>
  using aligned_list = Eigen::aligned_list<T>;

  double mdeltatij;  // 0 means not preintegrated
  double &dt_ij_ = mdeltatij;

  OdomPreIntegratorBase() : mdeltatij(0) {}
  // though copy constructor/operator = already deep, please don't copy the list when preintegration is not related to
  // the statei...j
  virtual ~OdomPreIntegratorBase() {}
  // Odom PreIntegration List Setting
  virtual void SetPreIntegrationList(const typename listeig(_OdomData)::const_iterator &begin,
                                     typename listeig(_OdomData)::const_iterator end) {
    mlOdom.clear();
    mlOdom.insert(mlOdom.begin(), begin, end);
  }
  // splice operation (like move) for fast append
  virtual void AppendFrontPreIntegrationList(aligned_list<_OdomData> &x,
                                             const typename aligned_list<_OdomData>::const_iterator &begin,
                                             const typename aligned_list<_OdomData>::const_iterator &end) {
    mlOdom.splice(mlOdom.begin(), x, begin, end);
  }
  const listeig(_OdomData) & getlOdom() const { return mlOdom; }  // the list of Odom, for KFCulling()
  // the list of Odom, for deep proc(like KFCulling())
  aligned_list<_OdomData> &GetRawDataRef() { return mlOdom; }
  // Odom PreIntegration
  virtual void PreIntegration(const double timeStampi, const double timeStampj) {
    assert(0 && "You called an empty virtual function!!!");
  }  // cannot use =0 for we allow transformed in derived class

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Eigen::Matrix<double, 6, 1> Vector6d;

// next derived classes don't use operator=!
class EncPreIntegrator : public OdomPreIntegratorBase<EncData> {
  // mlOdom: mlOdomEnc list for vl,vr& its own timestamp
  void reset() override;
  Eigen::Vector2d eigdeltaPijM;  // deltaPii=0
  double deltaThetaijMz;

 public:
  Vector6d mdelxEij;   // delta~Phiij(3*1),delta~pij(3*1) from Encoder PreIntegration, 6*1*float
  Matrix6d mSigmaEij;  // by Enc, 6*6*float

  EncPreIntegrator() : mdelxEij(Vector6d::Zero()), mSigmaEij(Matrix6d::Zero()) {}
  EncPreIntegrator(const EncPreIntegrator &pre)
      : eigdeltaPijM(pre.eigdeltaPijM),
        deltaThetaijMz(pre.deltaThetaijMz),
        mdelxEij(pre.mdelxEij),
        mSigmaEij(pre.mSigmaEij) {
    mdeltatij = pre.mdeltatij;
  }  // don't copy list!
  EncPreIntegrator &operator=(const EncPreIntegrator &pre) {
    eigdeltaPijM = pre.eigdeltaPijM;
    deltaThetaijMz = pre.deltaThetaijMz;
    mdelxEij = pre.mdelxEij;
    mSigmaEij = pre.mSigmaEij;
    mdeltatij = pre.mdeltatij;  // don't copy list!
    return *this;
  }
  int PreIntegration(const double &timeStampi, const double &timeStampj,
                     const listeig(EncData)::const_iterator &iterBegin, const listeig(EncData)::const_iterator &iterEnd,
                     bool breset = true);  // rewrite
  void PreIntegration(const double &timeStampi, const double &timeStampj) {
    PreIntegration(timeStampi, timeStampj, mlOdom.begin(), mlOdom.end());
  }  // rewrite, inline

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

template <class IMUDataBase>
class IMUPreIntegratorBase
    : public OdomPreIntegratorBase<IMUDataBase> {  // refer the IMUPreintergrator.cpp by JingWang, so use PVR/PRV Cov.
 public:
  typedef double Tcalc;
  using SO3calc = Sophus::SO3ex<Tcalc>;
  Matrix3d mRij;  // deltaR~ij(bgi_bar) by awIMU, 3*3*float/delta~Rbibj
  Matrix3d &Rij_ = mRij;
  Vector3d mvij, mpij;  // deltav~ij,deltap~ij(bi_bar)
  Vector3d &vij_ = mvij, &pij_ = mpij;
  Matrix9d mSigmaijPRV;  // Cov_p_Phi_v_ij, a bit different with paper for convenience
  Matrix9d GetProcessedInfoijPRV() const {
    Matrix9d InfoijPRV = mSigmaijPRV.inverse();
    //    InfoijPRV = (InfoijPRV + InfoijPRV.transpose()) / 2;
    //    Eigen::SelfAdjointEigenSolver<Matrix9d> es(InfoijPRV);
    //    Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
    //    for (int i = 0; i < 9; ++i)  // maybe for fast speed opt.?
    //      if (eigs[i] < 1e-12) eigs[i] = 0;
    //    InfoijPRV = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    return InfoijPRV;
  }
  Matrix9d GetProcessedInfoij() const {
    Matrix9d Infoij = mSigmaij.inverse();
    //    Infoij = (Infoij + Infoij.transpose()) / 2;
    //    Eigen::SelfAdjointEigenSolver<Matrix9d> es(Infoij);
    //    Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
    //    for (int i = 0; i < 9; ++i)
    //      if (eigs[i] < 1e-12) eigs[i] = 0;
    //    Infoij = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    return Infoij;
  }
  Matrix9d mSigmaij;  // Cov_p_v_Phi_ij, opposite order with the paper
  // jacobian of delta measurements w.r.t bias of gyro/acc
  Matrix3d mJgpij;  // position / gyro, Jgdeltapij(bi_bar) in VIORBSLAM paper, par(deltapij)/par(bgi).t()(bi_bar) in
                    // Preintegration Paper, we may drop .t() later
  Matrix3d mJapij;  // position / acc
  Matrix3d mJgvij;  // velocity / gyro
  Matrix3d mJavij;  // velocity / acc
  Matrix3d mJgRij;  // rotation / gyro
  Matrix3d &Japij_ = mJapij, &Jgvij_ = mJgvij, &Javij_ = mJavij, &JgRij_ = mJgRij, &Jgpij_ = mJgpij;
  Matrix9d &SigmaijPRV_ = mSigmaijPRV;

  // Vector3d mbgij,mbaij;//bg(ti)=b_bar_gi+db_gi, bg~ij not exists, neither do ba~ij
  // Matrix3d mSigmabgd,mSigmabad;//should be deltatij*Cov(eta_bg),deltatij*Cov(eta_ba)

  // for lower imu frequency test
  Matrix3d mRij_hf;           // deltaR~ij(bgi_bar) by awIMU, 3*3*float/delta~Rbibj
  Vector3d mvij_hf, mpij_hf;  // deltav~ij,deltap~ij(bi_bar)
  double mdt_hf, mdt_hf_ref;

  IMUPreIntegratorBase()
      : mRij(Matrix3d::Identity()),
        mvij(0, 0, 0),
        mpij(0, 0, 0),
        mSigmaijPRV(Matrix9d::Zero()),
        mSigmaij(Matrix9d::Zero()) {
    mJgpij.setZero();
    mJapij.setZero();
    mJgvij.setZero();
    mJavij.setZero();
    mJgRij.setZero();
  }
  IMUPreIntegratorBase(const IMUPreIntegratorBase &pre)
      : mRij(pre.mRij),
        mvij(pre.mvij),
        mpij(pre.mpij),
        mSigmaijPRV(pre.mSigmaijPRV),
        mSigmaij(pre.mSigmaij),
        mJgpij(pre.mJgpij),
        mJapij(pre.mJapij),
        mJgvij(pre.mJgvij),
        mJavij(pre.mJavij),
        mJgRij(pre.mJgRij) {
    this->mdeltatij = pre.mdeltatij;  // 2-phase name lookup used in Derived template class
  }                                   // don't copy list!
  IMUPreIntegratorBase &operator=(const IMUPreIntegratorBase &pre) {
    this->mRij = pre.mRij, this->mvij = pre.mvij, this->mpij = pre.mpij, this->mSigmaijPRV = pre.mSigmaijPRV,
    this->mSigmaij = pre.mSigmaij, this->mJgpij = pre.mJgpij, this->mJapij = pre.mJapij, this->mJgvij = pre.mJgvij,
    this->mJavij = pre.mJavij, this->mJgRij = pre.mJgRij;
    this->mdeltatij = pre.mdeltatij;  // don't copy list!
    return *this;
  }
  virtual ~IMUPreIntegratorBase() {}

  int PreIntegration(const double &timeStampi, const double &timeStampj, const Vector3d &bgi_bar,
                     const Vector3d &bai_bar, const typename listeig(IMUDataBase)::const_iterator &iterBegin,
                     const typename listeig(IMUDataBase)::const_iterator &iterEnd,
                     bool breset = true);  // rewrite, like override but different
  void PreIntegration(const double &timeStampi, const double &timeStampj, const Vector3d &bgi_bar,
                      const Vector3d &bai_bar) {  // inline
    PreIntegration(timeStampi, timeStampj, bgi_bar, bai_bar, this->mlOdom.begin(), this->mlOdom.end());
  }  // rewrite
  // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
  void update(const Vector3d &omega, const Vector3d &acc, const double &dt);           // don't allow dt<0!
  void update_highfreq(const Vector3d &omega, const Vector3d &acc, const double &dt);  // don't allow dt<0!

  // reset to initial state
  void reset() override {
    OdomPreIntegratorBase<IMUDataBase>::reset();
    mRij.setIdentity();
    mvij.setZero();
    mpij.setZero();
    mSigmaijPRV.setZero();
    mSigmaij.setZero();
    mJgpij.setZero();
    mJapij.setZero();
    mJgvij.setZero();
    mJavij.setZero();
    mJgRij.setZero();
    mRij_hf.setIdentity();
    mvij_hf.setZero();
    mpij_hf.setZero();
    mdt_hf = 0;
    mdt_hf_ref = 1. / 105;  // 60;//
  }
};
// when template<>: specialized definition should be defined in .cpp(avoid redefinition) or use inline/static(not good)
// in .h and template func. in template class can't be specialized(only fully) when its class is not fully specialized
template <class IMUDataBase>
int IMUPreIntegratorBase<IMUDataBase>::PreIntegration(const double &timeStampi, const double &timeStampj,
                                                      const Vector3d &bgi_bar, const Vector3d &bai_bar,
                                                      const typename listeig(IMUDataBase)::const_iterator &iterBegin,
                                                      const typename listeig(IMUDataBase)::const_iterator &iterEnd,
                                                      bool breset) {
  if (iterBegin != iterEnd) {  // default parameter = !mlOdom.empty(); timeStampi may >=timeStampj for Map Reuse
    // Reset pre-integrator first
    if (breset) reset();
    // remember to consider the gap between the last KF and the first IMU
    // integrate each imu

    // for iterBegin!=iterEnd, here iter_stop can no init
    typename listeig(IMUDataBase)::const_iterator iter_start = iterBegin, iter_stop;
    using Ttime = double;
    bool bimu_order_back = timeStampi > timeStampj;
    Ttime timemin = timeStampi, timemax = timeStampj;
    using std::swap;
    if (bimu_order_back) swap(timemin, timemax);
    for (auto iterj = iterBegin; iterj != iterEnd && iterj->mtm <= timemin; iter_start = iterj++) {
    }
    for (auto iterj = iterEnd; iterj != iterBegin;) {
      iter_stop = iterj--;
      if (iterj->mtm >= timemax) continue;
      break;
    }
    if (bimu_order_back) {
      if (iter_stop == iterEnd) {
        --iter_stop;
      }
      swap(iter_start, iter_stop);
      if ((iter_stop)->mtm > timemin) {
        assert(iter_stop == iterBegin);
        iter_stop = iterEnd;
      }
    }

    for (typename listeig(IMUDataBase)::const_iterator iterj = iter_start; iterj != iter_stop;) {
      typename listeig(IMUDataBase)::const_iterator iterjm1 = iterj;  // iterj-1
      if (bimu_order_back) {
        if (iterj == iterBegin) {
          iterj = iter_stop;
        } else
          --iterj;
      } else
        ++iterj;

      // delta time
      double dt, tj, tj_1;
      if (iterjm1 == iter_start)
        tj_1 = timeStampi;
      else
        tj_1 = iterjm1->mtm;
      if (iterj == iter_stop)
        tj = timeStampj;
      else {
        tj = iterj->mtm;
      }
      dt = tj - tj_1;
      // for we use [nearest imu data at timeStampi, nearest but <=timeStampj] or [/(timeStampi,timeStampj],
      // when we concate them in KeyFrameCulling(), dt may be 0
      if (dt == 0) continue;
      // for Map Reuse, the edge between last KF of the map and 0th KF of 2nd SLAM should have no odom info
      // (20frames,>=10Hz, 1.5s<=2s is enough for not using MAP_REUSE_RELOC)
      if (abs(dt) > 1.5) {
        this->mdeltatij = 0;
        std::cout << "CheckIMU!!!" << std::endl;
        return -1;
      }

      // selete/design measurement_j-1
#ifdef USE_PREINT_EULA
      // imuj-1 for w~j-1 & a~j-1 chooses imu(tj-1), maybe u can try (imu(tj-1)+imu(tj))/2 or other filter here
      const IMUDataBase &imu = *iterjm1;
#else
      //(interplot)
      IMUDataBase imu = *iterjm1, imu_now = iterj != iterEnd ? *iterj : imu;
#endif
#ifndef USE_PREINT_EULA
      // when iterj==iterEnd, iterj not exist, no interplot
      // we could use imu to preintegrate [imu, timeStampj]/[timeStampi, timeStampj](1imu)
      if (iterj != iterEnd) {
        if (iterj == iter_stop) {
          Tcalc dt_tmp = (Tcalc)(iterj->mtm - timeStampj);
          // if dt_comple_stop > 0, then dt_tmp < 0, no interplot
          if (bimu_order_back ? dt_tmp < 0 : dt_tmp > 0) {
            Tcalc rat = dt_tmp / (Tcalc)(iterj->mtm - iterjm1->mtm);
            imu_now.mw = rat * imu.mw + (1 - rat) * imu_now.mw;
            imu_now.ma = rat * imu.ma + (1 - rat) * imu_now.ma;
          }
        }
        if (iterjm1 == iter_start) {
          Tcalc dt_tmp = (Tcalc)(timeStampi - iterjm1->mtm);
          // if dt_comple > 0, then dt_tmp < 0, no interplot
          if (bimu_order_back ? dt_tmp < 0 : dt_tmp > 0) {
            // Eigen::AngleAxisd ang_last(imu.mw.norm(),imu.mw.normalized()),
            // ang_now(imu_now.mw.norm(),imu_now.mw.normalized());
            Tcalc rat = dt_tmp / (Tcalc)(iterj->mtm - iterjm1->mtm);
            // Eigen::AngleAxisd ang_mid(Eigen::Quaterniond(ang_last).slerp(rat, Eigen::Quaterniond(ang_now)));
            // imu.mw = ang_mid.angle() * ang_mid.axis();
            imu.mw = (1 - rat) * imu.mw + rat * imu_now.mw;
            imu.ma = (1 - rat) * imu.ma + rat * imu_now.ma;
          }
        }
      }
#endif

      /*
      const bool test_lower_freq = false;  // true;//
      if (test_lower_freq) {
        if (iterjm1 == iter_start) imu_last = imu;
        mdt_hf += dt;
        update_highfreq(imu.mw - bgi_bar, imu.ma - bai_bar, dt);
        if (mdt_hf >= mdt_hf_ref || iterj == iter_stop) {
          if (iterj == iter_stop) std::cout << "mdt_hf=" << mdt_hf << std::endl;
          IMUDataBase imu_fake;
          double dt_fake = 1. / 200;

          //              int n = int(mdt_hf / dt_fake) + 1;
          //              Eigen::Vector3d alphadt, a0;
          //              double coeff[4];//vij=c0*a0+c1*alpha*dt;pij=a0*c2+alpha*dt*c3;
          //              double D = 0;
          //              if (n > 0) {
          //                  double dtf_fake = mdt_hf - (n-1) * dt_fake;
          //                  double dt2_fake = dt_fake * dt_fake, dtf2_fake = dtf_fake * dtf_fake, dtdtf_fake = dt_fake
      * dtf_fake; double n2 = n * n; coeff[0] = (n - 1) * dt_fake + dtf_fake; coeff[1] = (n2 - 3 * n + 2) * dt_fake / 2
      + (n - 1) * dtf_fake; coeff[2] = (n2 - 2 * n + 1) * dt2_fake / 2 + dtf2_fake / 2 + (n - 1) * dtdtf_fake; coeff[3]
      = ((2 * n2 * n - 9 * n2 + 13 * n - 6) * dt2_fake / 6 + (n - 1) * dtf2_fake +
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
          std::cout << "ma = " << mvij_hf / mdt_hf << " map = " << mpij_hf / (mdt_hf * mdt_hf / 2) << std::endl;
          std::cout << "before i_p_i_j = Rij-1 * j-1(j)_p_j-1_j + p_i_j-1(j) = "
                    << (mRij * mpij_hf + mpij + mvij * mdt_hf).transpose() << std::endl;
          std::cout << "i_v_i_j = " << (mRij * mvij_hf + mvij).transpose() << std::endl;
          std::cout << mRij * mRij_hf << std::endl;

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

          std::cout << "after i_p_i_j = " << mpij.transpose() << std::endl;
          std::cout << "i_v_i_j = " << mvij.transpose() << std::endl;
          std::cout << mRij << std::endl;
          mRij_hf.setIdentity();
          mvij_hf.setZero();
          mpij_hf.setZero();
          mdt_hf = 0;
        }
        continue;
      }*/

#ifndef USE_PREINT_EULA
      // update pre-integrator(interplot)
      if (iterjm1 == iter_start) {  // we could use imu to preintegrate [timeStampi,imu]
        double dt_comple = iterjm1->mtm - timeStampi;
        if (bimu_order_back ? dt_comple < 0 : dt_comple > 0) {
          update(imu.mw - bgi_bar, imu.ma - bai_bar, dt_comple);
          dt -= dt_comple;
          if (!dt) continue;
        }
      }
      Tcalc dt_comple_stop = 0;
      if (iterj == iter_stop) {
        dt_comple_stop = (Tcalc)(timeStampj - imu_now.mtm);
        if (bimu_order_back ? dt_comple_stop < 0 : dt_comple_stop > 0) {
          dt -= dt_comple_stop;
        }
      }
      // end speical constant process
      update((imu_now.mw + imu.mw) / 2 - bgi_bar, (imu_now.ma + imu.ma) / 2 - bai_bar, dt);
      if (bimu_order_back ? dt_comple_stop < 0 : dt_comple_stop > 0) {
        update(imu_now.mw - bgi_bar, imu_now.ma - bai_bar, dt_comple_stop);
      }
#else
      // update pre-integrator
      update(imu.mw - bgi_bar, imu.ma - bai_bar, dt);
#endif
    }
  }
  return 0;
}
template <class IMUDataBase>
void IMUPreIntegratorBase<IMUDataBase>::update(const Vector3d &omega, const Vector3d &acc, const double &dt) {
  using namespace Sophus;
  using namespace Eigen;
  double dt2div2 = dt * dt / 2;
  Matrix3d dR = SO3calc::Exp(omega * dt);       // Exp((w~j-1 - bgi_bar)*dtj-1j)=delta~Rj-1j
  Matrix3d Jr = SO3exd::JacobianR(omega * dt);  // Jrj-1=Jr(dtj-1j*(w~j-1 - bgi_bar))
  Matrix3d skewa = SO3exd::hat(acc);            //(~aj-1 - bai_bar)^

  // see paper On-Manifold Preintegration (63), notice PRV is different from paper RVP, but the BgBa is the same(A
  // change row&col, B just change row)
  //  err_k+1 = A*err_k + Bg*err_gyro + Ba*err_acc; or Bj-1=[Bg Ba],Aj-1=A
  Matrix3d I3x3 = Matrix3d::Identity();
  Matrix<double, 9, 9> A = Matrix9d::Identity();
  A.block<3, 3>(3, 3) = dR.transpose();
  A.block<3, 3>(6, 3) = -mRij * skewa * dt;
  A.block<3, 3>(0, 3) = -mRij * skewa * dt2div2;
  A.block<3, 3>(0, 6) = I3x3 * dt;
  Matrix<double, 9, 3> Bg = Matrix<double, 9, 3>::Zero();
  Bg.block<3, 3>(3, 0) = Jr * dt;
  Matrix<double, 9, 3> Ba = Matrix<double, 9, 3>::Zero();
  Ba.block<3, 3>(6, 0) = mRij * dt;
  Ba.block<3, 3>(0, 0) = mRij * dt2div2;
  if (IMUDataBase::mdt_cov_noise_fixed)
    mSigmaijPRV = A * mSigmaijPRV * A.transpose() + Bg * IMUDataBase::mSigmag * Bg.transpose() +
                  Ba * IMUDataBase::mSigmaa * Ba.transpose();  // notice using Sigma_eta_g/a_d here
  else if (!IMUDataBase::mFreqRef || dt < 1.5 / IMUDataBase::mFreqRef)
    mSigmaijPRV = A * mSigmaijPRV * A.transpose() + Bg * (IMUDataBase::mSigmag / dt) * Bg.transpose() +
                  Ba * (IMUDataBase::mSigmaa / dt) * Ba.transpose();
  else
    mSigmaijPRV = A * mSigmaijPRV * A.transpose() +
                  Bg * (IMUDataBase::mSigmag * IMUDataBase::mFreqRef) * Bg.transpose() +
                  Ba * (IMUDataBase::mSigmaa * IMUDataBase::mFreqRef) * Ba.transpose();
  // the order is opposite(row&col order) for A, opposite row for B
  A = Matrix9d::Identity();
  A.block<3, 3>(6, 6) = dR.transpose();
  A.block<3, 3>(3, 6) = -mRij * skewa * dt;
  A.block<3, 3>(0, 6) = -mRij * skewa * dt2div2;
  A.block<3, 3>(0, 3) = I3x3 * dt;
  Bg = Matrix<double, 9, 3>::Zero();
  Bg.block<3, 3>(6, 0) = Jr * dt;
  Ba = Matrix<double, 9, 3>::Zero();
  Ba.block<3, 3>(3, 0) = mRij * dt;
  Ba.block<3, 3>(0, 0) = mRij * dt2div2;
  if (IMUDataBase::mdt_cov_noise_fixed)
    mSigmaij = A * mSigmaij * A.transpose() + Bg * IMUDataBase::mSigmag * Bg.transpose() +
               Ba * IMUDataBase::mSigmaa * Ba.transpose();
  else if (!IMUDataBase::mFreqRef || dt < 1.5 / IMUDataBase::mFreqRef)
    mSigmaij = A * mSigmaij * A.transpose() + Bg * (IMUDataBase::mSigmag / dt) * Bg.transpose() +
               Ba * (IMUDataBase::mSigmaa / dt) * Ba.transpose();
  else
    mSigmaij = A * mSigmaij * A.transpose() + Bg * (IMUDataBase::mSigmag * IMUDataBase::mFreqRef) * Bg.transpose() +
               Ba * (IMUDataBase::mSigmaa * IMUDataBase::mFreqRef) * Ba.transpose();

  // see the same paper (69) & use similar iterative rearrange method (59)
  //  jacobian of delta measurements w.r.t bias of gyro/acc, for motion_update_with_dbi & residual error &
  //  J_error_dxi,xj calculation update P first, then V, then R for using ij as ij-1 term
  mJapij += mJavij * dt - mRij * dt2div2;  // mRij here is delta~Rij-1 before its updation
  mJgpij += mJgvij * dt - mRij * skewa * mJgRij * dt2div2;
  mJavij += -mRij * dt;
  mJgvij += -mRij * skewa * mJgRij * dt;       // notice except mJgRij use dR, the other Jxxij use mRij!
  mJgRij = dR.transpose() * mJgRij - Jr * dt;  // like (59): JgRij=delta~Rj-1j.t()*JgRij-1 - Jrj-1*dtj-1j, the left
                                               // incremental formula is easy to get for there's no j label

  // see paper On-Manifold Preintegration (35~37)
  mpij += mvij * dt +
          mRij * (acc *
                  dt2div2);   // delta~pij=delta~pij-1 + delta~vij-1*dtj-1j + 1/2*delta~Rij-1*(~aj-1 - bai_bar)*dtj-1j^2
  mvij += mRij * (acc * dt);  // here mRij=mRij-1, delta~vij=delta~vij-1 + delta~Rij-1 * (~aj-1 - bai_bar)*dtj-1j
  // normalize rotation, in case of numerical error accumulation
  mRij = SO3calc::normalizeRotationM(
      mRij *
      dR);  // here omega=(w~k-bgi_bar)(k=j-1), deltaR~ij(bgi_bar)=deltaRij-1(bgi_bar) * Exp((w~j-1 - bgi_bar)*dtj-1j)

  this->mdeltatij += dt;
}

template <class IMUDataBase>
void IMUPreIntegratorBase<IMUDataBase>::update_highfreq(const Vector3d &omega, const Vector3d &acc, const double &dt) {
  using namespace Sophus;
  using namespace Eigen;
  double dt2div2 = dt * dt / 2;
  Matrix3d dR = SO3calc::Exp(omega * dt);  // Exp((w~j-1 - bgi_bar)*dtj-1j)=delta~Rj-1j

  // see paper On-Manifold Preintegration (35~37)
  mpij_hf +=
      mvij_hf * dt +
      mRij_hf *
          (acc * dt2div2);  // delta~pij=delta~pij-1 + delta~vij-1*dtj-1j + 1/2*delta~Rij-1*(~aj-1 - bai_bar)*dtj-1j^2
  mvij_hf += mRij_hf * (acc * dt);  // here mRij=mRij-1, delta~vij=delta~vij-1 + delta~Rij-1 * (~aj-1 - bai_bar)*dtj-1j
  // normalize rotation, in case of numerical error accumulation
  mRij_hf = SO3calc::normalizeRotationM(
      mRij_hf *
      dR);  // here omega=(w~k-bgi_bar)(k=j-1), deltaR~ij(bgi_bar)=deltaRij-1(bgi_bar) * Exp((w~j-1 - bgi_bar)*dtj-1j)
}

class IMUPreIntegratorDerived : public IMUPreIntegratorBase<IMUDataDerived> {
 public:
  Matrix3d mdelxRji;     // delta~Rij.t() from qIMU PreIntegration, 3*3*float
  Matrix3d mSigmaPhiij;  // SigmaPhiij by qIMU, 3*3*float

  IMUPreIntegratorDerived() : mdelxRji(Matrix3d::Identity()), mSigmaPhiij(Matrix3d::Zero()) {}
  void SetPreIntegrationList(
      const listeig(IMUDataDerived)::const_iterator &begin,
      const listeig(IMUDataDerived)::const_iterator &end) {  // rewrite, will override the base class one
    this->mlOdom.clear();
    auto pback = end;
    --pback;
    this->mlOdom.push_front(*begin);
    this->mlOdom.push_back(*pback);
  }
  void PreIntegration(const double &timeStampi, const double &timeStampj);  // rewrite

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#ifndef TRACK_WITH_IMU
typedef IMUPreIntegratorDerived IMUPreintegrator;
#else
typedef IMUPreIntegratorBase<IMUDataBase> IMUPreintegrator;
#endif

}  // namespace VIEO_SLAM

#endif
