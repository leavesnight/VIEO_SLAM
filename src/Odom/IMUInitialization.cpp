#include "IMUInitialization.h"
#include "Optimizer.h"
//#include "sophus/se3.hpp"
#include "eigen_utils.h"

namespace VIEO_SLAM {

using namespace std;
using namespace Eigen;
using Sophus::SO3ex;
using Sophus::SO3exd;
typedef Sophus::SE3<IMUInitialization::Tcalc_sgba> SE3calc;

int8_t kCoeffPriorDefault = 1;  // 0 will make coeff_priora/g:1->5e4 */1e6 *

IMUKeyFrameInit::IMUKeyFrameInit(KeyFrame &kf)
    : timestamp_(kf.timestamp_),
      mTwc(kf.GetPoseInverse()),
      mTcw(kf.GetPose()),
      mpPrevKeyFrame(NULL),                // GetPose() already return .clone()
      mOdomPreIntIMU(kf.GetIMUPreInt()) {  // this func. for IMU Initialization cache of KFs, so need deep copy
  bg_ = ba_ = Vector3d::Zero();            // as stated in IV-A in VIORBSLAM paper
  const listeig(IMUData) limu = kf.GetListIMUData();
  mOdomPreIntIMU.SetPreIntegrationList(limu.begin(), limu.end());
}
IMUKeyFrameInitFix::IMUKeyFrameInitFix(KeyFrame &kf) : IMUKeyFrameInit(kf) {
  NavStated ns = kf.GetNavState();
  dbg_ = ns.mdbg;
  dba_ = ns.mdba;
}

cv::Mat IMUInitialization::GetGravityVec(void) {
  // may need mutex for it 1stly calculated in this or gba thread and then it will be a const!
  unique_lock<mutex> lock(mMutexInitIMU);
  return mGravityVec.clone();  // avoid simultaneous operation
}
void IMUInitialization::SetGravityVec(const cv::Mat &mat) {
  unique_lock<mutex> lock(mMutexInitIMU);
  mGravityVec = mat.clone();  // avoid simultaneous operation
}

void IMUInitialization::Run() {
  unsigned long initedid;
  cout << "start VINSInitThread" << endl;
  mbFinish = false;
  static int n_imu_extra_init;
  while (1) {
    if (GetSensorIMU()) {
      if (!GetVINSInited()) {  // at least 4 consecutive KFs, see IV-B/C VIORBSLAM paper
        if (mdStartTime == -1) {
          initedid = 0;
          mdStartTime = -2;
          n_imu_extra_init = 0;
        }

        KeyFrame *pCurKF = GetCurrentKeyFrame();
        if (pCurKF) {
#define USE_FAST_IMU_INIT
          if (mdStartTime < 0 && !mpMap->mvpKeyFrameOrigins.empty()) {
            mdStartTime = mpMap->mvpKeyFrameOrigins.front()->timestamp_;
          }
          if (mdStartTime >= 0 && pCurKF->timestamp_ - mdStartTime >= mdInitTime)
            if (pCurKF->mnId > initedid) {
              // if succeed in IMU Initialization, this thread will finish, when u want the users' pushing reset button
              // be effective, delete break!
#ifndef USE_FAST_IMU_INIT
              initedid = pCurKF->mnId;
              if (TryInitVIO()) break;
#else
              if (-2 != TryInitVIO_zzh()) {
                initedid = pCurKF->mnId;
              }
#endif
            }
        }
      }
#ifdef USE_FAST_IMU_INIT
      else if (2 > n_imu_extra_init) {
        KeyFrame *pCurKF = GetCurrentKeyFrame();
        if (pCurKF->timestamp_ - mdStartTime > (1 > n_imu_extra_init ? 5 : 15) &&
            (int8_t)(Tracking::OK) == pCurKF->getState()) {
          KeyFrame *pKFlocal = pCurKF;
          auto prefkf = mpMap->mvpKeyFrameOrigins.empty() ? pCurKF : mpMap->mvpKeyFrameOrigins.front();
          int Nlocal = 10;
          do {
            if (pKFlocal->isBad()) break;
            NavStated ns2 = prefkf->GetNavState(), ns1 = pKFlocal->GetNavState();
            Vector3d rb1b2 = (ns1.mRwb.inverse() * ns2.mRwb).log();
            if (rb1b2.norm() > 0.1) {
              SetInitGBAOver(false);
              SetInitGBA(true);
              if (2 == ++n_imu_extra_init) SetInitGBA2(true);
              break;
            }
            pKFlocal = pKFlocal->GetPrevKeyFrame();
          } while (--Nlocal > 0 && pKFlocal);
        }
      }
#endif
    }

    ResetIfRequested();
    if (GetFinishRequest()) break;
    //     usleep(3000);
    usleep(mnSleepTime);  // 3,1,0.5
  }
  SetFinish(true);
  cout << "VINSInitThread is Over." << endl;
}

int IMUInitialization::deleteKFs_ret(vector<vector<IMUKeyFrameInit *> *> &vKFsInit) {
  for (int h = 0; h < vKFsInit.size(); ++h) {
    for (int i = 0; i < vKFsInit[h]->size(); ++i) {
      if ((*vKFsInit[h])[i]) delete (*vKFsInit[h])[i];
    }
    delete vKFsInit[h];
  }
#ifdef DEBUG_SEGMENT_OR_STUCK
  PRINT_DEBUG_INFO("deleteKFs_ret over" << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt")
#endif
  return 0;
}

int IMUInitialization::reduceKFs(const vector<int8_t> &reduced_hids, vector<Vector3d> &bgs_est,
                                 vector<Vector3d> &bas_est, vector<vector<IMUKeyFrameInit *> *> &vKFsInit,
                                 vector<vector<IMUKeyFrameInit *> *> &vKFsInit2, vector<int> &Ns, int &num_handlers,
                                 vector<int8_t> &id_cams, vector<int8_t> *id_cams_ref) {
  num_handlers = reduced_hids.size();
  Ns.clear();
  int8_t ref_pass = 0;
  id_cams.resize(num_handlers);
  if (id_cams_ref && (*id_cams_ref).size() == num_handlers) ref_pass = 1;
  for (int i = 0; i < num_handlers; ++i) {
    vKFsInit2.push_back(vKFsInit[reduced_hids[i]]);
    vKFsInit[reduced_hids[i]] = NULL;
    assert(i <= reduced_hids[i]);
    bgs_est[i] = bgs_est[reduced_hids[i]];
    bas_est[i] = bas_est[reduced_hids[i]];
    if (ref_pass)
      id_cams[i] = (*id_cams_ref)[i];
    else
      id_cams[i] = reduced_hids[i];
    Ns.push_back(vKFsInit2.back()->size());
  }
  bgs_est.resize(num_handlers);
  bas_est.resize(num_handlers);
  // delete redundant KFsInit
  for (int h = 0; h < vKFsInit.size(); ++h) {
    if (NULL != vKFsInit[h]) {
      for (int i = 0; i < vKFsInit[h]->size(); ++i) {
        if ((*vKFsInit[h])[i]) delete (*vKFsInit[h])[i];
      }
      vKFsInit[h] = NULL;
    }
  }
  vKFsInit.clear();

  return 0;
}

void IMUInitialization::ConstructAxeqb(const IMUPreintegrator &imupreint12, const IMUPreintegrator &imupreint23,
                                       const IMUKeyFrameInit *pKF1, const IMUKeyFrameInit *pKF2,
                                       const IMUKeyFrameInit *pKF3, const Matrix3calc &Rcb1, const Matrix3calc &Rcb2,
                                       const Vector3calc &pcb1, const Vector3calc &pcb2, const Vector3calc &pcb3,
                                       Matrix3calc &phi, Vector3calc &psi, Vector3calc &lambda, bool bcalc_cov,
                                       Matrix3calc *pzeta, const SO3calc *pRwI, const Vector3calc *pGI, int fixMode) {
  double dt12 = imupreint12.dt_ij_;
  double dt23 = imupreint23.dt_ij_;
  // Pre-integrated measurements
  Vector3calc dp12 = imupreint12.pij_.cast<Tcalc_sgba>();
  Vector3calc dp23 = imupreint23.pij_.cast<Tcalc_sgba>();
  Vector3calc dv12 = imupreint12.vij_.cast<Tcalc_sgba>();
  // Pose of camera in world frame
  // Twci for pwci&Rwci, not necessary for clone()
  SE3calc Twc1 = Converter::toSE3<Tcalc_sgba>(pKF1->mTwc).cast<Tcalc_sgba>();
  SE3calc Twc2 = Converter::toSE3<Tcalc_sgba>(pKF2->mTwc).cast<Tcalc_sgba>();
  SE3calc Twc3 = Converter::toSE3<Tcalc_sgba>(pKF3->mTwc).cast<Tcalc_sgba>();
  Vector3calc pc1 = Twc1.translation();  // pwci
  Vector3calc pc2 = Twc2.translation();
  Vector3calc pc3 = Twc3.translation();
  Matrix3calc Rc1 = Twc1.so3().matrix();  // Rwci
  Matrix3calc Rc2 = Twc2.so3().matrix();
  Matrix3calc Rc3 = Twc3.so3().matrix();
  //  Eigen::Vector3d angRc1 =Twc1.so3().log().cast<double>(), angRc12 = SO3calc(Rc1).log().cast<double>();

  // pzeta!=nullptr: Stack to C/D matrix; lambda*s + phi(:,0:1)*dthetaxy + zeta*ba = psi, Ci(3*6),Di/psi(3*1)
  // pzeta=nullptr: fill A/B matrix: lambda*s + beta*g = gamma(3*1), Ai(3*4)=[lambda beta], (13) in the paper
  phi = (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 * Matrix3calc::Identity();  // the part of beta
  // note:  - paper & deltatij^2 in paper means dt12^2*dt23+dt23^2*dt12
  // notice here use Rwi*GI instead of gwstar for it's designed for iterative usage
  Matrix3calc Rwb1dt23 = dt23 * Rc1 * Rcb1, Rwb2dt12 = dt12 * Rc2 * Rcb2, Rwb1dt23dt12 = Rwb1dt23 * dt12;
  psi = (Rc1 * pcb1 - Rc2 * pcb2) * dt23 + (Rc3 * pcb3 - Rc2 * pcb2) * dt12 - Rwb2dt12 * dp23 - Rwb1dt23dt12 * dv12 +
        Rwb1dt23 * dp12;  // the part of gamma
  if (pzeta) {
    assert(pRwI && pGI);
    Matrix3calc Jav12 = imupreint12.Javij_.cast<Tcalc_sgba>();
    Matrix3calc Jap12 = imupreint12.Japij_.cast<Tcalc_sgba>();
    Matrix3calc Jap23 = imupreint23.Japij_.cast<Tcalc_sgba>();
    phi *= -pRwI->matrix() * SO3ex<Tcalc_sgba>::hat(*pGI);  // 3*3 note: this has a '-', different to paper
    psi -= (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 * ((*pRwI) * (*pGI));

    if (!(kFixBgBa & fixMode)) {
      *pzeta = Rc2 * Rcb2 * Jap23 * dt12 + Rc1 * Rcb1 * Jav12 * (dt12 * dt23) -
               Rc1 * Rcb1 * Jap12 * dt23;  // 3*3 notice here is Jav12, paper writes a wrong Jav23
    } else {
      Vector3calc dba2 = dynamic_cast<const IMUKeyFrameInitFix *>(pKF2)->dba_.cast<Tcalc_sgba>();
      Vector3calc dba1 = dynamic_cast<const IMUKeyFrameInitFix *>(pKF1)->dba_.cast<Tcalc_sgba>();
      psi -= Rc2 * (Rcb2 * (Jap23 * (dt12 * dba2))) + Rc1 * (Rcb1 * (Jav12 * ((dt12 * dt23) * dba1))) -
             Rc1 * (Rcb1 * (Jap12 * (dt23 * dba1)));  // 3*3 notice here is Jav12, paper writes a wrong Jav23
    }
  }

  lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;  // 3*1

  if (bcalc_cov) {
    // D(x1+x2)=D(x1)+D(x2) if x1 dep. from x2
    //    Matrix3d Cov_dp_tmp = imupreint12.SigmaijPRV_.block<3, 3>(0, 0) * dt23 * dt23 +
    //                          imupreint23.SigmaijPRV_.block<3, 3>(0, 0) * dt12 * dt12;
    Matrix3d Rwb1dt23d = Rwb1dt23.cast<double>(), Rwb2dt12d = Rwb2dt12.cast<double>(),
             Rwb1dt23dt12d = Rwb1dt23dt12.cast<double>();
    Matrix3d Cov_dp_tmp = Rwb1dt23d * imupreint12.SigmaijPRV_.block<3, 3>(0, 0) * Rwb1dt23d.transpose() +
                          Rwb2dt12d * imupreint23.SigmaijPRV_.block<3, 3>(0, 0) * Rwb2dt12d.transpose() +
                          Rwb1dt23dt12d * imupreint12.SigmaijPRV_.block<3, 3>(6, 6) * Rwb1dt23dt12d.transpose() -
                          Rwb1dt23d * imupreint12.SigmaijPRV_.block<3, 3>(0, 6) * Rwb1dt23dt12d.transpose() -
                          Rwb1dt23dt12d * imupreint12.SigmaijPRV_.block<3, 3>(6, 0) * Rwb1dt23d.transpose();
    if (isnan(imupreint12.SigmaijPRV_(0, 0)))
      PRINT_DEBUG_INFO(redSTR << "SigmaPRV12=" << imupreint12.SigmaijPRV_ << ",dt=" << imupreint12.dt_ij_ << endl,
                       imu_tightly_debug_path, "imu_init_thread_debug.txt");
    // U is LT
    Matrix3calc sqrt_Info_dp = Matrix3d(Eigen::LLT<Matrix3d>(Cov_dp_tmp.inverse()).matrixU()).cast<Tcalc_sgba>();
    //                    sqrt_Info_dp = Matrix3calc::Identity();

    lambda = sqrt_Info_dp * lambda;
    phi = sqrt_Info_dp * phi;
    psi = sqrt_Info_dp * psi;
    if (pzeta) *pzeta = sqrt_Info_dp * (*pzeta);
  }
}

template <class _FrameBase>
void IMUInitialization::FillAxeqb(const vector<_FrameBase *> &fbs_init, int num_var_opt2, int h, MatrixXXcalc *pC,
                                  VectorXcalc *pD, int &num_eq2, int fixMode, const double *pscale, bool bcalc_ba,
                                  bool bcalc_gdir, int *pnum_eq_h, vector<int8_t> *preduced_hids,
                                  vector<vector<size_t>> *pid_Tcbs, aligned_vector<aligned_vector<SO3calc>> *pRcbs,
                                  aligned_vector<aligned_vector<Vector3calc>> *ppcbs, int *plast_h, const SO3calc *pRwI,
                                  const Vector3calc *pGI, double *psum_dt, bool bcalc_cov, bool verbose) {
  size_t reduced_h;
  aligned_vector<SO3calc> *pRcbs_h;
  aligned_vector<Vector3calc> *ppcbs_h;
  if (preduced_hids) {
    reduced_h = (*preduced_hids)[h];
    pRcbs_h = &((*pRcbs)[reduced_h]);
    ppcbs_h = &((*ppcbs)[reduced_h]);
  }
  Matrix3calc *pzeta = nullptr;
  if (bcalc_gdir) pzeta = new Matrix3calc();
  SO3calc Rcb[3];
  Vector3calc pcb[3];

  size_t num_framebases = fbs_init.size();
  for (size_t i = 0, last_i = i; i < num_framebases - 2; ++i) {
    _FrameBase *pKF2 = fbs_init[i + 1], *pKF3 = fbs_init[i + 2];
    const IMUPreintegrator &imupreint12 = pKF2->mOdomPreIntIMU, &imupreint23 = pKF3->mOdomPreIntIMU;
    // d means delta
    double dt12 = imupreint12.dt_ij_;  // deltat12
    double dt23 = imupreint23.dt_ij_;

    if (psum_dt) {
      if (!i) {
        *psum_dt += dt12 + dt23;
      } else {
        *psum_dt += dt23;
      }
    }

    if (dt12 == 0 || dt23 == 0) {
      if (verbose) {
        PRINT_DEBUG_INFO(redSTR << "Tm=" << fixed << setprecision(9) << pKF2->timestamp_ << " lack IMU data!" << dt12
                                << "," << dt23 << whiteSTR << defaultfloat << endl,
                         imu_tightly_debug_path, "imu_init_thread_debug.txt");
      }
      continue;
    }

    // Extrinsics
    if (!preduced_hids) {
      if (plast_h && h > *plast_h) {
        assert(pRcbs && ppcbs && pid_Tcbs);
        pRcbs->resize(pRcbs->size() + 1);
        ppcbs->resize(ppcbs->size() + 1);
        pid_Tcbs->resize(pid_Tcbs->size() + 1);
        *plast_h = h;
      }

      for (int k = 0; k < 3; ++k) {
        SE3calc Tbc = Converter::toSE3<Tcalc_sgba>(Frame::mTbc);  // fbs_init[i + k]->mTbc.template cast<Tcalc_sgba>();
        SO3calc Rbc = Tbc.so3();
        Vector3calc pbc = Tbc.translation();
        Rcb[k] = Rbc.inverse();
        pcb[k] = -(Rcb[k] * pbc);

        if (plast_h && i + k + 1 > last_i) {
          assert(pRcbs && ppcbs && pid_Tcbs);
          pRcbs->back().push_back(Rcb[k]);
          ppcbs->back().push_back(pcb[k]);
          last_i = i + k + 1;
        }
      }
      if (plast_h && pid_Tcbs) {
        assert(pRcbs && ppcbs);
        pid_Tcbs->back().push_back(pRcbs->back().size() - 3);
      }
    }

    if (pC && pD) {
      Matrix3calc phi;
      Vector3calc psi, lambda;
      if (preduced_hids) {
        assert(pnum_eq_h);
        size_t reduced_i = (*pid_Tcbs)[reduced_h][*pnum_eq_h];

        const Matrix3calc &Rcb1 = (*pRcbs_h)[reduced_i].matrix(), &Rcb2 = (*pRcbs_h)[reduced_i + 1].matrix();
        const Vector3calc &pcb1 = (*ppcbs_h)[reduced_i], &pcb2 = (*ppcbs_h)[reduced_i + 1],
                          &pcb3 = (*ppcbs_h)[reduced_i + 2];
        ConstructAxeqb(imupreint12, imupreint23, fbs_init[i], pKF2, pKF3, Rcb1, Rcb2, pcb1, pcb2, pcb3, phi, psi,
                       lambda, bcalc_cov, pzeta, pRwI, pGI, fixMode);
      } else {
        ConstructAxeqb(imupreint12, imupreint23, fbs_init[i], pKF2, pKF3, Rcb[0].matrix(), Rcb[1].matrix(), pcb[0],
                       pcb[1], pcb[2], phi, psi, lambda, bcalc_cov, pzeta, pRwI, pGI, fixMode);
      }
      if (!(kFixS & fixMode)) {
        assert(!(kIMUInitFixScale & mode_init_));
        pC->block<3, 1>(3 * num_eq2, 0) = lambda;
      } else {
        assert(kIMUInitFixScale & mode_init_);
        assert(pscale);
        psi -= lambda * (*pscale);
      }

      if (!(kFixG & fixMode)) {
        if (bcalc_gdir) {
          // phi(:,0:1)(3*2) / only first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
          pC->block<3, 2>(3 * num_eq2, num_var_opt2 - 5) = phi.block<3, 2>(0, 0);
        } else {
          pC->block<3, 3>(3 * num_eq2, num_var_opt2 - 3) = phi;  // Ai
        }
      } else {
        if (!bcalc_gdir)  // for delta_theta_xy=0, no bcalc_gdir=true needs to be processed
          psi -= phi * Converter::toVector3d(GetGravityVec()).cast<Tcalc_sgba>();
      }

      if (bcalc_ba && pzeta) {
        assert(!(kFixBgBa & fixMode));
        pC->block<3, 3>(3 * num_eq2, num_var_opt2 - 3 + h * 3) = *pzeta;
      }
      // if bcalc_gdir=false: step 2, the paper missed a minus before γ(i)/B(i)
      pD->segment<3>(3 * num_eq2) = psi;
    }

    ++num_eq2;
    if (pnum_eq_h) ++*pnum_eq_h;
  }

  if (pzeta) delete pzeta;
}

void IMUInitialization::SetIMUInited(vector<bool> &initedIMUs, const vector<int8_t> &id_cams2) {
  SetInitedIMUs(initedIMUs);
  SetHasInitedIMU(true);
  bool all_inited = true;
  for (auto iter = initedIMUs.begin(); initedIMUs.end() != iter; ++iter)
    if (!(*iter)) {
      all_inited = false;
      break;
    }
  if (all_inited) SetVINSInited(true);
}

template <class OdomData>
int IMUInitialization::InitIMUv_lbags(const vector<FrameBase *> &vfb_scale_v_b, vector<FrameBase *> &pnewestkfs,
                                      const vector<unsigned long> &fixed_ref_ids, const vector<int8_t> &id_cams2,
                                      vector<bool> &last_imu_inited, const int fixMode, const vector<Vector3d> &bgs_est,
                                      const vector<Vector3d> &bas_star, const Vector3f &gw, double scale, Map *pMap,
                                      unique_lock<mutex> &lock, int8_t mode_lba, bool bfast_init,
                                      const vector<typename aligned_list<OdomData>::const_iterator> *pviterbeg) {
  // recover right scaled Twc&NavState from old unscaled Twc with scale
  if (verbose) {
    PRINT_DEBUG_INFO("Step4: calculate vwbi and scale recover...", imu_tightly_debug_path, "imu_init_thread_debug.txt");
  }
  vector<int8_t> id_cam_to_i(num_imus_, -1);
  for (int h = 0; h < id_cams2.size(); ++h) id_cam_to_i[id_cams2[h]] = h;
  Vector3d gwd = gw.cast<double>();
  size_t ibeg = 0;
  for (vector<FrameBase *>::const_iterator vit = vfb_scale_v_b.begin(), vend = vfb_scale_v_b.end(); vit != vend;
       ++vit, ++ibeg) {
    FrameBase *pfb = *vit;
    if ((last_imu_inited[pfb->id_cam_] && (kFixSVBgBa == (kFixSVBgBa & fixMode))) || pfb->isBad()) continue;

    // we can SetPose() first even no IMU data
    SE3calc Twc = pfb->GetTwc().template cast<Tcalc_sgba>();
    // Position and rotation of visual SLAM
    Vector3calc wPc = Twc.translation();  // wPc/twc
    SO3calc Rwc = Twc.so3();
    // Set position and rotation of navstate
    SE3calc Tbc = Converter::toSE3<Tcalc_sgba>(pfb->mTbc);  // GetTbc().template cast<Tcalc_sgba>();
    SO3calc Rcb = Tbc.so3().inverse();
    Vector3calc pcb = -(Rcb * Tbc.translation());
    NavStated ns;
    if (!last_imu_inited[pfb->id_cam_] || !(kFixS & fixMode)) {
      Vector3calc wPb = scale * wPc + Rwc * pcb;  // right scaled pwb from right scaled pwc
      ns.mpwb = wPb.cast<double>();
      ns.mRwb = (Rwc * Rcb).cast<double>();
    } else {
      ns = pfb->GetNavState();
    }
    int8_t id_cam2i = id_cam_to_i[pfb->id_cam_];
    if (0 <= id_cam2i) {
      assert(!last_imu_inited[pfb->id_cam_]);  // only not fixed one could enter here
      ns.mbg = bgs_est[id_cam2i];
      ns.mba = bas_star[id_cam2i];  // bg* ba*
      // Set delta_bias to zero. (only updated during optimization)
      ns.mdbg = ns.mdba = Vector3d::Zero();
    } else {
      // inited bgba_fixed one, or just inherit the ns;
      assert(last_imu_inited[pfb->id_cam_] && (kFixBgBa & fixMode));
    }
    //  or uninited one then keep the original ns as well
    if (!last_imu_inited[pfb->id_cam_] || !(kFixV & fixMode)) {
      // Step 4. / See IV-D/(18)/(3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
      // compute velocity
      FrameBase *pfbnext = ((KeyFrame *)pfb)->GetNextKeyFrame();  // GetNextFrameBase();
      if (NULL != pfbnext) {
        if (pfbnext->GetIMUPreInt().dt_ij_ == 0) {
          PRINT_DEBUG_INFO("time 0" << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
          pfb->SetNavState(ns);  // no right vwb is ok
          continue;
        }
        pfb->SetNavState(ns);  // we must update the pfb->bg_&ba_ before pfbnext->PreIntegration()
        // it's originally based on bi_bar=0, but now it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
        if (!pviterbeg) {
          dynamic_cast<KeyFrame *>(pfbnext)->PreIntegration<OdomData>(dynamic_cast<KeyFrame *>(pfb));
        } else {
          assert(pviterbeg->size() == vfb_scale_v_b.size() + 1);
          assert(fabs((pviterbeg[0][ibeg])->mtm - pfb->timestamp_) < pfbnext->timestamp_ - pfb->timestamp_);
          dynamic_cast<FrameBase *>(pfbnext)->PreIntegration<OdomData>(pfb, pviterbeg[0][ibeg], pviterbeg[0][ibeg + 1]);
        }
        // IMU pre-int between pfb ~ pfbnext, though the paper seems to use the
        // vKFInit[k].preint_imu_
        // so its dbgi=0 but its dbai=bai, we use more precise bi_bar here
        const IMUPreintegrator imupreint = pfbnext->GetIMUPreInt();
        double dt = imupreint.dt_ij_;                        // deltati_i+1
        Vector3calc dp = imupreint.pij_.cast<Tcalc_sgba>();  // deltapi_i+1
        // cv::Mat Japij=Converter::toCvMat(imupreint.Japij_);//Ja_deltap
        SE3calc Twcnext = pfbnext->GetTwc().template cast<Tcalc_sgba>();
        Vector3calc wPcnext = Twcnext.translation().cast<Tcalc_sgba>();  // wPci+1
        SO3calc Rwcnext = Twcnext.so3().cast<Tcalc_sgba>();              // Rwci+1
        SE3calc Tbcnext = Converter::toSE3<Tcalc_sgba>(pfbnext->mTbc);   // GetTbc().template cast<Tcalc_sgba>();
        SO3calc Rcbnext = Tbcnext.so3().inverse();
        Vector3calc pcbnext = -(Rcbnext * Tbcnext.translation());
        //-1/dt*(pwbi-pwbj+1/2*gw*dt^2+Rwbi*(dp+Japij*dbai)), pwbi=s*pwc+Rwc*pcb,
        // s=sw=swRightScaled_wNow
        Vector3calc vwbi =
            -1. / dt *
            (scale * (wPc - wPcnext) + (Rwc * pcb - Rwcnext * pcbnext) + dt * dt / 2 * gw + Rwc * Rcb * (dp));
        ns.mvwb = vwbi.cast<double>();
      } else {
        // If this is the last KeyFrame, no 'next' KeyFrame exists, use (3) in VOIRBSLAM paper with
        // ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
        if (pfb->GetIMUPreInt().dt_ij_ == 0) {
          PRINT_DEBUG_INFO("time 0" << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
          pfb->SetNavState(ns);  // no right vwb is ok
          continue;
        }
        FrameBase *pfbprev = ((KeyFrame *)pfb)->GetPrevKeyFrame();  // GetPrevFrameBase();
        assert(pfbprev && "pfbnext is NULL");
        // notice it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
        const IMUPreintegrator imupreint = pfb->GetIMUPreInt();
        double dt = imupreint.dt_ij_;
        NavStated nsprev = pfbprev->GetNavState();
        // vwbj=vwbi+gw*dt+Rwbi*(dvij+Javij*dbai)
        ns.mvwb = nsprev.mvwb + gwd * dt + nsprev.mRwb * (imupreint.vij_);
      }
    }
    if (verbose) {
      PRINT_DEBUG_INFO("id" << (int)pfb->id_cam_ << ",tm" << pfb->timestamp_ << "," << ns.mvwb.transpose() << ";",
                       imu_tightly_debug_path, "imu_init_thread_debug.txt");
    }
    pfb->SetNavState(ns);  // now ns also has the right vwb_
  }
  if (verbose) PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");

  // Update MPs' Position
  // we don't change the vpMPs[i] but change the *vpMPs[i]
  vector<MapPoint *> vpMPs = mpMap->GetAllMapPoints();
  for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; ++vit)
    (*vit)->UpdateScale(scale);

  // Now every thing in Map is right scaled & mGravityVec is got
  for (int h = 0; h < id_cams2.size(); ++h) {
    last_imu_inited[id_cams2[h]] = true;
  }

  if (bfast_init) {
    SetIMUInited(last_imu_inited, id_cams2);
    mpMap->InformNewChange();
  }
  lock.unlock();
  if (bfast_init) {
    mpLocalMapper->Release();  // recover LocalMapping thread, same as CorrectLoop()
  }

  if (kLBANone > mode_lba) {
    if (GetFinishRequest()) {
      return -2;
    }

    // initial Inertial-Only BA here!
    int niters = 5;  // 200
    bool *pabort_BA = &abortBA_;
    bool first_time = false;
    vector<bool> binited_imus = last_imu_inited, binit_imus = binited_imus;
    if (!tot_imu_inited_) {
      for (int h = 0; h < num_imus_; ++h)
        if (binit_imus[h]) {
          PRINT_DEBUG_INFO("first_time,h=" << (int)vfirst_time_[h] << h << endl, imu_tightly_debug_path,
                           "imu_init_thread_debug.txt");
          if (vfirst_time_[h]) {
            first_time = true;
          } else
            binit_imus[h] = false;
        }
      if (first_time) {
        pabort_BA = nullptr;
        //                            niters *= 2;
        //                            local_window_size *= 2;
      }
    }

    chrono::steady_clock::time_point t1;
    if (verbose) {
      t1 = chrono::steady_clock::now();
    }

    PRINT_DEBUG_INFO("IMU_init_lba" << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
    int ret_val = Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap, GetGravityVec(), niters + 10, pabort_BA, 0, false,
                                                               false, this);
    //        mbFixScale=true;//not good for V203 when used
    //    int ret_val = Optimizer::SlidingWindowBA(this, binited_imus, pabort_BA, pnewestkfs, fixed_ref_ids, pMap,
    //                                             local_window_size_, niters, 10, 1, Optimizer::kRobustSmart,
    //                                             first_time ? &binit_imus : NULL, 2, kVerbRel, mode_lba);

    if (verbose) {
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
      double duration = (double)std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      PRINT_DEBUG_INFO(yellowSTR "LBA cost " << duration << "s" << whiteSTR << endl, imu_tightly_debug_path,
                       "imu_init_thread_debug.txt");
    }

    if (ret_val <= 0) {
      PRINT_DEBUG_INFO("Warning: ret_val=" << ret_val << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
      // don't change vfirst_time_ and tot_imu_inited and SetIMUInited() when !bfast_init if opt failed!
      return ret_val;
    }

    if (abortBA_)
      PRINT_DEBUG_INFO(redSTR "init too slow" << whiteSTR << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
    if (first_time && (!pabort_BA || !*pabort_BA)) {
      for (int h = 0; h < num_imus_; ++h)
        if (binit_imus[h]) vfirst_time_[h] = false;
      tot_imu_inited_ = true;
      for (int h = 0; h < num_imus_; ++h) {
        if (vfirst_time_[h]) {
          tot_imu_inited_ = false;
          break;
        }
      }
      //                    SetTryInitBA(false);
    }
    if (num_imus_ > 1)
      PRINT_DEBUG_INFO("check vfirst_time[0]=" << (int)vfirst_time_[0] << ", vfirst1=" << (int)vfirst_time_[1],
                       imu_tightly_debug_path, "imu_init_thread_debug.txt");
  }

  return 1;
}

int8_t IMUInitialization::fopened = 0;
// designed both for frame && keyframe; auto relase vKFsInit; bgba_init size=vKFsInit.size(), 3d bg then 3d ba;
// we will auto adjust the following inpu for efficiency: id_cams2[h] means vKFsInit[h]'s id_cam_;
// last_imu_inited[h]=true consists of vKFsFixed
int IMUInitialization::InitIMU(vector<vector<IMUKeyFrameInit *> *> &vKFsInit,
                               aligned_vector<aligned_vector<IMUKeyFrameInitFix>> &vKFsFixed, vector<int8_t> &id_cams2,
                               vector<bool> &last_imu_inited, const int fixMode, int8_t mode_lba,
                               const vector<Vector6d> &vbgba_init, const vector<bool> *pdebug_benough_id_cam) {
  int num_var_opt = !(kFixS & fixMode) ? 4 : 3;
  int num_var_opt2 = num_var_opt + 3 - 1;
  if (kFixG & fixMode) {
    num_var_opt -= 3;
    num_var_opt2 -= 2;
  }
  int fixmode_init = (kFixS | kFixG) & fixMode;
  vector<vector<IMUKeyFrameInit *> *> vKFsInit2;

  // Recording data in txt files for further analysis
  static ofstream fgw, fscale, fbiasa, fcondnum, fbiasg;
  std::ios_base::openmode openmode = ios::app;
  if (mTmpfilepath.length() > 0 && 1 >= fopened) {
    if (!fopened) {
      if (kVerbDeb < verbose)
        PRINT_DEBUG_INFO("open " << mTmpfilepath << "...", imu_tightly_debug_path, "imu_init_thread_debug.txt");
      // Need to modify this to correct path
      fbiasg.open(mTmpfilepath + "biasg.txt", openmode);  // optimized initial bg for these N KFs,3*1
      fgw.open(mTmpfilepath + "gw.txt", openmode);        // gwafter then gw before,6*1
      fscale.open(mTmpfilepath + "scale.txt", openmode);  // scale_fine then scale_rough
      fbiasa.open(mTmpfilepath + "biasa.txt", openmode);  // optimized initial ba for these N KFs,3*1
      // for optimized x is 6*1 vector, see (19) in VOIRBSLAM paper, here just show these 6*1 raw data
      fcondnum.open(mTmpfilepath + "condnum.txt", openmode);
      if (fbiasg.is_open() && fgw.is_open() && (fscale.is_open()) && fbiasa.is_open() && fcondnum.is_open())
        fopened = 2;
      else {
        cerr << "file open error in TryInitVIO" << endl;
        fopened = 0;
      }
    }
    static unsigned long time_imu_init = 0;
    fbiasg << std::fixed << std::setprecision(9) << time_imu_init << endl;
    fgw << std::fixed << std::setprecision(9) << time_imu_init << endl;
    fscale << std::fixed << std::setprecision(9) << time_imu_init << endl;
    fbiasa << std::fixed << std::setprecision(9) << time_imu_init << endl;
    fcondnum << std::fixed << std::setprecision(9) << time_imu_init << endl;
    ++time_imu_init;
    if (kVerbDeb < verbose) PRINT_DEBUG_INFO("...ok..." << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
  }

  // Step 1. / see VIORBSLAM paper IV-A
  int num_handlers = vKFsInit.size();
  size_t num_handlers_fixed = vKFsFixed.size();
  vector<int> Ns(num_handlers), NsFixed(num_handlers_fixed);
  if (verbose)
    PRINT_DEBUG_INFO("Step1: left num_handlers=" << num_handlers << endl, imu_tightly_debug_path,
                     "imu_init_thread_debug.txt");
  vector<Vector3d> bgs_est(num_handlers, Vector3d::Zero());
  vector<Vector3d> bas_star(num_handlers, Vector3d::Zero());
  // init bgbas to init prior ones
  if (vbgba_init.size()) {
    assert(vbgba_init.size() == (size_t)num_handlers);
    for (size_t h = 0; h < (size_t)num_handlers; ++h) {
      bgs_est[h] = vbgba_init[h].segment<3>(0);
      bas_star[h] = vbgba_init[h].segment<3>(3);
    }
  }
  vector<int8_t> reduced_hids, flag_remain(num_handlers);
  // fast return judging
  int8_t tot_imu_init_enable = 0;
  for (int i = 0; i < num_handlers; ++i) {
    Ns[i] = vKFsInit[i]->size();
    if (verbose) {
      PRINT_DEBUG_INFO((int)id_cams2[i] << ":" << Ns[i] << " keyframes" << endl, imu_tightly_debug_path,
                       "imu_init_thread_debug.txt");
    }
    if (4 <= Ns[i]) {
      ++tot_imu_init_enable;
      break;
    }
  }
  for (size_t i = 0; i < num_handlers_fixed; ++i) {
    NsFixed[i] = vKFsFixed[i].size();
  }
  if (!tot_imu_init_enable) {  // at least 1 imu may be initialized
    deleteKFs_ret(vKFsInit);
    return 0;
  }
  std::chrono::steady_clock::time_point tm_start, tm_end;
  // Try to compute initial gyro bias, using optimization with Gauss-Newton
  // nothing changed, just return the optimized result bg*
  for (int i = 0; i < num_handlers; ++i) {
    if (verbose) {
      tm_start = std::chrono::steady_clock::now();
    }
    // TODO(zzh): ceres
    if (0 < Optimizer::OptimizeInitialGyroBias<IMUKeyFrameInit>(*vKFsInit[i], bgs_est[i])) reduced_hids.push_back(i);
    if (verbose) {
      static double max_duration = 0, tot_duration = 0;
      static size_t mean_times = 0;
      tm_end = std::chrono::steady_clock::now();
      double duration = (double)std::chrono::duration_cast<std::chrono::duration<double>>(tm_end - tm_start).count();
      if (max_duration < duration) max_duration = duration;
      tot_duration += duration;
      ++mean_times;
      PRINT_DEBUG_INFO("OptimizeInitialGyroBias cost : " << duration << "s; max = " << max_duration
                                                         << "; mean = " << tot_duration / mean_times << endl,
                       imu_tightly_debug_path, "imu_init_thread_debug.txt");
    }
  }
  // reduce vKFsInit to vKFsInit2
  vector<int8_t> id_cams;
  reduceKFs(reduced_hids, bgs_est, bas_star, vKFsInit, vKFsInit2, Ns, num_handlers, id_cams, &id_cams2);
  if (verbose) {
    for (int i = 0; i < num_handlers; ++i)
      PRINT_DEBUG_INFO("bgs_est[" << (int)id_cams[i] << "]: " << bgs_est[i].transpose() << endl, imu_tightly_debug_path,
                       "imu_init_thread_debug.txt");
    PRINT_DEBUG_INFO("Step2: left num_handlers=" << num_handlers << endl, imu_tightly_debug_path,
                     "imu_init_thread_debug.txt");
  }

  // Update biasg and pre-integration in LocalWindow(here all KFs).
  int num_eq = 0, num_eq2_ref = 0;
  for (int h = 0; h < num_handlers; ++h) {
    for (int i = 0; i < Ns[h]; ++i) (*vKFsInit2[h])[i]->bg_ = bgs_est[h];
    if (2 < Ns[h]) num_eq += Ns[h] - 2;
  }
  for (int h = 0; h < num_handlers; ++h)
    for (int i = 1; i < Ns[h]; ++i)
      // so vKFInit[i].preint_imu_ is based on bg_bar=bgest,ba_bar=0; dbg=0 but dba/ba waits to be optimized
      (*vKFsInit2[h])[i]->ComputePreInt();

  // Step 2. / See VIORBSLAM paper IV-B
  // Approx Scale and Gravity vector in 'world' frame (first/0th KF's camera frame)
  // Solve A*x=B for x=[s,gw] 4x1 vector, using SVD method
  bool skip_step2 = GetHasInitedIMU() || !num_var_opt;
  bool skip_step3 = false;  // true;

  double s_star = 1;
  Vector3calc gw_star = Vector3calc::Zero();
  aligned_vector<aligned_vector<SO3calc>> Rcbs(1);
  aligned_vector<aligned_vector<Vector3calc>> pcbs(1);
  vector<vector<size_t>> id_Tcbs(1);
  tot_imu_init_enable = 0;
  reduced_hids.clear();
  id_cams2.clear();
  const int thresh_cond = 1e6;
  VectorXcalc w = VectorXcalc::Identity(1, 1);
  double cond_num = w[0] / w[w.size() - 1];

  if (!skip_step2) {
    MatrixXXcalc A(3 * num_eq, num_var_opt);  // if num_var_opt=4 unknowns then N must >=4 else >=3
    VectorXcalc B(3 * num_eq);

    A.setZero();
    B.setZero();
    num_eq = 0;
    for (int h = 0, last_h = h; h < num_handlers; ++h) {
      int numEquations = 0;

      FillAxeqb(*vKFsInit2[h], num_var_opt, h, &A, &B, num_eq, fixmode_init, &s_star, false, false, &numEquations,
                nullptr, &id_Tcbs, &Rcbs, &pcbs, &last_h, nullptr, nullptr, nullptr, true, true);

      if (skip_step3 || 3 * numEquations >= num_var_opt2) {
        ++tot_imu_init_enable;
        reduced_hids.push_back(h);
        id_cams2.push_back(id_cams[h]);
        num_eq2_ref += numEquations;
      }
    }
    // for more robust judgement instead of judging KeyFramesInMap()
    if (!tot_imu_init_enable) {  //|| 3 * num_eq < num_var_opt) {
      deleteKFs_ret(vKFsInit2);
      if (verbose) {
        PRINT_DEBUG_INFO("tot_imu_init_enable failed" << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
      }
      return 0;
    }
    // reduce A,B rows to speed up
    A.resize(3 * num_eq, num_var_opt);
    B.resize(3 * num_eq);
    // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
    // A=u*S*vt=u*w*vt, u*w*vt*x=B => x=vt'*winv*u'*B, or we call the pseudo inverse of A/A.inv()=(A.t()*A).inv()*
    // A.t(), in SVD we have A.inv()=v*winv*u.t() where winv is the w with all nonzero term is the reciprocal of the
    // corresponding singular value
    //        A.ldlt().solve(B);
    // Note w is 4x1 vector by SVDecomp()/SVD::compute() not the 4*4(not FULL_UV)/m*n(FULL_UV) singular matrix we
    //  stated last line
    Eigen::JacobiSVD<MatrixXXcalc> svdA(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    w = svdA.singularValues();
    cond_num = w[0] / w[w.size() - 1];
    if (isnan(cond_num) || cond_num > thresh_cond) {
      deleteKFs_ret(vKFsInit2);
      if (verbose) {
        PRINT_DEBUG_INFO("cond_num=" << cond_num << " > thresh_cond(" << thresh_cond << ")" << endl,
                         imu_tightly_debug_path, "imu_init_thread_debug.txt");
      }
      return 0;
    }
    MatrixXXcalc u = svdA.matrixU();
    MatrixXXcalc v = svdA.matrixV();
    MatrixXXcalc winv = MatrixXXcalc::Identity(w.size(), w.size());
    for (int i = 0; i < num_var_opt; ++i) {
      // too small in sufficient w meaning the linear dependent equations causing the solution is not unique(or
      // A.inv() not exist)
      //            if (fabs(w(i)) < 1e-10) {
      //                w(i) += 1e-10;//or [] here for vector only
      //                cerr << "w(i) < 1e-10, w=" << endl << w << endl;
      //            }
      winv(i, i) = 1. / w(i);
    }
    VectorXcalc x = v * (winv * (u.transpose() * B));
    if (!(kFixS & fixMode)) s_star = x(0);                            // scale should be positive
    if (!(kFixG & fixMode)) gw_star = x.segment(num_var_opt - 3, 3);  // gravity should be about ~9.8
  } else {
    gw_star = Converter::toVector3d(GetGravityVec()).cast<Tcalc_sgba>();

    num_eq = 0;
    for (int h = 0, last_h = h; h < num_handlers; ++h) {
      int numEquations = 0;

      FillAxeqb(*vKFsInit2[h], num_var_opt, h, nullptr, nullptr, num_eq, fixmode_init, &s_star, false, false,
                &numEquations, nullptr, &id_Tcbs, &Rcbs, &pcbs, &last_h, nullptr, nullptr, nullptr, true, true);

      if (skip_step3 || 3 * numEquations >= num_var_opt2) {
        ++tot_imu_init_enable;
        reduced_hids.push_back(h);
        id_cams2.push_back(id_cams[h]);
        num_eq2_ref += numEquations;
      }
    }
    if (!tot_imu_init_enable) {  // even in fixMode, we still hope cur init imu could give the result with itself
      deleteKFs_ret(vKFsInit2);
      if (verbose) {
        PRINT_DEBUG_INFO("tot_imu_init_enable failed" << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
      }
      return 0;
    }
  }
  PRINT_DEBUG_INFO("gw_star: " << gw_star.transpose() << ", |gw_star|=" << gw_star.norm() << endl,
                   imu_tightly_debug_path, "imu_init_thread_debug.txt");
  // reduce vKFsInit2 to vKFsInit
  assert(Rcbs.size() <= num_handlers);
  reduceKFs(reduced_hids, bgs_est, bas_star, vKFsInit2, vKFsInit, Ns, num_handlers, id_cams2, &id_cams);
  // Step 3. / See VIORBSLAM paper IV-C
  // Use gravity magnitude 9.810 as constraint; gIn/^gI=[0;0;1], the normalized gravity vector in an inertial
  // frame, we can also choose gIn=[0;0;-1] as the VIORBSLAM paper
  Vector3calc gIn = Vector3calc::Zero();
  gIn(2) = 1;
  Vector3calc GI = gIn * IMUData::mdRefG;  // gI or GI=^gI*G
  double s_ = s_star;
  SO3calc RwI_, RwI;  // RwI means step2's g dir

  Vector3calc gwn = gw_star / gw_star.norm();  //^gw=gw*/||gw*|| / Normalized approx. gravity vecotr in world frame
  Vector3calc gInxgwn = gIn.cross(gwn);
  double normgInxgwn = gInxgwn.norm();
  if (!normgInxgwn || !gw_star.norm()) {
    deleteKFs_ret(vKFsInit);
    if (verbose) {
      PRINT_DEBUG_INFO(redSTR << "gw_star too large!" << whiteSTR << endl, imu_tightly_debug_path,
                       "imu_init_thread_debug.txt");
    }
    return 0;
  }
  Vector3calc vhat = gInxgwn / normgInxgwn;  // RwI=Exp(theta*^v), or we can call it vn=(gI x gw)/||gI x gw||
  // notice theta*^v belongs to [-Pi,Pi]*|^v| though theta belongs to [0,Pi]
  double theta = std::atan2(normgInxgwn, gIn.dot(gwn));  // ORB3 code means acos(gIn.dot(gwn)) is enough
  RwI = RwI_ = SO3exd::exp(vhat.cast<double>() * theta).cast<Tcalc_sgba>();  // RwI for print

  VectorXcalc w2;
  double cond_num2;
  if (!skip_step3) {
    // old_coeff_bg = 1; we found 1e3~5e4*(best_gw 7.5e3) for 10s,  5e4* for 0.35s and 2e4~1e8(best_gw 1e8) for 0.08s
    vector<double> coeff_deltat_prior(num_handlers);
    for (int h = 0; h < num_handlers; ++h) {
      coeff_deltat_prior[h] = kCoeffPriorDefault ? kCoeffDeltatPrior[0] : 5.e4 * 2 / (Ns[h] - 2);
    }

    // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
    int num_eq2 = num_eq;
    for (int hf = 0; hf < num_handlers_fixed; ++hf) {
      if (2 < NsFixed[hf]) num_eq2 += NsFixed[hf] - 2;
    }
    size_t dim_b = 3 * (num_eq2 + num_handlers);           //+num_handlers for n sigma_ba_inv * ba = 0
    size_t dim_x = num_var_opt2 + (num_handlers - 1) * 3;  // 5/6 unknowns so N must >=4
    MatrixXXcalc C(dim_b, dim_x);
    VectorXcalc D(dim_b);
    assert(Rcbs.size() >= num_handlers && pcbs.size() == Rcbs.size());

    //   for (int k=0;k<2;++k){//we prefer 1 iteration
    //     if (k==1){
    //       for(int i=0;i<N;++i) vKFInit[i]->ba_=bas_star[i];
    //       for(int i=1;i<N;++i) vKFInit[i]->ComputePreInt();
    //     } else {
    //       RwI_ = RwI;
    //     }

    C.setZero();
    D.setZero();
    num_eq2 = 0;
    for (int h = 0; h < num_handlers; ++h) {
      int num_eq_h = 0;
      double sum_dt_tmp = 0;

      FillAxeqb(*vKFsInit[h], num_var_opt2, h, &C, &D, num_eq2, fixmode_init, &s_, true, true, &num_eq_h, &reduced_hids,
                &id_Tcbs, &Rcbs, &pcbs, nullptr, &RwI_, &GI, &sum_dt_tmp);

      // TODO: ORB3 suggests sqrt_Info_ba = 1e5 for mono but sqrt(1e5) for stereo and niters = 200
      Tcalc_sgba info_ba = sqrt(IMUData::mInvSigmaba2);  //[id_cams2[h]]);

      // add priorA limit like ORB3 to make ba not far away from 0
#define ORB3_STRATEGY
#ifdef ORB3_STRATEGY
      C.block<3, 3>(3 * (num_eq2_ref + h), num_var_opt2 - 3 + h * 3) = sqrt(1e5) * Matrix3calc::Identity();
#else
      C.block<3, 3>(3 * (num_eq2_ref + h), num_var_opt2 - 3 + h * 3) =
          (info_ba / sqrt(sum_dt_tmp * coeff_deltat_prior[h]) * Matrix3calc::Identity()).cast<Tcalc_sgba>();
      //                C.block<3, 3>(3 * (num_eq2_ref + h), num_var_opt2 - 3 + h * 3) = Matrix3calc::Identity();
#endif

      if (kVerbDeb < verbose && Rcbs[h].size() != num_eq_h + 2)
        PRINT_DEBUG_INFO("Error "
                             << "Rcb[" << h << "].size=" << Rcbs[h].size() << ";num_eq_h=" << num_eq_h << endl,
                         imu_tightly_debug_path, "imu_init_thread_debug.txt");
    }
    if (num_eq2_ref != num_eq2) {
      PRINT_DEBUG_INFO("Error "
                           << "num_eq2 in Step3 != num_eq2_ref in Step2!" << endl,
                       imu_tightly_debug_path, "imu_init_thread_debug.txt");
    }
    assert(num_eq2 == num_eq2_ref);
    num_eq2 += num_handlers;  // priorA or ba limit

    if (kFixAll > fixMode && kFixNone < fixMode) {
      for (int hf = 0; hf < num_handlers_fixed; ++hf) {
        size_t iend = vKFsFixed[hf].size();
        vector<IMUKeyFrameInitFix *> vpfbsfixed(iend);
        for (size_t i = 0; i < iend; ++i) {
          vpfbsfixed[i] = &vKFsFixed[hf][i];
        }
        FillAxeqb(vpfbsfixed, num_var_opt2, hf, &C, &D, num_eq2, fixMode, &s_, false, true, nullptr, nullptr, nullptr,
                  nullptr, nullptr, nullptr, &RwI_, &GI);
      }

      // reduce C,D rows to speed up
      dim_b = 3 * num_eq2;
      C.resize(dim_b, dim_x);
      D.resize(dim_b);
    }
    // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
    // Note w2 is 6x1 vector by SVDecomp(), for Recording
    Eigen::JacobiSVD<MatrixXXcalc> svdC(C, Eigen::ComputeThinU | Eigen::ComputeThinV);
    w2 = svdC.singularValues();
    cond_num2 = w2(0) / w2(w2.size() - 1);
#ifdef VERBOSE_DEBUG
    PRINT_DEBUG_INFO("cond_num2=" << cond_num2 << " > thresh_cond(" << thresh_cond << ")" << endl,
                     imu_tightly_debug_path, "imu_init_thread_debug.txt")
#endif
    if (isnan(cond_num2) || cond_num2 > thresh_cond) {
      deleteKFs_ret(vKFsInit);
      if (verbose) {
        PRINT_DEBUG_INFO("cond_num2=" << cond_num2 << " > thresh_cond(" << thresh_cond << ")" << endl,
                         imu_tightly_debug_path, "imu_init_thread_debug.txt");
      }
      return 0;
    }
    MatrixXXcalc u2 = svdC.matrixU();
    MatrixXXcalc v2 = svdC.matrixV();
    MatrixXXcalc w2inv = MatrixXXcalc::Identity(w2.size(), w2.size());
    for (int i = 0; i < w2.size(); ++i) {
      //            if (fabs(w2(i)) < 1e-10) {
      //                w2(i) += 1e-10;
      //                cerr << "w2("<<i<<") < 1e-10, w=" << endl << w2 << endl;
      //            }
      w2inv(i, i) = 1. / w2(i);
    }
    VectorXcalc y = v2 * (w2inv * (u2.transpose() * D));  // Then y/x = vt'*winv*u'*D
    if (!(kFixS & fixMode)) s_ = y(0);                    // s*_C, C means IV-C in the paper
    if (!(kFixG & fixMode)) {
      // small deltatheta/dtheta=[dthetaxy.t() 0].t()
      Vector3d dthetaeig(y(num_var_opt2 - 5), y(num_var_opt2 - 4), 0);
      RwI_ = (RwI.cast<double>() * SO3exd::exp(dthetaeig)).cast<Tcalc_sgba>();  // RwI*_C=RwI*_B*Exp(dtheta)
    }
    //     if (k==0)
    for (int h = 0; h < num_handlers; ++h) {
      // here bai_bar may exist, so ba = bai_bar + dba
      bas_star[h] += y.segment(num_var_opt2 - 3 + h * 3, 3).cast<double>();
    }

    //   }
  } else {
    //            int niters = 200;
    //            Optimizer::InertialOptimization(vKFsInit2, this, RwI_, bas_star, &abortBA_, mpMap, local_window_size_,
    //            niters, false, true);//inertial only BA.

    w2 = w;
    cond_num2 = cond_num;
  }

  // Record data for analysis
  // direction of gwbefore is the same as gwstar, but value is different!
  double end_time = -1, start_time = -1;
  for (auto vkf_init : vKFsInit)
    if (end_time < vkf_init->back()->timestamp_) {
      end_time = vkf_init->back()->timestamp_;
      start_time = vkf_init->front()->timestamp_;
    }
  Vector3calc gwbefore = RwI * GI, gwafter = RwI_ * GI;
  if (verbose) {
    PRINT_DEBUG_INFO("Step3: gwbefore=" << gwbefore.transpose() << ", gwafter=" << gwafter.transpose() << ", ba_star=",
                     imu_tightly_debug_path, "imu_init_thread_debug.txt");
    for (size_t h = 0; h < num_handlers; ++h)
      PRINT_DEBUG_INFO(bas_star[h].transpose() << ";", imu_tightly_debug_path, "imu_init_thread_debug.txt");
    PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
    // Debug the frequency & sstar2&sstar
    PRINT_DEBUG_INFO("Time: " << fixed << setprecision(9) << end_time - mdStartTime << ", s_star: " << s_star
                              << ", s: " << s_ << defaultfloat << endl,
                     imu_tightly_debug_path, "imu_init_thread_debug.txt");
  }

  if (mTmpfilepath.length() > 0) {  // Debug the Rwistar2
    ofstream fRwi(mTmpfilepath + "Rwi.txt");
    fRwi << RwI_.unit_quaternion().vec().transpose() << endl;
    fRwi.close();

    fgw << end_time << " " << gwafter.transpose() << " " << gwbefore.transpose() << " " << endl;
    fscale << end_time << " " << s_ << " " << s_star << " " << endl;
    for (int h = 0; h < num_handlers; ++h) {
      fbiasg << (int)id_cams2[h] << " " << end_time << " " << bgs_est[h].transpose() << " " << endl;
      fbiasa << (int)id_cams2[h] << " " << end_time << " " << bas_star[h].transpose() << " " << endl;
    }
    fcondnum << end_time << " " << w2.transpose() << " " << endl;
  }

  // ********************************
  // Todo: Add some logic or strategy to confirm init status, VIORBSLAM paper just uses 15 seconds to confirm
  int8_t bVIOInited = 0;
  PRINT_DEBUG_INFO(yellowSTR "condnum=" << cond_num2 << ";max=" << w2(0) << ";min=" << w2(w2.size() - 1)
                                        << "; time_span=" << end_time - start_time << whiteSTR << endl,
                   imu_tightly_debug_path, "imu_init_thread_debug.txt");
  //  int min_cond_num = 600;  // try 100 for max_dt ~ 0.1s; try 600/500/650 for no priorA limit
  //        cond_num2 < min_cond_num &&
  if (!pdebug_benough_id_cam) {  // 15s in the paper V-A
    bVIOInited = 1;
  } else {
    bool bnew_imu_init = false;
    for (int h = 0; h < pdebug_benough_id_cam->size(); ++h) {
      if (!last_imu_inited[h] && (*pdebug_benough_id_cam)[h]) bnew_imu_init = true;
      //              cout<<"check benoughidcam[h]="<<(int)(*pdebug_benough_id_cam)[h]<<endl;
    }
    if (bnew_imu_init) {
      PRINT_DEBUG_INFO("debug_check time_span_upper_limit = " << end_time - start_time << ">=" << mdFinalTime << endl,
                       imu_tightly_debug_path, "imu_init_thread_debug.txt");
      bVIOInited = 1;
    }
  }

  // if VIO is initialized with appropriate bg*,s*,gw*,ba*, update the map(MPs' P,KFs' PRVB) like GBA/CorrrectLoop()
  if (bVIOInited) {
    if (GetFinishRequest()) {
      deleteKFs_ret(vKFsInit);
      return -2;
    }

    // gravity vector in world frame
    Vector3f gw;
    {
      unique_lock<mutex> lock(mMutexInitIMU);
      gw = (RwI_ * GI).cast<float>();
      mGravityVec = Converter::toCvMat(Vector3d(gw.cast<double>()));
    }

    // Update the Map needs mutex lock: Stop local mapping, like RunGlobalBundleAdjustment() in LoopClosing.cc
    // same as CorrectLoop(), suspend/stop/freeze LocalMapping thread(Map main tread here)
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    //  if LocalMapping thread is killed, don't wait any more
    while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {  //) {  //&&
      // mpLocalMapper->Stop();  // simulate local mapping thread stop() for we incorporate it into imu init thread
      usleep(1000);
    }
    //    // for localMapping thread is also this thread, we should avoid Release delete KFs
    //    while (CheckNewKeyFrame()) {
    //      ProcessNewKeyFrame();
    //    }

    // notice we cannot update scale during LoopClosing or LocalBA!
    unique_lock<mutex> lockScale(mpMap->mMutexScaleUpdateLoopClosing);
    unique_lock<mutex> lockScale2(mpMap->mMutexScaleUpdateGBA);
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate, std::defer_lock);  // Get Map Mutex
    while (!lock.try_lock()) {
      if (GetReset()) {
        mpLocalMapper->Release();  // recover LocalMapping thread, same as CorrectLoop()

        deleteKFs_ret(vKFsInit);
        return -2;
      }
      usleep(3000);
    }
    // Update KFs' PRVB
    // update the vfb_scale_v_b to the current size, and pNewestKF is mpCurrentKeyFrame during the
    // LocalMapping thread is stopped
    // SLAM could use mpMap->GetAllKeyFrames();
    auto vkfs_all = mpMap->GetAllKeyFrames();
    vector<FrameBase *> vfb_scale_v_b;  // mpMap->GetLastFBs(local_window_size_ + mpMap->GetUnfixedCamNum(), 0);
    vfb_scale_v_b.reserve(vkfs_all.size());
    for (auto &pkf : vkfs_all) vfb_scale_v_b.emplace_back(pkf);
    // they must be same for we change the set less func. in Map.h
    assert((GetCurrentKeyFrame())->timestamp_ == vfb_scale_v_b.back()->timestamp_);

    bool bfast_init = false;  // true;//

    // Update NavState(v + scale + bias) for all KeyFrames
    vector<FrameBase *> pcurfbs;  // mpMap->GetvpnewestFB();
    pcurfbs.push_back(GetCurrentKeyFrame());
    vector<unsigned long> fixed_ref_ids;  // = mpMap->GetvFixedRefID();
    fixed_ref_ids.push_back(0);
    bVIOInited = InitIMUv_lbags<IMUData>(vfb_scale_v_b, pcurfbs, fixed_ref_ids, id_cams2, last_imu_inited, fixMode,
                                         bgs_est, bas_star, gw, s_, mpMap, lock, mode_lba, bfast_init);

    /*
    vector<FrameBase *> pnewestkfs;
    vector<unsigned long> fixed_ref_ids;
    aligned_list<IMUDataUsed> lodoms;
    vector<typename aligned_list<IMUDataUsed>::const_iterator> viterbeg;
    vector<typename aligned_list<IMUDataUsed>::const_iterator>::iterator last_iterbeg = viterbeg.end();
    for (size_t i = 0, iend = vfb_scale_v_b.size(); i < iend; ++i) {
      int8_t id_cam = vfb_scale_v_b[i]->id_cam_;
      if (id_cam + 1 > pnewestkfs.size()) {
        pnewestkfs.resize(id_cam + 1, nullptr);
        fixed_ref_ids.resize(id_cam + 1);
      }
      if (!pnewestkfs[id_cam]) {
        fixed_ref_ids[id_cam] = vfb_scale_v_b[i]->mnId;
      }
      pnewestkfs[id_cam] = vfb_scale_v_b[i];
      FrameBase *pnextfb = vfb_scale_v_b[i]->GetNextFrameBase();
      aligned_list<IMUDataUsed> lodom_tmp;
      if (pnextfb) {
        IMUPreintegratorUsed imupreint_tmp;
        pnextfb->GetIMUPreIntCopied(imupreint_tmp);
        lodom_tmp.swap(imupreint_tmp.GetRawDataRef());
      }
      bool has_vals = lodoms.size(), has_vals_tmp = lodom_tmp.size();
      if (has_vals && has_vals_tmp) {
        bool bminus = last_iterbeg == viterbeg.end();
        viterbeg.push_back(--lodoms.end());
        if (bminus)
          last_iterbeg = --viterbeg.end();
      }
      lodoms.insert(lodoms.end(), lodom_tmp.begin(), lodom_tmp.end());
      if (has_vals_tmp) {
        if (has_vals) {
          auto iterref = ++viterbeg.back();
          for (;last_iterbeg != viterbeg.end(); ++last_iterbeg) {
            *last_iterbeg = iterref;
          }
          assert(viterbeg.back() != lodoms.end());
        } else {
          bool bend = last_iterbeg == viterbeg.end();
          viterbeg.push_back(lodoms.begin());
          if (bend)
            last_iterbeg = viterbeg.end();
        }
      } else {
        viterbeg.push_back(lodoms.end());
        last_iterbeg = --viterbeg.end();
      }
    }
    viterbeg.push_back(lodoms.end());
    vector<FrameBase *> pnewestkfsref = mpMap->GetvpnewestFB();
    vector<unsigned long> fixed_ref_idsref = mpMap->GetvFixedRefID();
    for (size_t i = 0; i < pnewestkfsref.size(); ++i) {
      if (i < pnewestkfs.size() && pnewestkfs[i] != pnewestkfsref[i])
        return 0;
      else
        break;
    }
    bVIOInited = InitIMUv_lbags<IMUDataUsed>(vfb_scale_v_b, pnewestkfsref, fixed_ref_idsref, id_cams2,
    last_imu_inited, fixMode, bgs_est, bas_star, gw, s_, mpMap, mode_lba, plock, &viterbeg);*/

    if (!bfast_init) {
      // ensure new frame could use imu info, then we can release local map
      while (!lock.try_lock()) {
        if (GetReset()) {
          mpLocalMapper->Release();  // recover LocalMapping thread, same as CorrectLoop()
          //          int8_t id_cam = Getreset_id_cam_();
          // now this framework will lose some efficiency when bad situation happens
          if (bVIOInited > 0) bVIOInited = -2;

          deleteKFs_ret(vKFsInit);
          return bVIOInited;
        }
        usleep(3000);
      }
      if (bVIOInited > 0)
        SetIMUInited(last_imu_inited, id_cams2);  // we've to put it after lock for maybe reset happens
      // though lba may Informed, if no fast_init, still may happen 1st IMUInited but map unchanged situation!
      mpMap->InformNewChange();
      lock.unlock();
      mpLocalMapper->Release();
    }
  }

  deleteKFs_ret(vKFsInit);
  return bVIOInited;
}

// TODO: check se3 approximation error, so3 has been checked
int IMUInitialization::TryInitVIO_zzh(void) {
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  // at least N>=4
  if (!mpMap || mpMap->KeyFramesInMap() < 4) {
    return 0;
  }  // 21,11, 4

  // Cache KFs / wait for KeyFrameCulling() over
  while (GetCopyInitKFs()) {
    if (kVerbDeb < verbose) PRINT_DEBUG_INFO(".", imu_tightly_debug_path, "imu_init_thread_debug.txt");
    usleep(1000);
  }
  if (kVerbDeb < verbose)
    PRINT_DEBUG_INFO(endl << "copy init KFs...", imu_tightly_debug_path, "imu_init_thread_debug.txt");
  SetCopyInitKFs(true);  // stop KeyFrameCulling() when this copying KFs

  // see VIORBSLAM paper IV, here N=all KFs in map, not the meaning of local KFs' number
  // Use all KeyFrames in map to compute
  //        vector<KeyFrame *> vkf_scale_v_b = mpMap->GetAllKeyFrames();
  vector<bool> benough_id_cam;
  vector<KeyFrame *> vkf_scale_v_b = mpMap->GetLastKFs(mdFinalTime, benough_id_cam, mdFinalTime + 0.5, 3.0, 2, 4);
  assert(benough_id_cam.size() <= num_imus_);
  assert(vkf_scale_v_b.size());
  int NvSGKF = vkf_scale_v_b.size();
  PRINT_DEBUG_INFO("NvSGKF=" << vkf_scale_v_b.size() << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");
  vector<bool> last_imu_inited = GetInitedIMUs();
  bool debug_init_imu = false;  // true;
  if (!debug_init_imu) {
    bool bnew_imu_init = false;
    for (int h = 0; h < benough_id_cam.size(); ++h)
      if (!last_imu_inited[h] && benough_id_cam[h]) bnew_imu_init = true;
    if (!bnew_imu_init) {
      SetCopyInitKFs(false);
      return 0;
    }
  }
  // Store initialization-required KeyFrame data
  vector<vector<IMUKeyFrameInit *> *> vKFsInit;
  vector<IMUKeyFrameInit *> lastKFInit;
  aligned_vector<aligned_vector<IMUKeyFrameInitFix>> vKFsFixed;
  vector<IMUKeyFrameInitFix *> lastKFFixed;
  // firstly suppose no inited imu, so no bgba/v to be fixed
  int fixMode = kIMUInitFixScale & mode_init_ ? kFixS : kFixNone;
  int8_t lba_mode = kIMUInitLBA & mode_init_ ? kLBAAll : kLBANone;
  if (kIMUInitFixGravity & mode_init_) {
    fixMode |= kFixG;
    lba_mode |= kLBAFixGS | kLBAFixBias;
  }
  //  lba_mode |= kLBAFixPR | kLBAFixBias;

  vector<int8_t> id_cam_to_i(num_imus_, -1);
  size_t num_cam_used = 0, num_cam_fixed = 0;
  vector<int8_t> id_cams2;
  for (int i = 0; i < NvSGKF; ++i) {
    KeyFrame *pKF = vkf_scale_v_b[i];
    int8_t id_cam = pKF->id_cam_;
    assert(id_cam < num_imus_);
    if (last_imu_inited[id_cam]) {
      fixMode |= kFixBgBa | kFixV;  // for situation last_imu_inited[id_cam] could be true; we found kFixV better
      if (kFixAll < fixMode) {
        IMUKeyFrameInitFix kfi(*pKF);
        if (0 > id_cam_to_i[id_cam]) {
          id_cam_to_i[id_cam] = (int8_t)num_cam_fixed;

          ++num_cam_fixed;
          vKFsFixed.resize(num_cam_fixed);
          lastKFFixed.resize(num_cam_fixed, NULL);  // notice resize only change val when it's appended!
        }
        int8_t id_cam2i = id_cam_to_i[id_cam];
        if (lastKFFixed[id_cam2i]) kfi.mpPrevKeyFrame = lastKFFixed[id_cam2i];

        vKFsFixed[id_cam2i].push_back(kfi);
        lastKFFixed[id_cam2i] = &vKFsFixed[id_cam2i].back();
      }

      continue;
    }

    IMUKeyFrameInit *pkfi = new IMUKeyFrameInit(*pKF);
    if (0 > id_cam_to_i[id_cam]) {
      id_cam_to_i[id_cam] = (int8_t)num_cam_used;
      id_cams2.push_back(id_cam);

      ++num_cam_used;
      vKFsInit.resize(num_cam_used, new vector<IMUKeyFrameInit *>());
      lastKFInit.resize(num_cam_used, NULL);
    }
    int8_t id_cam2i = id_cam_to_i[id_cam];
    if (lastKFInit[id_cam2i]) pkfi->mpPrevKeyFrame = lastKFInit[id_cam2i];
    lastKFInit[id_cam2i] = pkfi;

    vKFsInit[id_cam2i]->push_back(pkfi);
  }

  SetCopyInitKFs(false);
  if (kVerbDeb < verbose) PRINT_DEBUG_INFO("...end" << endl, imu_tightly_debug_path, "imu_init_thread_debug.txt");

  vector<Vector6d> vbgba_init(lastKFInit.size());
  for (int h = 0; h < vbgba_init.size(); ++h) {
    vbgba_init[h].segment<3>(0) = lastKFInit[h]->bg_;
    vbgba_init[h].segment<3>(3) = lastKFInit[h]->ba_;
    if (verbose)
      PRINT_DEBUG_INFO("initba=" << vbgba_init[h].segment<3>(3).transpose()
                                 << " bg=" << vbgba_init[h].segment<3>(0).transpose() << endl,
                       imu_tightly_debug_path, "imu_init_thread_debug.txt");
  }

  int ret_val = InitIMU(vKFsInit, vKFsFixed, id_cams2, last_imu_inited, fixMode, lba_mode, vbgba_init,
                        debug_init_imu ? &benough_id_cam : nullptr);

  PRINT_DEBUG_INFO(yellowSTR " Used time in IMU Initialization="
                       << chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count()
                       << whiteSTR << " " << ret_val << endl,
                   imu_tightly_debug_path, "imu_init_thread_debug.txt");
  return ret_val;
}

bool IMUInitialization::TryInitVIO(
    void) {  // now it's the version cannot allow the KFs has no inter IMUData in initial 15s!!!
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  // at least N>=4
  if (mpMap->KeyFramesInMap() < 4) {
    return false;
  }  // 21,11, 4
  // Recording data in txt files for further analysis
  static bool fopened = false;
  static ofstream fgw, fscale, fbiasa, fcondnum, fbiasg;
  if (mTmpfilepath.length() > 0 && !fopened) {
    // Need to modify this to correct path
    fbiasg.open(mTmpfilepath + "biasg.txt");  // optimized initial bg for these N KFs,3*1
    fgw.open(mTmpfilepath + "gw.txt");        // gwafter then gw before,6*1
    fscale.open(mTmpfilepath +
                "scale.txt");  // scale_fine then scale_rough, 2*1 only for Monocular camera//if (mbMonocular)
    fbiasa.open(mTmpfilepath + "biasa.txt");      // optimized initial ba for these N KFs,3*1
    fcondnum.open(mTmpfilepath + "condnum.txt");  // for optimized x is 6*1 vector, see (19) in VOIRBSLAM paper, here
                                                  // just show these 6*1 raw data
    if (fbiasg.is_open() && fgw.is_open() && (fscale.is_open()) &&  //! mbMonocular||
        fbiasa.is_open() && fcondnum.is_open())
      fopened = true;
    else {
      cerr << "file open error in TryInitVIO" << endl;
      fopened = false;
    }
    fbiasg << std::fixed << std::setprecision(6);
    fgw << std::fixed << std::setprecision(6);
    fscale << std::fixed << std::setprecision(6);
    fbiasa << std::fixed << std::setprecision(6);
    fcondnum << std::fixed << std::setprecision(6);
  }

  //   Optimizer::GlobalBundleAdjustment(mpMap, 10);//GBA by only vision 1stly, suggested by JingWang

  // Extrinsics
  cv::Mat Tbc = Frame::mTbc;
  cv::Mat Rbc = Tbc.rowRange(0, 3).colRange(0, 3);
  cv::Mat pbc = Tbc.rowRange(0, 3).col(3);
  cv::Mat Rcb = Rbc.t();
  cv::Mat pcb = -Rcb * pbc;

  // Cache KFs / wait for KeyFrameCulling() over
  while (GetCopyInitKFs()) usleep(1000);
  SetCopyInitKFs(true);  // stop KeyFrameCulling() when this copying KFs
  //   if(mpMap->KeyFramesInMap()<4){ SetCopyInitKFs(false);return false;}//ensure no KeyFrameCulling() during the start
  //   of this func. till here

  // see VIORBSLAM paper IV, here N=all KFs in map, not the meaning of local KFs' number
  // Use all KeyFrames in map to compute
  vector<KeyFrame *> vScaleGravityKF = mpMap->GetAllKeyFrames();
  // sort(vScaleGravityKF.begin(),vScaleGravityKF.end(),[](const KeyFrame *a,const KeyFrame *b){return
  // a->mnId<b->mnId;});//we change the set less/compare func. so that we don't need to sort them!
  assert((*vScaleGravityKF.begin())->mnId == 0);
  int N = 0, NvSGKF = vScaleGravityKF.size();
  KeyFrame *pNewestKF = vScaleGravityKF[NvSGKF - 1];
  // Store initialization-required KeyFrame data
  vector<IMUKeyFrameInit *> vKFInit;

  for (int i = 0; i < NvSGKF; ++i) {
    KeyFrame *pKF = vScaleGravityKF[i];
    //     if (pKF->timestamp_<pNewestKF->timestamp_-15) continue;//15s as the VIORBSLAM paper
    IMUKeyFrameInit *pkfi = new IMUKeyFrameInit(*pKF);
    if (N > 0) pkfi->mpPrevKeyFrame = vKFInit[N - 1];
    vKFInit.push_back(pkfi);
    ++N;
  }

  SetCopyInitKFs(false);

  // Step 1. / see VIORBSLAM paper IV-A
  // Try to compute initial gyro bias, using optimization with Gauss-Newton
  Vector3d bgest = Vector3d::Zero();
  Optimizer::OptimizeInitialGyroBias<IMUKeyFrameInit>(vKFInit,
                                                      bgest);  // nothing changed, just return the optimized result bg*
  cout << "bgest: " << bgest << endl;

  // Update biasg and pre-integration in LocalWindow(here all KFs).
  for (int i = 0; i < N; ++i) vKFInit[i]->bg_ = bgest;
  for (int i = 1; i < N; ++i)
    vKFInit[i]->ComputePreInt();  // so vKFInit[i].mOdomPreIntIMU is based on bg_bar=bgest,ba_bar=0; dbg=0 but dba/ba
                                  // waits to be optimized

  // Step 2. / See VIORBSLAM paper IV-B
  // Approx Scale and Gravity vector in 'world' frame (first/0th KF's camera frame)
  // Solve A*x=B for x=[s,gw] 4x1 vector, using SVD method
  cv::Mat A = cv::Mat::zeros(3 * (N - 2), 4, CV_32F);  // 4 unknowns so N must >=4
  cv::Mat B = cv::Mat::zeros(3 * (N - 2), 1, CV_32F);
  cv::Mat I3 = cv::Mat::eye(3, 3, CV_32F);
  int numEquations = 0;
  for (int i = 0; i < N - 2; ++i) {
    IMUKeyFrameInit *pKF2 = vKFInit[i + 1], *pKF3 = vKFInit[i + 2];
    double dt12 = pKF2->mOdomPreIntIMU.mdeltatij;  // deltat12
    double dt23 = pKF3->mOdomPreIntIMU.mdeltatij;
    if (dt12 == 0 || dt23 == 0) {
      cout << redSTR << "Tm=" << pKF2->timestamp_ << " lack IMU data!" << whiteSTR << endl;
      continue;
    }
    ++numEquations;
    // Pre-integrated measurements
    cv::Mat dp12 = Converter::toCvMat(pKF2->mOdomPreIntIMU.mpij);  // deltap12
    cv::Mat dv12 = Converter::toCvMat(pKF2->mOdomPreIntIMU.mvij);
    cv::Mat dp23 = Converter::toCvMat(pKF3->mOdomPreIntIMU.mpij);
    //     cout<<fixed<<setprecision(6);
    //     cout<<"dt12:"<<dt12<<" KF1:"<<vKFInit[i]->timestamp_<<" KF2:"<<pKF2->timestamp_<<" dt23:"<<dt23<<"
    //     KF3:"<<pKF3->timestamp_<<endl; cout<<dp12.t()<<" 1id:"<<vScaleGravityKF[i]->mnId<<"
    //     2id:"<<vScaleGravityKF[i+1]->mnId<<" 3id:"<<vScaleGravityKF[i+2]->mnId<<endl; cout<<"
    //     Size12="<<pKF2->mOdomPreIntIMU.getlOdom().size()<<" Size23="<<pKF3->mOdomPreIntIMU.getlOdom().size()<<endl;
    // Pose of camera in world frame
    cv::Mat Twc1 = vKFInit[i]->mTwc;  // Twci for pwci&Rwci, not necessary for clone()
    cv::Mat Twc2 = pKF2->mTwc;
    cv::Mat Twc3 = pKF3->mTwc;
    cv::Mat pc1 = Twc1.rowRange(0, 3).col(3);  // pwci
    cv::Mat pc2 = Twc2.rowRange(0, 3).col(3);
    cv::Mat pc3 = Twc3.rowRange(0, 3).col(3);
    cv::Mat Rc1 = Twc1.rowRange(0, 3).colRange(0, 3);  // Rwci
    cv::Mat Rc2 = Twc2.rowRange(0, 3).colRange(0, 3);
    cv::Mat Rc3 = Twc3.rowRange(0, 3).colRange(0, 3);

    // fill A/B matrix: lambda*s + beta*g = gamma(3*1), Ai(3*4)=[lambda beta], (13) in the paper
    cv::Mat lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
    cv::Mat beta = (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 * I3;
    cv::Mat gamma = (Rc1 - Rc2) * pcb * dt23 + (Rc3 - Rc2) * pcb * dt12 - Rc2 * Rcb * dp23 * dt12 -
                    Rc1 * Rcb * dv12 * dt12 * dt23 + Rc1 * Rcb * dp12 * dt23;
    lambda.copyTo(A.rowRange(3 * i + 0, 3 * i + 3).col(0));
    beta.copyTo(A.rowRange(3 * i + 0, 3 * i + 3).colRange(1, 4));  // Ai
    gamma.copyTo(B.rowRange(3 * i + 0, 3 * i + 3));                // gamma/B(i), but called gamma(i) in the paper
    // JingWang tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx, or we can say the
    // papaer missed a minus before γ(i)
  }
  if (numEquations < 4) {          // for more robust judgement instead of judging KeyFramesInMap()
    for (int i = 0; i < N; i++) {  // delete the newed pointer
      if (vKFInit[i]) delete vKFInit[i];
    }
    return false;
  }
  // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
  // A=u*S*vt=u*w*vt, u*w*vt*x=B => x=vt'*winv*u'*B, or we call the pseudo inverse of A/A.inv()=(A.t()*A).inv()*A.t(),
  // in SVD we have A.inv()=v*winv*u.t() where winv is the w with all nonzero term is the reciprocal of the
  // corresponding singular value
  cv::Mat w, u, vt;  // Note w is 4x1 vector by SVDecomp()/SVD::compute() not the 4*4(not FULL_UV)/m*n(FULL_UV) singular
                     // matrix we stated last line
  cv::SVD::compute(
      A, w, u, vt,
      cv::SVD::MODIFY_A);  // A is changed in SVDecomp()(just calling the SVD::compute) with cv::SVD::MODIFY_A for speed
  cv::Mat winv = cv::Mat::eye(4, 4, CV_32F);
  for (int i = 0; i < 4; ++i) {
    if (fabs(w.at<float>(i)) < 1e-10) {  // too small in sufficient w meaning the linear dependent equations causing the
                                         // solution is not unique(or A.inv() not exist)
      w.at<float>(i) += 1e-10;
      cerr << "w(i) < 1e-10, w=" << endl << w << endl;
    }
    winv.at<float>(i, i) = 1. / w.at<float>(i);
  }
  cv::Mat x = vt.t() * winv * u.t() * B;
  double sstar = x.at<float>(0);      // scale should be positive
  cv::Mat gwstar = x.rowRange(1, 4);  // gravity should be about ~9.8
  cout << "gwstar: " << gwstar.t() << ", |gwstar|=" << cv::norm(gwstar) << endl;

  // Step 3. / See VIORBSLAM paper IV-C
  cv::Mat Rwi;              // for Recording
  cv::Mat w2, u2, vt2;      // Note w2 is 6x1 vector by SVDecomp(), for Recording
  Eigen::Matrix3d Rwieig_;  // for Recording
  // Use gravity magnitude 9.810 as constraint; gIn/^gI=[0;0;1], the normalized gravity vector in an inertial frame, we
  // can also choose gIn=[0;0;-1] as the VIORBSLAM paper
  cv::Mat gIn = cv::Mat::zeros(3, 1, CV_32F);
  gIn.at<float>(2) = 1;
  cv::Mat GI = gIn * IMUData::mdRefG;  // gI or GI=^gI*G
  double s_;
  cv::Mat Rwi_;
  Vector3d bastareig;
  //   for (int k=0;k<2;++k){//we prefer 1 iteration
  //     if (k==1){
  //       gwstar=Rwi_*GI;
  //       for(int i=0;i<N;++i) vKFInit[i]->ba_=bastareig;
  //       for(int i=1;i<N;++i) vKFInit[i]->ComputePreInt();
  //     }

  cv::Mat gwn = gwstar / cv::norm(gwstar);  //^gw=gw*/||gw*|| / Normalized approx. gravity vecotr in world frame
  cv::Mat gInxgwn = gIn.cross(gwn);
  double normgInxgwn = cv::norm(gInxgwn);
  cv::Mat vhat = gInxgwn / normgInxgwn;  // RwI=Exp(theta*^v), or we can call it vn=(gI x gw)/||gI x gw||
  double theta =
      std::atan2(normgInxgwn, gIn.dot(gwn));  // notice theta*^v belongs to [-Pi,Pi]*|^v| though theta belongs to [0,Pi]
  Matrix3d RWIeig = Sophus::SO3exd::Exp(Converter::toVector3d(vhat) * theta);
  Rwi = Converter::toCvMat(RWIeig);  // RwI

  // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
  cv::Mat C = cv::Mat::zeros(3 * (N - 2), 6, CV_32F);
  cv::Mat D = cv::Mat::zeros(3 * (N - 2), 1, CV_32F);
  for (int i = 0; i < N - 2; i++) {
    IMUKeyFrameInit *pKF2 = vKFInit[i + 1], *pKF3 = vKFInit[i + 2];
    const IMUPreintegrator &imupreint12 = pKF2->mOdomPreIntIMU, &imupreint23 = pKF3->mOdomPreIntIMU;
    // d means delta
    double dt12 = imupreint12.mdeltatij;
    double dt23 = imupreint23.mdeltatij;
    if (dt12 == 0 || dt23 == 0) continue;
    cv::Mat dp12 = Converter::toCvMat(imupreint12.mpij);
    cv::Mat dp23 = Converter::toCvMat(imupreint23.mpij);
    cv::Mat dv12 = Converter::toCvMat(imupreint12.mvij);
    cv::Mat Jav12 = Converter::toCvMat(imupreint12.mJavij);
    cv::Mat Jap12 = Converter::toCvMat(imupreint12.mJapij);
    cv::Mat Jap23 = Converter::toCvMat(imupreint23.mJapij);
    cv::Mat Twc1 = vKFInit[i]->mTwc;  // Twci for pwci&Rwci, not necessary for clone()
    cv::Mat Twc2 = pKF2->mTwc;
    cv::Mat Twc3 = pKF3->mTwc;
    cv::Mat pc1 = Twc1.rowRange(0, 3).col(3);  // pwci
    cv::Mat pc2 = Twc2.rowRange(0, 3).col(3);
    cv::Mat pc3 = Twc3.rowRange(0, 3).col(3);
    cv::Mat Rc1 = Twc1.rowRange(0, 3).colRange(0, 3);  // Rwci
    cv::Mat Rc2 = Twc2.rowRange(0, 3).colRange(0, 3);
    cv::Mat Rc3 = Twc3.rowRange(0, 3).colRange(0, 3);
    // Stack to C/D matrix; lambda*s + phi(:,0:1)*dthetaxy + zeta*ba = psi, Ci(3*6),Di/psi(3*1)
    cv::Mat lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;  // 3*1
    cv::Mat phi = -(dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 * Rwi *
                  SkewSymmetricMatrix(GI);  // 3*3 note: this has a '-', different to paper
    cv::Mat zeta = Rc2 * Rcb * Jap23 * dt12 + Rc1 * Rcb * Jav12 * dt12 * dt23 -
                   Rc1 * Rcb * Jap12 * dt23;  // 3*3 notice here is Jav12, paper writes a wrong Jav23
    cv::Mat psi = (Rc1 - Rc2) * pcb * dt23 + (Rc3 - Rc2) * pcb * dt12 - Rc2 * Rcb * dp23 * dt12 -
                  Rc1 * Rcb * dv12 * dt12 * dt23  // note:  - paper & deltatij^2 in paper means dt12^2*dt23+dt23^2*dt12
                  + Rc1 * Rcb * dp12 * dt23 -
                  (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 *
                      (Rwi * GI);  // notice here use Rwi*GI instead of gwstar for it's designed for iterative usage
    lambda.copyTo(C.rowRange(3 * i + 0, 3 * i + 3).col(0));
    phi.colRange(0, 2).copyTo(
        C.rowRange(3 * i + 0, 3 * i + 3).colRange(1, 3));  // phi(:,0:1)(3*2) / only the first 2 columns, third term in
                                                           // dtheta is zero, here compute dthetaxy 2x1.
    zeta.copyTo(C.rowRange(3 * i + 0, 3 * i + 3).colRange(3, 6));
    psi.copyTo(D.rowRange(3 * i + 0, 3 * i + 3));
  }
  // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
  cv::SVD::compute(C, w2, u2, vt2, cv::SVD::MODIFY_A);
  cv::Mat w2inv = cv::Mat::eye(6, 6, CV_32F);
  for (int i = 0; i < 6; ++i) {
    if (fabs(w2.at<float>(i)) < 1e-10) {
      w2.at<float>(i) += 1e-10;
      cerr << "w2(i) < 1e-10, w=" << endl << w2 << endl;
    }
    w2inv.at<float>(i, i) = 1. / w2.at<float>(i);
  }
  cv::Mat y = vt2.t() * w2inv * u2.t() * D;                      // Then y/x = vt'*winv*u'*D
  s_ = y.at<float>(0);                                           // s*_C, C means IV-C in the paper
  Eigen::Vector3d dthetaeig(y.at<float>(1), y.at<float>(2), 0);  // small deltatheta/dtheta=[dthetaxy.t() 0].t()
  Rwieig_ = RWIeig * Sophus::SO3exd::Exp(dthetaeig);             // RwI*_C=RwI*_B*Exp(dtheta)
  Rwi_ = Converter::toCvMat(Rwieig_);
  //     if (k==0)
  bastareig = Converter::toVector3d(y.rowRange(3, 6));  // here bai_bar=0, so dba=ba
  //     else bastareig+=Converter::toVector3d(y.rowRange(3,6));
  //   }

  // Record data for analysis
  cv::Mat gwbefore = Rwi * GI,
          gwafter = Rwi_ * GI;  // direction of gwbefore is the same as gwstar, but value is different!
  cout << "gwbefore=" << gwbefore << ", gwafter=" << gwafter << endl;

  cout << "Time: " << pNewestKF->timestamp_ - mdStartTime << ", sstar: " << sstar << ", s: " << s_
       << endl;  // Debug the frequency & sstar2&sstar
  //<<" bgest: "<<bgest.transpose()<<", gw*(gwafter)="<<gwafter.t()<<", |gw*|="<<cv::norm(gwafter)<<",
  // norm(gwbefore,gwstar)"<<cv::norm(gwbefore.t())<<" "<<cv::norm(gwstar.t())<<endl;
  if (mTmpfilepath.length() > 0) {  // Debug the Rwistar2
    ofstream fRwi(mTmpfilepath + "Rwi.txt");
    fRwi << Rwieig_(0, 0) << " " << Rwieig_(0, 1) << " " << Rwieig_(0, 2) << " " << Rwieig_(1, 0) << " "
         << Rwieig_(1, 1) << " " << Rwieig_(1, 2) << " " << Rwieig_(2, 0) << " " << Rwieig_(2, 1) << " "
         << Rwieig_(2, 2) << endl;
    fRwi.close();
  }
  fbiasg << pNewestKF->timestamp_ << " " << bgest(0) << " " << bgest(1) << " " << bgest(2) << " " << endl;
  fgw << pNewestKF->timestamp_ << " " << gwafter.at<float>(0) << " " << gwafter.at<float>(1) << " "
      << gwafter.at<float>(2) << " " << gwbefore.at<float>(0) << " " << gwbefore.at<float>(1) << " "
      << gwbefore.at<float>(2) << " " << endl;
  fscale << pNewestKF->timestamp_ << " " << s_ << " " << sstar << " " << endl;  // if (mbMonocular)
  fbiasa << pNewestKF->timestamp_ << " " << bastareig[0] << " " << bastareig[1] << " " << bastareig[2] << " " << endl;
  fcondnum << pNewestKF->timestamp_ << " " << w2.at<float>(0) << " " << w2.at<float>(1) << " " << w2.at<float>(2) << " "
           << w2.at<float>(3) << " " << w2.at<float>(4) << " " << w2.at<float>(5) << " " << endl;

  // ********************************
  // Todo: Add some logic or strategy to confirm init status, VIORBSLAM paper just uses 15 seconds to confirm
  bool bVIOInited = false;
  if (mdStartTime < 0) mdStartTime = pNewestKF->timestamp_;
  if (pNewestKF->timestamp_ - mdStartTime >= mdFinalTime) {  // 15s in the paper V-A
    cout << yellowSTR "condnum=" << w2.at<float>(0) << ";" << w2.at<float>(5) << whiteSTR << endl;
    //     if (w2.at<float>(0)/w2.at<float>(5)<700)
    bVIOInited = true;
  }

  // if VIO is initialized with appropriate bg*,s*,gw*,ba*, update the map(MPs' P,KFs' PRVB) like GBA/CorrrectLoop()
  if (bVIOInited) {
    // Set NavState , scale and bias for all KeyFrames
    double scale = s_;
    // gravity vector in world frame
    cv::Mat gw;
    {
      unique_lock<mutex> lock(mMutexInitIMU);
      mGravityVec = Rwi_ * GI;
      gw = mGravityVec.clone();
    }
    Vector3d gweig = Converter::toVector3d(gw);

    {  // Update the Map needs mutex lock: Stop local mapping, like RunGlobalBundleAdjustment() in LoopClosing.cc
      mpLocalMapper->RequestStop();  // same as CorrectLoop(), suspend/stop/freeze LocalMapping thread
      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped() &&
             !mpLocalMapper->isFinished()) {  // if LocalMapping is killed by System::Shutdown(), don't wait any more
        usleep(1000);
      }

      unique_lock<mutex> lockScale(
          mpMap->mMutexScaleUpdateLoopClosing);  // notice we cannot update scale during LoopClosing or LocalBA!
      unique_lock<mutex> lockScale2(mpMap->mMutexScaleUpdateGBA);
      unique_lock<mutex> lock(mpMap->mMutexMapUpdate, std::defer_lock);  // Get Map Mutex
      while (!lock.try_lock()) {
        if (GetReset()) {
          mpLocalMapper->Release();  // recover LocalMapping thread, same as CorrectLoop()
          return false;
        }
        usleep(3000);
      }
      // Update KFs' PRVB
      // update the vScaleGravityKF to the current size, and pNewestKF is mpCurrentKeyFrame during the LocalMapping
      // thread is stopped
      vScaleGravityKF = mpMap->GetAllKeyFrames();
      pNewestKF = GetCurrentKeyFrame();
      assert(pNewestKF == vScaleGravityKF.back());  // they must be same for we change the set less func. in Map.h
      // recover right scaled Twc&NavState from old unscaled Twc with scale
      for (vector<KeyFrame *>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end(); vit != vend;
           ++vit) {
        KeyFrame *pKF = *vit;
        if (pKF->isBad()) continue;
        // we can SetPose() first even no IMU data
        cv::Mat Tcw = pKF->GetPose(), Twc = pKF->GetPoseInverse();  // we must cache Twc first!
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3) * scale;            // right scaled pwc
        tcw.copyTo(Tcw.rowRange(0, 3).col(3));
        pKF->SetPose(Tcw);  // manually SetPose(right scaled Tcw)
        // Position and rotation of visual SLAM
        cv::Mat wPc = Twc.rowRange(0, 3).col(3);          // wPc/twc
        cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);  // Rwc
        // Set position and rotation of navstate
        cv::Mat wPb = scale * wPc + Rwc * pcb;  // right scaled pwb from right scaled pwc
        NavState ns;
        ns.mpwb = Converter::toVector3d(wPb);
        ns.setRwb(Converter::toMatrix3d(Rwc * Rcb));
        ns.mbg = bgest;
        ns.mba = bastareig;                    // bg* ba*
        ns.mdbg = ns.mdba = Vector3d::Zero();  // Set delta_bias to zero. (only updated during optimization)
        // Step 4. / See IV-D/(18)/(3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
        // compute velocity
        if (pKF != vScaleGravityKF.back()) {
          KeyFrame *pKFnext = pKF->GetNextKeyFrame();
          assert(pKFnext && "pKFnext is NULL");
          if (pKFnext->GetIMUPreInt().mdeltatij == 0) {
            cout << "time 0" << endl;
            continue;
          }
          pKF->SetNavStateOnly(ns);  // we must update the pKF->mbg&mba before pKFnext->PreIntegration()
          pKFnext->PreIntegration<IMUData>(
              pKF);  // it's originally based on bi_bar=0, but now it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
          const IMUPreintegrator imupreint =
              pKFnext->GetIMUPreInt();  // IMU pre-int between pKF ~ pKFnext, though the paper seems to use the
                                        // vKFInit[k].mOdomPreIntIMU so its dbgi=0 but its dbai=bai, we use more precise
                                        // bi_bar here
          double dt = imupreint.mdeltatij;                  // deltati_i+1
          cv::Mat dp = Converter::toCvMat(imupreint.mpij);  // deltapi_i+1
          // cv::Mat Japij=Converter::toCvMat(imupreint.mJapij);    			// Ja_deltap
          cv::Mat wPcnext = pKFnext->GetPoseInverse().rowRange(0, 3).col(3);          // wPci+1
          cv::Mat Rwcnext = pKFnext->GetPoseInverse().rowRange(0, 3).colRange(0, 3);  // Rwci+1
          cv::Mat vwbi = -1. / dt *
                         (scale * (wPc - wPcnext) + (Rwc - Rwcnext) * pcb + dt * dt / 2 * gw +
                          Rwc * Rcb * (dp));  //-1/dt*(pwbi-pwbj+1/2*gw*dt^2+Rwbi*(dp+Japij*dbai)), pwbi=s*pwc+Rwc*pcb,
                                              // s=sw=swRightScaled_wNow
          ns.mvwb = Converter::toVector3d(vwbi);
        } else {
          // If this is the last KeyFrame, no 'next' KeyFrame exists, use (3) in VOIRBSLAM paper with
          // ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
          if (pKF->GetIMUPreInt().mdeltatij == 0) {
            cout << "time 0" << endl;
            continue;
          }
          KeyFrame *pKFprev = pKF->GetPrevKeyFrame();
          assert(pKFprev && "pKFnext is NULL");
          const IMUPreintegrator imupreint =
              pKF->GetIMUPreInt();  // notice it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
          double dt = imupreint.mdeltatij;
          NavState nsprev = pKFprev->GetNavState();
          ns.mvwb =
              nsprev.mvwb + gweig * dt + nsprev.mRwb * (imupreint.mvij);  // vwbj=vwbi+gw*dt+Rwbi*(dvij+Javij*dbai)
        }
        pKF->SetNavStateOnly(ns);  // now ns also has the right mvwb
      }
      // Update MPs' Position
      vector<MapPoint *> vpMPs = mpMap->GetAllMapPoints();  // we don't change the vpMPs[i] but change the *vpMPs[i]
      for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; ++vit)
        (*vit)->UpdateScale(scale);
      // Now every thing in Map is right scaled & mGravityVec is got
      if (!mbUsePureVision) SetVINSInited(true);
      mpMap->InformNewChange();  // used to notice Tracking thread bMapUpdated

      mpLocalMapper->Release();  // recover LocalMapping thread, same as CorrectLoop()
      std::cout << std::endl << "... Map scale & NavState updated ..." << std::endl << std::endl;
      // Run global BA/full BA after inited, we use LoopClosing thread to do this job for safety!
      //       Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap,GetGravityVec(),15,NULL,0,false,true/false);SetInitGBAOver(true);
      SetInitGBA(true);
      SetInitGBA2(true);
    }
  }

  cout << yellowSTR "Used time in IMU Initialization="
       << chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count() << whiteSTR << endl;

  for (int i = 0; i < N; i++) {  // delete the newed pointer
    if (vKFInit[i]) delete vKFInit[i];
  }
  return bVIOInited;
}

}  // namespace VIEO_SLAM
