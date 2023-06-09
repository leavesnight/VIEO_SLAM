/**
 * This file is part of VIEO_SLAM
 */

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include "KannalaBrandt8.h"
#include "common/config.h"
#include "common/mlog/log.h"

namespace VIEO_SLAM {
// For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);
bool Frame::usedistort_ = false;

const cv::Mat &Frame::GetcvTcwCst() const { return Tcw_; }

void Frame::UpdatePoseFromNS() {
  cv::Mat Rbc = mTbc.rowRange(0, 3).colRange(0, 3);  // don't need clone();
  cv::Mat Pbc = mTbc.rowRange(0, 3).col(3);          // or tbc

  cv::Mat Rwb = Converter::toCvMat(mNavState.getRwb());
  cv::Mat Pwb = Converter::toCvMat(mNavState.mpwb);  // or twb
  // Tcw=Tcb*Twb, Twc=Twb*Tbc
  cv::Mat Rcw = (Rwb * Rbc).t();
  cv::Mat Pwc = Rwb * Pbc + Pwb;
  cv::Mat Pcw = -Rcw * Pwc;  // tcw=-Rwc.t()*twc=-Rcw*twc

  cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
  Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
  Pcw.copyTo(Tcw.rowRange(0, 3).col(3));

  SetPose(Tcw);  // notice w means B0/0th IMU Frame, c means ci/c(ti)/now camera Frame
}
void Frame::UpdateNavStatePVRFromTcw() {
  cv::Mat Twb = Converter::toCvMatInverse(mTbc * Tcw_);
  Eigen::Matrix3d Rwb = Converter::toMatrix3d(Twb.rowRange(0, 3).colRange(0, 3));
  Eigen::Vector3d Pwb = Converter::toVector3d(Twb.rowRange(0, 3).col(3));

  Eigen::Matrix3d Rw1 = mNavState.getRwb();  // Rwbj_old/Rwb1
  Eigen::Vector3d Vw1 = mNavState.mvwb;  // Vw1/wV1=wvbj-1bj_old now bj_old/b1 is changed to bj_new/b2, wV2=wvbj-1bj_new
  Eigen::Vector3d Vw2 = Rwb * Rw1.transpose() * Vw1;  // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

  mNavState.mpwb = Pwb;
  mNavState.setRwb(Rwb);
  mNavState.mvwb = Vw2;
}

template <>
void Frame::DeepMovePreintOdomFromLastKF(IMUPreintegrator &preint_odom) {
  preint_odom = *ppreint_imu_kf_;
  auto &lodom = ppreint_imu_kf_->GetRawDataRef();
  preint_odom.AppendFrontPreIntegrationList(lodom, lodom.begin(), lodom.end());
}
template <>
int Frame::PreIntegrationFromLastKF<IMUData>(FrameBase *plastkf, double tmi, double tmj_1,
                                             const typename aligned_list<IMUData>::const_iterator &iteri,
                                             const typename aligned_list<IMUData>::const_iterator &iterj, bool breset,
                                             int8_t verbose) {
  CV_Assert(ppreint_imu_kf_);
  NavState ns = plastkf->GetNavState();
  if (breset) CV_Assert(plastkf->ftimestamp_ == tmi);
  return FrameBase::PreIntegration<IMUData>(tmi, tmj_1, ns.mbg, ns.mba, iteri, iterj, breset, ppreint_imu_kf_, verbose);
}
// zzh

// please don't forget voc!! Or ComputeBoW() will have a segement fault problem
Frame::Frame(istream &is, ORBVocabulary *voc) : mpORBvocabulary(voc) {
  mnId = nNextId++;  // new Frame ID
  read(is);
  vvkeys_.resize(1);
  vvkeys_[0] = mvKeys;
  // N is got in read(), very important allocation! for LoadMap()
  mvpMapPoints.resize(N, static_cast<MapPoint *>(NULL));
}
bool Frame::read(istream &is, bool bOdomList) {
  if (!FrameBase::read(is)) return false;
  // TODO(zzh): for nonRGBD MAPREUSE
  //  vdescriptors_.resize(1);
  mDescriptors = cv::Mat::zeros(N, 32, CV_8UC1);  // 256bit binary descriptors
  readMat(is, mDescriptors);
  ComputeBoW();  // calculate mBowVec & mFeatVec, or we can do it by pKF
  is.read((char *)&mnScaleLevels, sizeof(mnScaleLevels));
  is.read((char *)&mfScaleFactor, sizeof(mfScaleFactor));
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors.resize(mnScaleLevels);
  mvLevelSigma2.resize(mnScaleLevels);
  mvInvLevelSigma2.resize(mnScaleLevels);
  mvScaleFactors[0] = mvLevelSigma2[0] = 1.0f;
  for (int i = 1; i < mnScaleLevels; i++) {
    mvScaleFactors[i] = mvScaleFactors[i - 1] * mfScaleFactor;
    mvLevelSigma2[i] = mvScaleFactors[i] * mvScaleFactors[i];  // at 0 level sigma=1 pixel
  }
  for (int i = 0; i < mnScaleLevels; i++) mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
  is.read((char *)&mnMinX, sizeof(mnMinX));
  is.read((char *)&mnMinY, sizeof(mnMinY));
  is.read((char *)&mnMaxX, sizeof(mnMaxX));
  is.read((char *)&mnMaxY, sizeof(mnMaxY));
  mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
  mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);
  // load mNavState
  // For VIO, we should compare the Pose of B/IMU Frame!!! not the Twc but the Twb! with EuRoC's Twb_truth(using
  // Tb_prism/Tbs from vicon0/data.csv) (notice vicon0 means the prism's Pose), and I found state_groundtruth_estimate0
  // is near Twb_truth but I don't think it's truth!
  double pdData[3];
  double pdData4[4];
  NavState ns;
  is.read((char *)pdData, sizeof(pdData));
  ns.mpwb << pdData[0], pdData[1], pdData[2];  // txyz
  is.read((char *)pdData4, sizeof(pdData4));
  ns.mRwb.setQuaternion(Eigen::Quaterniond(pdData4));  // qxyzw
  is.read((char *)pdData, sizeof(pdData));
  ns.mvwb << pdData[0], pdData[1], pdData[2];  // vxyz
  is.read((char *)pdData, sizeof(pdData));
  ns.mbg << pdData[0], pdData[1], pdData[2];  // bgxyz
  is.read((char *)pdData, sizeof(pdData));
  ns.mba << pdData[0], pdData[1], pdData[2];  // baxyz
  is.read((char *)pdData, sizeof(pdData));
  ns.mdbg << pdData[0], pdData[1], pdData[2];  // dbgxyz
  is.read((char *)pdData, sizeof(pdData));
  ns.mdba << pdData[0], pdData[1], pdData[2];  // dbaxyz
  mNavState = ns;
  UpdatePoseFromNS();
  // load vgrids_[i][j]
  AssignFeaturesToGrid();
  if (bOdomList) {
    double &tmEnc = mOdomPreIntEnc.mdeltatij;
    is.read((char *)&tmEnc, sizeof(tmEnc));
    if (tmEnc > 0) {
      readEigMat(is, mOdomPreIntEnc.mdelxEij);
      readEigMat(is, mOdomPreIntEnc.mSigmaEij);
    }
    double &tmIMU = mOdomPreIntIMU.mdeltatij;
    is.read((char *)&tmIMU, sizeof(tmIMU));
    if (tmIMU > 0) {  // for IMUPreIntegratorBase<IMUDataBase>
      readEigMat(is, mOdomPreIntIMU.mpij);
      readEigMat(is, mOdomPreIntIMU.mRij);
      readEigMat(is, mOdomPreIntIMU.mvij);  // PRV
      readEigMat(is, mOdomPreIntIMU.mSigmaijPRV);
      readEigMat(is, mOdomPreIntIMU.mSigmaij);
      readEigMat(is, mOdomPreIntIMU.mJgpij);
      readEigMat(is, mOdomPreIntIMU.mJapij);
      readEigMat(is, mOdomPreIntIMU.mJgvij);
      readEigMat(is, mOdomPreIntIMU.mJavij);
      readEigMat(is, mOdomPreIntIMU.mJgRij);
    }
  }
  return is.good();
}
bool Frame::write(ostream &os) const {
  if (!FrameBase::write(os)) return false;
  // we can get these from mnMaxX...
  //   os.write((char*)&mfGridElementWidthInv,sizeof(mfGridElementWidthInv));
  //   os.write((char*)&mfGridElementHeightInv,sizeof(mfGridElementHeightInv));
  // we can directly ComputeBoW() from mDescriptors
  //   mBowVec.write(os);
  //   mFeatVec.write(os);
  os.write((char *)&mnScaleLevels, sizeof(mnScaleLevels));
  os.write((char *)&mfScaleFactor, sizeof(mfScaleFactor));
  // we can get these from former 2 parameters mnScaleLevels & mfScaleFactor
  //   os.write((char*)&mfLogScaleFactor,sizeof(mfLogScaleFactor));
  //   os.write((char*)&mvScaleFactors,sizeof(mvScaleFactors));
  float fTmp[4] = {(float)mnMinX, (float)mnMinY, (float)mnMaxX, (float)mnMaxY};  // compatible with Frame
  os.write((char *)fTmp, sizeof(fTmp));
  // from fx~cy
  //   writeMat(os,mK);
  // save mvpMapPoints,{mpParent,mbNotErase(mspLoopEdges)} in LoadMap for convenience
  {  // save mNavState
    const double *pdData;
    Eigen::Quaterniond q = mNavState.mRwb.unit_quaternion();  // qwb from Rwb
    pdData = mNavState.mpwb.data();
    os.write((const char *)pdData, sizeof(*pdData) * 3);  // txyz
    pdData = q.coeffs().data();
    os.write((const char *)pdData, sizeof(*pdData) * 4);  // qxyzw
    pdData = mNavState.mvwb.data();
    os.write((const char *)pdData, sizeof(*pdData) * 3);  // vxyz
    pdData = mNavState.mbg.data();
    os.write((const char *)pdData, sizeof(*pdData) * 3);  // bgxyz_bar
    pdData = mNavState.mba.data();
    os.write((const char *)pdData, sizeof(*pdData) * 3);  // baxyz_bar
    pdData = mNavState.mdbg.data();
    os.write((const char *)pdData, sizeof(*pdData) * 3);  // dbgxyz
    pdData = mNavState.mdba.data();
    os.write((const char *)pdData, sizeof(*pdData) * 3);  // dbaxyz
  }
  // we can still get it from mvKeysUn
  //  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
  //    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) writeVec(os, vgrids_[i][j]);

  // save mOdomPreIntOdom, code starting from here is diffrent from KeyFrame::write()
  double tm = mOdomPreIntEnc.mdeltatij;
  os.write((char *)&tm, sizeof(tm));
  if (tm > 0) {
    writeEigMat(os, mOdomPreIntEnc.mdelxEij);
    writeEigMat(os, mOdomPreIntEnc.mSigmaEij);
  }
  tm = mOdomPreIntIMU.mdeltatij;
  os.write((char *)&tm, sizeof(tm));
  if (tm > 0) {  // for IMUPreIntegratorBase<IMUDataBase>
    writeEigMat(os, mOdomPreIntIMU.mpij);
    writeEigMat(os, mOdomPreIntIMU.mRij);
    writeEigMat(os, mOdomPreIntIMU.mvij);  // PRV
    writeEigMat(os, mOdomPreIntIMU.mSigmaijPRV);
    writeEigMat(os, mOdomPreIntIMU.mSigmaij);
    writeEigMat(os, mOdomPreIntIMU.mJgpij);
    writeEigMat(os, mOdomPreIntIMU.mJapij);
    writeEigMat(os, mOdomPreIntIMU.mJgvij);
    writeEigMat(os, mOdomPreIntIMU.mJavij);
    writeEigMat(os, mOdomPreIntIMU.mJgRij);
  }
  return os.good();
}

// created by zzh over.

long unsigned int Frame::nNextId = 0;
bool Frame::mbInitialComputations = true;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame() {}

// Copy Constructor
Frame::Frame(const Frame &frame, bool copy_shallow)
    : FrameBase(frame),  // mOdomPreIntIMU/Enc list uncopied
      ppreint_enc_kf_(frame.ppreint_enc_kf_),
      ppreint_imu_kf_(frame.ppreint_imu_kf_),
      mpORBvocabulary(frame.mpORBvocabulary),
      mpORBextractors(frame.mpORBextractors),
      num_mono(frame.num_mono),
      mvidxsMatches(frame.mvidxsMatches),
      goodmatches_(frame.goodmatches_),
      mapcamidx2idxs_(frame.mapcamidx2idxs_),
      mapidxs2n_(frame.mapidxs2n_),
      mv3Dpoints(frame.mv3Dpoints),
      vvkeys_(frame.vvkeys_),
      mapin2n_(frame.mapin2n_),
      mBowVec(frame.mBowVec),
      mFeatVec(frame.mFeatVec),
      mvbOutlier(frame.mvbOutlier),
      mpReferenceKF(frame.mpReferenceKF),
      mnScaleLevels(frame.mnScaleLevels),
      mfScaleFactor(frame.mfScaleFactor),
      mfLogScaleFactor(frame.mfLogScaleFactor),
      mvScaleFactors(frame.mvScaleFactors),
      mvInvScaleFactors(frame.mvInvScaleFactors),
      mvLevelSigma2(frame.mvLevelSigma2),
      mvInvLevelSigma2(frame.mvInvLevelSigma2) {
  if (copy_shallow) {
    mDescriptors = frame.mDescriptors;
    vdescriptors_ = frame.vdescriptors_;
  } else {
    mDescriptors = frame.mDescriptors.clone();
    vdescriptors_.resize(frame.vdescriptors_.size());
    for (int i = 0; i < vdescriptors_.size(); ++i) vdescriptors_[i] = frame.vdescriptors_[i].clone();
  }
  vgrids_ = frame.vgrids_;

  if (!frame.Tcw_.empty()) SetPose(frame.Tcw_);

  // created by zzh
  mMargCovInv = frame.mMargCovInv;
  mNavStatePrior = frame.mNavStatePrior;
  mbPrior = frame.mbPrior;
}

Frame::Frame(const vector<cv::Mat> &ims, const double &timeStamp, const vector<ORBextractor *> &extractors,
             ORBVocabulary *voc, const vector<GeometricCamera *> &CamInsts, const float &bf, const float &thDepth,
             IMUPreintegrator *ppreint_imu_kf, EncPreIntegrator *ppreint_enc_kf, bool usedistort,
             const float th_far_pts)
    : FrameBase(timeStamp),
      ppreint_enc_kf_(ppreint_enc_kf),
      ppreint_imu_kf_(ppreint_imu_kf),
      mpORBvocabulary(voc),
      mbPrior(false)  // zzh
{
  mpCameras = CamInsts;
  stereoinfo_.baseline_bf_[0] = bf / CamInsts[0]->toK()(0, 0);  // trans unit from pixel to metre
  stereoinfo_.baseline_bf_[1] = bf;
  mThDepth = thDepth;

  // Frame ID
  mnId = nNextId++;
  usedistort_ = usedistort;

  mpORBextractors = extractors;
  // Scale Level Info
  mnScaleLevels = mpORBextractors[0]->GetLevels();
  mfScaleFactor = mpORBextractors[0]->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractors[0]->GetScaleFactors();
  mvInvScaleFactors = mpORBextractors[0]->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractors[0]->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractors[0]->GetInverseScaleSigmaSquares();

  // ORB extraction
  size_t sz_extract = extractors.size();
  size_t sz_ims = ims.size();
  assert(sz_ims >= sz_extract);  // > means RGB-D
  vector<thread> threads(sz_extract);
  vdescriptors_.resize(sz_extract);
  vvkeys_.resize(sz_extract);
  num_mono.resize(sz_extract);
#ifdef TIMER_FLOW
  mlog::Timer timer_tmp;
#endif
  assert(sz_extract == CamInsts.size() || (sz_extract > CamInsts.size() && !usedistort && !CamInsts.empty()));
  for (int i = 0; i < sz_extract; ++i) {
    vector<int> *plapping_area = nullptr;
    int icam = CamInsts.size() > i ? i : 0;
    if (CamInsts[icam]->CAM_FISHEYE == CamInsts[icam]->GetType()) {
      plapping_area = &static_cast<KannalaBrandt8 *>(CamInsts[icam])->mvLappingArea;
    }
    threads[i] = thread(&Frame::ExtractORB, this, i, ims[i], plapping_area);
  }
  for (int i = 0; i < sz_extract; ++i) {
    threads[i].join();
  }
#ifdef TIMER_FLOW
  timer_tmp.GetDTfromInit(1, "tracking_thread_debug.txt", "tm cost extractfeats=");
#endif

  if (!usedistort || 1 == sz_extract) {
    mvKeys = vvkeys_[0];
    mDescriptors = vdescriptors_[0].clone();
    N = mvKeys.size();
    if (!N) return;

    if (!usedistort_) UndistortKeyPoints();

    if (1 < sz_extract)
      ComputeStereoMatches();
    else if (1 < sz_ims)
      ComputeStereoFromRGBD(ims[1]);
    else {
      // Set no stereo information for Mono Frame Constructor
      stereoinfo_.vdepth_ = vector<float>(N, -1);
      stereoinfo_.vuright_ = vector<float>(N, -1);
    }
  } else {
    ComputeStereoFishEyeMatches(th_far_pts);
    if (!usedistort_) UndistortKeyPoints();
  }
#ifdef TIMER_FLOW
  timer_tmp.GetDTfromLast(2, "tracking_thread_debug.txt", "tm stereomatch=");
#endif

  // for directly associated vector type mvpMapPoints,used in KF::AddMapPiont()
  mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(nullptr));
  mvbOutlier = vector<bool>(N, false);

  // This is done only for the first Frame (or after a change in the calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(ims[0]);
    // divide the img into ORB2SLAM like 64*48(rows) grids for features matching!
    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    mbInitialComputations = false;
  }

  AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid() {
  assert(!mpCameras.empty());
  size_t n_cams = vvkeys_.size();
  vgrids_.resize(n_cams, vector<vector<vector<size_t>>>(FRAME_GRID_COLS, vector<vector<size_t>>(FRAME_GRID_ROWS)));
  int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS * n_cams);
  for (size_t cami = 0; cami < n_cams; ++cami) {
    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
      for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) vgrids_[cami][i][j].reserve(nReserve);

    for (int i = 0; i < vvkeys_[cami].size(); i++) {
      const cv::KeyPoint &kp = !usedistort_ ? mvKeysUn[i] : vvkeys_[cami][i];

      int nGridPosX, nGridPosY;
      size_t mapi = !mapin2n_.size() ? i : mapin2n_[cami][i];
      if (PosInGrid(kp, nGridPosX, nGridPosY)) {
        vgrids_[cami][nGridPosX][nGridPosY].push_back(mapi);
      }
    }
  }
}

void Frame::ExtractORB(int flag, const cv::Mat &im, std::vector<int> *pvLappingArea) {
  num_mono[flag] = (*mpORBextractors[flag])(im, cv::Mat(), vvkeys_[flag], vdescriptors_[flag], pvLappingArea);
}

void Frame::SetPose(cv::Mat Tcw) {
  Tcw_ = Tcw.clone();
  UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices() {
  mRcw = Tcw_.rowRange(0, 3).colRange(0, 3);
  mRwc = mRcw.t();
  mtcw = Tcw_.rowRange(0, 3).col(3);
  mOw = -mRcw.t() * mtcw;
}

// TODO:extend this to 4 cams
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
  bool bret = false;
  size_t n_cams = !mpCameras.size() ? 1 : mpCameras.size();

  auto &trackinfo = pMP->GetTrackInfoRef();
  trackinfo.Reset();

  // 3D in absolute coordinates
  cv::Mat wP = pMP->GetWorldPos();
  cv::Mat Pn = pMP->GetNormal();
  const float maxDistance = pMP->GetMaxDistanceInvariance();
  const float minDistance = pMP->GetMinDistanceInvariance();
  cv::Mat Rcrw = Tcw_.rowRange(0, 3).colRange(0, 3);
  // 3D in camera coordinates
  const cv::Mat Pcr = mRcw * wP + mtcw;
  float sum_depth = 0;
  for (size_t cami = 0; cami < n_cams; ++cami) {
    // TODO: unify this cycle with SBP to one func
    Vector3d Pc = Converter::toVector3d(Pcr);
    GeometricCamera *pcam1 = nullptr;
    cv::Mat twc = mOw.clone();  // wO
    if (mpCameras.size() > cami) {
      pcam1 = mpCameras[cami];
      Pc = pcam1->GetTcr() * Pc;
      twc += Rcrw.t() * pcam1->Getcvtrc();
    }
    const float &PcZ = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f) continue;

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    float u, v;
    assert(pcam1);
    if (!usedistort_) {
      const float &PcX = Pc(0);
      const float &PcY = Pc(1);
      Vector3f p_normalize = Vector3f(PcX * invz, PcY * invz, 1);
      Vector3f uv = pcam1->toK().cast<float>() * p_normalize;  // K*Xc
      u = uv[0];
      v = uv[1];
    } else {
      auto pt = pcam1->project(Pc);
      u = pt[0];
      v = pt[1];
    }

    if (u < mnMinX || u > mnMaxX) continue;
    if (v < mnMinY || v > mnMaxY) continue;

    // Check viewing angle
    const cv::Mat PO = wP - twc;
    const float dist3D = cv::norm(PO);
    // Check distance is in the scale invariance region of the MapPoint
    // if it's out of the frustum, image pyramid is not effective
    if (dist3D < minDistance || dist3D > maxDistance) continue;
    // use nbar(P) instead of n(Frame) because the SBP() use the best descriptor of MapPoint
    const float viewCos = PO.dot(Pn) / dist3D;
    // if viewing angle(vec OP && vec nbar(P)) > arccos(viewingCosLimit)(here is 60 degrees), it's not in Frustum
    if (viewCos < viewingCosLimit) continue;
    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist3D, this);

    // Data used by the tracking

    trackinfo.vtrack_proj_[0].push_back(u);
    trackinfo.vtrack_proj_[1].push_back(v);
    trackinfo.vtrack_proj_[2].push_back(u - stereoinfo_.baseline_bf_[1] * invz);  // ur=ul-b*fx/dl
    trackinfo.vtrack_scalelevel_.push_back(nPredictedLevel);
    trackinfo.vtrack_viewcos_.push_back(viewCos);
    trackinfo.vtrack_cami_.push_back(cami);
    sum_depth += dist3D;
    bret = true;
  }
  if (bret) trackinfo.track_depth_ = sum_depth / trackinfo.vtrack_cami_.size();
  trackinfo.btrack_inview_ = bret;

  return bret;
}

vector<size_t> Frame::GetFeaturesInArea(size_t cami, const float &x, const float &y, const float &r, const int minLevel,
                                        const int maxLevel) const {
  vector<size_t> vIndices;
  if (vgrids_.empty()) return vIndices;
  vIndices.reserve(N);

  const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
  if (nMinCellX >= FRAME_GRID_COLS) return vIndices;

  const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
  if (nMaxCellX < 0) return vIndices;

  const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
  if (nMinCellY >= FRAME_GRID_ROWS) return vIndices;

  const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
  if (nMaxCellY < 0) return vIndices;

  const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);  // minLevel==0&&maxLevel<0 don't need to judge

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const vector<size_t> vCell = vgrids_[cami][ix][iy];
      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &kpUn = (!mpCameras.size() || !usedistort_) ? mvKeysUn[vCell[j]] : mvKeys[vCell[j]];
        if (bCheckLevels)  // if the octave is out of level range
        {
          if (kpUn.octave < minLevel)  //-1 is also ok,0 cannot be true
            continue;
          if (maxLevel >= 0)  // avoid for -1
            if (kpUn.octave > maxLevel) continue;
        }

        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        if (fabs(distx) < r && fabs(disty) < r)  // find the features in a rectangle window whose centre is (x,y)
          vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
  posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
  posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS) return false;

  return true;
}

void Frame::ComputeBoW() {
  if (mBowVec.empty()) {
    vector<cv::Mat> vCurrentDesc =
        Converter::toDescriptorVector(mDescriptors);  // transform Mat(N*32*8U) to vec<Mat>(N*1*32*8U)
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec,
                               4);  // compute mBowVec && mFeatVec(at level (d-levelsup)6-4) of this Frame
  }
}

void Frame::UndistortKeyPoints() {
  assert(!mpCameras.empty());
  if (mpCameras[0]->GetType() == mpCameras[0]->CAM_PINHOLE) {
    mvKeysUn = mvKeys;
    return;
  }

  // Fill matrix with points
  cv::Mat mat(N, 2, CV_32F);
  for (int i = 0; i < N; ++i) {
    size_t cami = mapn2in_.size() <= i ? 0 : get<0>(mapn2in_[i]);
    cv::Mat mattmp = mpCameras[cami]->toKcv() * mpCameras[cami]->unprojectMat(mvKeys[i].pt);
    mat.at<float>(i, 0) = mattmp.at<float>(0);
    mat.at<float>(i, 1) = mattmp.at<float>(1);
  }

  // Fill undistorted keypoint vector
  mvKeysUn.resize(N);
  for (int i = 0; i < N; i++) {
    cv::KeyPoint kp = mvKeys[i];
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    mvKeysUn[i] = kp;
  }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
  assert(!mpCameras.empty());
  // if not usedistort_ we should calc undistorted 4 corner pts(when it's pinhole, they're the same)
  if (mpCameras[0]->GetType() != mpCameras[0]->CAM_PINHOLE && !usedistort_) {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;
    mat.at<float>(1, 0) = imLeft.cols;
    mat.at<float>(1, 1) = 0.0;
    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imLeft.rows;
    mat.at<float>(3, 0) = imLeft.cols;
    mat.at<float>(3, 1) = imLeft.rows;

    // Undistort corners
    // TODO: limit different cam model's border
    size_t cami = 0;
    for (int i = 0; i < 4; ++i) {
      cv::Mat mattmp = mpCameras[cami]->toKcv() *
                       mpCameras[cami]->unprojectMat(cv::Point2f(mat.at<float>(i, 0), mat.at<float>(i, 1)));
      mat.at<float>(i, 0) = mattmp.at<float>(0);
      mat.at<float>(i, 1) = mattmp.at<float>(1);
    }

    mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  } else {
    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
  }
}

void Frame::ComputeStereoMatches() {
  stereoinfo_.vuright_ = vector<float>(N, -1.0f);
  stereoinfo_.vdepth_ = vector<float>(N, -1.0f);

  const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

  const int nRows = mpORBextractors[0]->mvImagePyramid[0].rows;

  // Assign keypoints to row table
  vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

  for (int i = 0; i < nRows; i++) vRowIndices[i].reserve(200);

  const int Nr = vvkeys_[1].size();

  for (int iR = 0; iR < Nr; iR++) {
    const cv::KeyPoint &kp = vvkeys_[1][iR];
    const float &kpY = kp.pt.y;
    const float r = 2.0f * mvScaleFactors[vvkeys_[1][iR].octave];
    const int maxr = ceil(kpY + r);
    const int minr = floor(kpY - r);

    for (int yi = minr; yi <= maxr; yi++) vRowIndices[yi].push_back(iR);
  }

  // Set limits for search
  const float minZ = stereoinfo_.baseline_bf_[0];
  const float minD = 0;
  const float maxD = stereoinfo_.baseline_bf_[1] / minZ;

  // For each left keypoint search a match in the right image
  vector<pair<int, int>> vDistIdx;
  vDistIdx.reserve(N);

  for (int iL = 0; iL < N; iL++) {
    const cv::KeyPoint &kpL = vvkeys_[0][iL];
    const int &levelL = kpL.octave;
    const float &vL = kpL.pt.y;
    const float &uL = kpL.pt.x;

    const vector<size_t> &vCandidates = vRowIndices[vL];

    if (vCandidates.empty()) continue;

    const float minU = uL - maxD;
    const float maxU = uL - minD;

    if (maxU < 0) continue;

    int bestDist = ORBmatcher::TH_HIGH;
    size_t bestIdxR = 0;

    const cv::Mat &dL = mDescriptors.row(iL);

    // Compare descriptor to right keypoints
    for (size_t iC = 0; iC < vCandidates.size(); iC++) {
      const size_t iR = vCandidates[iC];
      const cv::KeyPoint &kpR = vvkeys_[1][iR];

      if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1) continue;

      const float &uR = kpR.pt.x;

      if (uR >= minU && uR <= maxU) {
        const cv::Mat &dR = vdescriptors_[1].row(iR);
        const int dist = ORBmatcher::DescriptorDistance(dL, dR);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdxR = iR;
        }
      }
    }

    // Subpixel match by correlation
    if (bestDist < thOrbDist) {
      // coordinates in image pyramid at keypoint scale
      const float uR0 = vvkeys_[1][bestIdxR].pt.x;
      const float scaleFactor = mvInvScaleFactors[kpL.octave];
      const float scaleduL = round(kpL.pt.x * scaleFactor);
      const float scaledvL = round(kpL.pt.y * scaleFactor);
      const float scaleduR0 = round(uR0 * scaleFactor);

      // sliding window search
      const int w = 5;
      cv::Mat IL = mpORBextractors[0]
                       ->mvImagePyramid[kpL.octave]
                       .rowRange(scaledvL - w, scaledvL + w + 1)
                       .colRange(scaleduL - w, scaleduL + w + 1);
      IL.convertTo(IL, CV_32F);
      IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

      int bestDist = INT_MAX;
      int bestincR = 0;
      const int L = 5;
      vector<float> vDists;
      vDists.resize(2 * L + 1);

      const float iniu = scaleduR0 + L - w;
      const float endu = scaleduR0 + L + w + 1;
      if (iniu < 0 || endu >= mpORBextractors[1]->mvImagePyramid[kpL.octave].cols) continue;

      for (int incR = -L; incR <= +L; incR++) {
        cv::Mat IR = mpORBextractors[1]
                         ->mvImagePyramid[kpL.octave]
                         .rowRange(scaledvL - w, scaledvL + w + 1)
                         .colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
        IR.convertTo(IR, CV_32F);
        IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

        float dist = cv::norm(IL, IR, cv::NORM_L1);
        if (dist < bestDist) {
          bestDist = dist;
          bestincR = incR;
        }

        vDists[L + incR] = dist;
      }

      if (bestincR == -L || bestincR == L) continue;

      // Sub-pixel match (Parabola fitting)
      const float dist1 = vDists[L + bestincR - 1];
      const float dist2 = vDists[L + bestincR];
      const float dist3 = vDists[L + bestincR + 1];

      // calc the polynomial curve(power2)/parabolic curve y=ax^2+bx+c's minimum y value's x point
      const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

      if (deltaR < -1 || deltaR > 1) continue;

      // Re-scaled coordinate
      float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

      float disparity = (uL - bestuR);

      if (disparity >= minD && disparity < maxD) {
        if (disparity <= 0) {
          disparity = 0.01;
          bestuR = uL - 0.01;
        }
        stereoinfo_.vdepth_[iL] = stereoinfo_.baseline_bf_[1] / disparity;
        stereoinfo_.vuright_[iL] = bestuR;
        vDistIdx.push_back(pair<int, int>(bestDist, iL));
      }
    }
  }

  sort(vDistIdx.begin(), vDistIdx.end());
  const float median = vDistIdx[vDistIdx.size() / 2].first;
  const float thDist = 1.5f * 1.4f * median;

  for (int i = vDistIdx.size() - 1; i >= 0; i--) {
    if (vDistIdx[i].first < thDist)
      break;
    else {
      stereoinfo_.vuright_[vDistIdx[i].second] = -1;
      stereoinfo_.vdepth_[vDistIdx[i].second] = -1;
    }
  }
}

void Frame::ComputeStereoFishEyeMatches(const float th_far_pts) {
  // Speed it up by matching keypoints in the lapping area
  size_t n_cams = vvkeys_.size();

  // Perform a brute force between Keypoint in the all images
  vector<vector<vector<cv::DMatch>>> allmatches;
  for (int i = 0; i < n_cams; ++i) assert(-1 != num_mono[i]);
  for (int i = 0; i < n_cams - 1; ++i) {
    for (int j = i + 1; j < n_cams; ++j) {
      allmatches.push_back(vector<vector<cv::DMatch>>());
      if (num_mono[i] >= vdescriptors_[i].rows || num_mono[j] >= vdescriptors_[j].rows) continue;
      assert(!vdescriptors_[i].empty() && !vdescriptors_[j].empty());
      BFmatcher.knnMatch(vdescriptors_[i].rowRange(num_mono[i], vdescriptors_[i].rows),
                         vdescriptors_[j].rowRange(num_mono[j], vdescriptors_[j].rows), allmatches.back(), 2);
    }
  }

  int nMatches = 0;
  int descMatches = 0;
  // for theta << 1 here, approximately dmax=b/sqrt(2*(1-thresh_cos))
  assert(!mpCameras.empty());
  Eigen::Matrix3d K = mpCameras[0]->toK();
  float f_bar = (K(0, 0) + K(1, 1)) / 2.;
  double thresh_cosdisparity[2] = {0.9998, 1. - 1e-6};
  if (th_far_pts > 0) {
    for (int i = 0; i < 2; ++i)
      thresh_cosdisparity[i] =
          min(1. - pow(stereoinfo_.baseline_bf_[1] / f_bar / th_far_pts, 2) / 2., thresh_cosdisparity[i]);
  }

  // Check matches using Lowe's ratio
  assert(!goodmatches_.size() && !mapcamidx2idxs_.size() && !mvidxsMatches.size());
#ifdef USE_STRATEGY_MIN_DIST
  vector<vector<double>> lastdists;
#endif
  int num_thresh_try = thresh_cosdisparity[1] == thresh_cosdisparity[0] ? 1 : 2;
  for (int k = 0; k < num_thresh_try; ++k) {
    mvidxsMatches.clear();
    goodmatches_.clear();
    mapcamidx2idxs_.clear();
    mv3Dpoints.clear();
    lastdists.clear();
    descMatches = 0;
    for (size_t i = 0, idmatches = 0; i < n_cams - 1; ++i) {
      for (size_t j = i + 1; j < n_cams; ++j, ++idmatches) {
        auto &matches = allmatches[idmatches];
        for (vector<vector<cv::DMatch>>::iterator it = matches.begin(); it != matches.end(); ++it) {
          const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;
          if ((*it).size() >= 2 && ((*it)[0].distance < (*it)[1].distance * 0.7 ||
                                    ((*it)[0].distance < thOrbDist && (*it)[0].distance < (*it)[1].distance * 0.9))) {
            size_t idxi = (*it)[0].queryIdx + num_mono[i], idxj = (*it)[0].trainIdx + num_mono[j];
            vector<float> sigmas = {mvLevelSigma2[vvkeys_[i][idxi].octave], mvLevelSigma2[vvkeys_[j][idxj].octave]};
            aligned_vector<Eigen::Vector2d> kpts = {Eigen::Vector2d(vvkeys_[i][idxi].pt.x, vvkeys_[i][idxi].pt.y),
                                                    Eigen::Vector2d(vvkeys_[j][idxj].pt.x, vvkeys_[j][idxj].pt.y)};
            if (mpCameras[i]->FillMatchesFromPair(vector<GeometricCamera *>(1, mpCameras[j]), n_cams,
                                                  vector<pair<size_t, size_t>>{make_pair(i, idxi), make_pair(j, idxj)},
                                                  (*it)[0].distance, mvidxsMatches, goodmatches_, mapcamidx2idxs_,
                                                  thresh_cosdisparity[0], &mv3Dpoints, &kpts, &sigmas
#ifdef USE_STRATEGY_MIN_DIST
                                                  ,
                                                  &lastdists
#else
                                                  ,
                                                  nullptr
#endif
                                                  ,
                                                  &descMatches))
              ++nMatches;
          }
        }
      }
    }
    if (nMatches >= 30)
      break;
    else
      thresh_cosdisparity[0] = thresh_cosdisparity[1];
  }
#ifdef USE_STRATEGY_MIN_DIST
  for (size_t i = 0; i < mvidxsMatches.size(); ++i) {
    size_t count_num = 0;
    for (size_t itmp = 0; itmp < n_cams; ++itmp) {
      if (-1 != mvidxsMatches[i][itmp]) ++count_num;
    }
    if (count_num < 2) goodmatches_[i] = false;
  }
#endif
  if (n_cams > 2) {
    nMatches = 0;
    for (size_t i = 0; i < mvidxsMatches.size(); ++i) {
      if (!goodmatches_[i]) continue;
      // For every good match, check parallax and reprojection error to discard spurious matches
      cv::Mat p3D;
      auto &idx = mvidxsMatches[i];
      vector<float> sigmas;
      vector<size_t> vidx_used;
      vector<GeometricCamera *> pcams_in;
      aligned_vector<Eigen::Vector2d> kpts;
      for (int k = 0; k < idx.size(); ++k) {
        if (-1 != idx[k]) {
          vidx_used.push_back(idx[k]);
          sigmas.push_back(mvLevelSigma2[vvkeys_[k][idx[k]].octave]);
          pcams_in.push_back(mpCameras[k]);
          kpts.emplace_back(vvkeys_[k][idx[k]].pt.x, vvkeys_[k][idx[k]].pt.y);
        }
      }
      auto depths = pcams_in[0]->TriangulateMatches(pcams_in, kpts, sigmas, &p3D, thresh_cosdisparity[0]);
      bool bgoodmatch = depths.empty() ? false : true;
      for (auto d : depths) {
        if (d <= 0.0001f) {
          bgoodmatch = false;
          break;
        }
      }
      if (bgoodmatch) {
        mv3Dpoints[i] = Converter::toVector3d(p3D.clone());  // here should return pt in cami's ref frame, usually 0
        nMatches++;
      } else
        goodmatches_[i] = false;
    }
  }
  PRINT_DEBUG_FILE("match num=" << nMatches << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");

  assert(!stereoinfo_.vdepth_.size() && !stereoinfo_.vuright_.size());
  size_t num_pt_added = 0;
  for (size_t i = 0; i < n_cams; ++i) {
    if (vdescriptors_.size() <= i || vdescriptors_[i].empty()) continue;
    if (mDescriptors.empty()) {
      mDescriptors = vdescriptors_[i].clone();
    } else
      cv::vconcat(mDescriptors, vdescriptors_[i], mDescriptors);
    for (size_t k = 0; k < vvkeys_[i].size(); ++k) {
      auto camidx = make_pair(i, k);
      auto iteridxs = mapcamidx2idxs_.find(camidx);
      if (iteridxs != mapcamidx2idxs_.end()) {
        auto &ididxs = iteridxs->second;
        if (-1 != ididxs && goodmatches_[ididxs]) {
          Vector3d x3Dc = ((KannalaBrandt8 *)mpCameras[i])->GetTcr() * mv3Dpoints[ididxs];
          stereoinfo_.vdepth_.push_back(x3Dc[2]);
        } else
          stereoinfo_.vdepth_.push_back(-1);
      } else {
        stereoinfo_.vdepth_.push_back(-1);
      }
      stereoinfo_.vuright_.push_back(-1);
      mvKeys.push_back(vvkeys_[i][k]);
      mapn2in_.push_back(camidx);
      ++num_pt_added;
    }
  }

  if (mapin2n_.size() < n_cams) mapin2n_.resize(n_cams);
  mapidxs2n_.resize(mv3Dpoints.size(), -1);
  for (size_t k = 0; k < mvKeys.size(); ++k) {
    size_t cami = get<0>(mapn2in_[k]);
    assert(mapin2n_.size() > cami);
    if (mapin2n_[cami].size() < vvkeys_[cami].size()) mapin2n_[cami].resize(vvkeys_[cami].size());
    assert(mapin2n_[cami].size() > get<1>(mapn2in_[k]));
    mapin2n_[cami][get<1>(mapn2in_[k])] = k;
    auto iteridxs = mapcamidx2idxs_.find(mapn2in_[k]);
    if (iteridxs != mapcamidx2idxs_.end()) mapidxs2n_[iteridxs->second] = k;
  }
  N = num_pt_added;
  assert(N == mvKeys.size());
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth) {
  stereoinfo_.vuright_ = vector<float>(N, -1);
  stereoinfo_.vdepth_ = vector<float>(N, -1);

  for (int i = 0; i < N; i++) {
    const cv::KeyPoint &kp = mvKeys[i];
    const cv::KeyPoint &kpU = mvKeysUn[i];

    const float &v = kp.pt.y;
    const float &u = kp.pt.x;

    const float d = imDepth.at<float>(v, u);

    if (d > 0) {
      stereoinfo_.vdepth_[i] = d;
      // here maybe <0 and >=mnMaxX, suppose [mnMinX,mnMaxX), is there some problem?
      stereoinfo_.vuright_[i] = kpU.pt.x - stereoinfo_.baseline_bf_[1] / d;
    }
  }
}

size_t Frame::GetMapn2idxs(size_t i) {
  if (mapn2in_.size() <= i) return -1;  // for no mpCameras mode sz is 0
  auto iteridx = mapcamidx2idxs_.find(mapn2in_[i]);
  if (iteridx == mapcamidx2idxs_.end())
    return -1;
  else
    return iteridx->second;
}

cv::Mat Frame::UnprojectStereo(const int &i) {
  if (mapn2in_.size() > i) {
    const float z = stereoinfo_.vdepth_[i];
    if (z > 0) {
      auto ididxs = GetMapn2idxs(i);
      CV_Assert(-1 != ididxs && goodmatches_[ididxs]);
      Vector3d x3Dw = Converter::toMatrix3d(mRwc) * (GetTcr() * mv3Dpoints[ididxs]) + Converter::toVector3d(mOw);
      return Converter::toCvMat(x3Dw);
    } else {
      return cv::Mat();
    }
  }

  const float z = stereoinfo_.vdepth_[i];
  if (z > 0) {
    const float u = mvKeysUn[i].pt.x;
    const float v = mvKeysUn[i].pt.y;
    Vector3f uv_normal = mpCameras[0]->toK().cast<float>().inverse() * Vector3f(u, v, 1);
    const float x = uv_normal[0] * z;
    const float y = uv_normal[1] * z;
    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
    return mRwc * x3Dc + mOw;
  } else
    return cv::Mat();
}

}  // namespace VIEO_SLAM
