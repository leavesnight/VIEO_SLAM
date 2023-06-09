//
// Created by leavesnight on 2021/12/20.
//

#include "FrameBase.h"
#include "MapPoint.h"
#include "KannalaBrandt8.h"
#include "radtan.h"

using namespace VIEO_SLAM;
using std::pair;
using std::set;

cv::Mat FrameBase::mTbc, FrameBase::mTce;
Eigen::Matrix3d FrameBase::meigRcb;
Eigen::Vector3d FrameBase::meigtcb;

const Sophus::SE3d FrameBase::GetTwc() { return GetTcw().inverse(); }
const Sophus::SE3d FrameBase::GetTcw() { return GetTcwCst(); }

void FrameBase::AddMapPoint(MapPoint *pMP, const size_t &idx) {
  assert(mvpMapPoints.size() > idx);
  //  if (mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad()) {
  //    cout << "check mpid=" << mvpMapPoints[idx]->mnId << " ";
  //    auto vobs = mvpMapPoints[idx]->GetObservations();
  //    for (auto obs : vobs) {
  //      cout << obs.first->mnId << ":";
  //      for (auto idx : obs.second) cout << idx << " ";
  //      cout << endl;
  //    }
  //    cout << endl;
  //  }
  assert(!mvpMapPoints[idx] || mvpMapPoints[idx]->isBad() || mvpMapPoints[idx]->Observations() < 1);
  mvpMapPoints[idx] = pMP;
}

std::set<std::pair<MapPoint *, size_t>> FrameBase::GetMapPointsCami() {
  set<pair<MapPoint *, size_t>> s;
  for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
    if (!mvpMapPoints[i]) continue;
    MapPoint *pMP = mvpMapPoints[i];
    if (!pMP->isBad()) {
      size_t cami = mapn2in_.size() <= i ? 0 : get<0>(mapn2in_[i]);
      s.insert(make_pair(pMP, cami));
    }
  }
  return s;
}

template <>
void FrameBase::ClearOdomPreInt<IMUData>() {
  mOdomPreIntIMU.mdeltatij = 0;
}
template <>
int FrameBase::PreIntegration<IMUData, IMUPreintegrator>(IMUData::TTtime tm_start, IMUData::TTtime tm_end,
                                                         const Eigen::Vector3d &bgi_bar, const Eigen::Vector3d &bai_bar,
                                                         const typename aligned_list<IMUData>::const_iterator &iteri,
                                                         const typename aligned_list<IMUData>::const_iterator &iterj,
                                                         bool breset, IMUPreintegrator *ppreint_odom, int8_t verbose) {
  if (!ppreint_odom) ppreint_odom = &mOdomPreIntIMU;
#ifndef TRACK_WITH_IMU
  // TODO: fix this
  return ppreint_odom->PreIntegration(tm_start, tm_end, bgi_bar, bai_bar, iteri, iterj, breset);
#else
  return ppreint_odom->PreIntegration(tm_start, tm_end, bgi_bar, bai_bar, iteri, iterj, breset);
#endif
}
template <>
void FrameBase::PreIntegration<IMUData>(FrameBase *plastfb, const typename aligned_list<IMUData>::const_iterator &iteri,
                                        const typename aligned_list<IMUData>::const_iterator &iterj, bool breset,
                                        int8_t verbose) {
  NavState ns = plastfb->GetNavState();
  PreIntegration<IMUData, IMUPreintegrator>(plastfb->ftimestamp_, ftimestamp_, ns.mbg, ns.mba, iteri, iterj, breset,
                                            nullptr, verbose);
}

bool FrameBase::read(istream &is) {
  // we don't save old ID for it's useless in LoadMap()
  is.read((char *)&timestamp_, sizeof(timestamp_));
  ftimestamp_ = TS2S(timestamp_);

  uint8_t sz_cams;
  is.read((char *)&sz_cams, sizeof(sz_cams));
  mpCameras.resize(sz_cams);
  for (auto i = 0; i < sz_cams; ++i) {
    int cam_type;
    is.read((char *)&cam_type, sizeof(cam_type));
    uint8_t sz_params_tmp;
    is.read((char *)&sz_params_tmp, sizeof(sz_params_tmp));
    vector<float> params_tmp(sz_params_tmp);
    readVec(is, params_tmp);
    switch (cam_type) {
      case GeometricCamera::CAM_PINHOLE:
        mpCameras[i] = new Pinhole();
        break;
      case GeometricCamera::CAM_RADTAN:
        mpCameras[i] = new Radtan();
        break;
      case GeometricCamera::CAM_FISHEYE:
        mpCameras[i] = new KannalaBrandt8();
        break;
      default:
        PRINT_ERR_MUTEX("Unsupported Camera Model in " << __FUNCTION__ << endl);
        exit(-1);
    }
    mpCameras[i]->setParameters(params_tmp);
  }
  // Notice K.copyTo(mK) here mK will be allocated for it does not have a proper size or type before the operation

  is.read((char *)&stereoinfo_.baseline_bf_, sizeof(stereoinfo_.baseline_bf_));
  is.read((char *)&mThDepth, sizeof(mThDepth));
  is.read((char *)&N, sizeof(N));
  mvKeys.resize(N);
  mvKeysUn.resize(N);
  KeyFrame::readVec(is, mvKeys);
  KeyFrame::readVec(is, mvKeysUn);
  stereoinfo_.vdepth_.resize(N);
  stereoinfo_.vuright_.resize(N);
  KeyFrame::readVec(is, stereoinfo_.vdepth_);
  KeyFrame::readVec(is, stereoinfo_.vuright_);

  return is.good();
}
bool FrameBase::write(ostream &os) const {
  // we don't save old ID for it's useless in LoadMap(), but we'll save old KF ID/mnId in SaveMap()
  os.write((char *)&timestamp_, sizeof(timestamp_));

  uint8_t sz_cams = (uint8_t)mpCameras.size();
  os.write((char *)&sz_cams, sizeof(sz_cams));
  for (auto i = 0; i < sz_cams; ++i) {
    int cam_type = (int)mpCameras[i]->GetType();
    os.write((char *)&cam_type, sizeof(cam_type));
    vector<float> params_tmp = mpCameras[i]->getParameters();
    uint8_t sz_params_tmp = (uint8_t)params_tmp.size();
    os.write((char *)&sz_params_tmp, sizeof(sz_params_tmp));
    writeVec(os, params_tmp);
  }

  os.write((char *)&stereoinfo_.baseline_bf_, sizeof(stereoinfo_.baseline_bf_));
  os.write((char *)&mThDepth, sizeof(mThDepth));
  // mb=mbf/fx
  //   os.write((char*)&mb,sizeof(mb));
  os.write((char *)&N, sizeof(N));
  writeVec(os, mvKeys);
  writeVec(os, mvKeysUn);
  writeVec(os, stereoinfo_.vdepth_);
  writeVec(os, stereoinfo_.vuright_);
  writeMat(os, mDescriptors);

  return os.good();
}
