//
// Created by leavesnight on 2021/12/20.
//

#include "FrameBase.h"
#include "common/serialize/serialize.h"
#include "MapPoint.h"
#include "KannalaBrandt8.h"
#include "radtan.h"
#include "ORBVocabulary.h"

#include "Converter.h"

using namespace VIEO_SLAM;
using std::pair;
using std::set;

cv::Mat FrameBase::mTbc, FrameBase::mTce;
Eigen::Matrix3d FrameBase::meigRcb;
Eigen::Vector3d FrameBase::meigtcb;
bool FrameBase::usedistort_ = false;
bool FrameBase::busedist_set_ = false;
FrameBase::GridInfo FrameBase::gridinfo_;

const Sophus::SE3d FrameBase::GetTwc() { return GetTcw().inverse(); }
const Sophus::SE3d FrameBase::GetTcw() { return GetTcwCst(); }

void FrameBase::AddMapPoint(MapPoint *pMP, const size_t &idx) {
  assert(mvpMapPoints.size() > idx);
  //  if (mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad()) {
  //    cout << "check mpid=" << mvpMapPoints[idx]->mnId << " ";
  //    auto vobs = mvpMapPoints[idx]->GetObservations();
  //    for (auto obs : vobs) {
  //      cout << obs.first->nid_ << ":";
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

void FrameBase::ComputeBoW() {
  if (mBowVec.empty()) {
    // transform Mat(N*32*8U) to vec<Mat>(N*1*32*8U)
    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // compute mBowVec && mFeatVec(at level (d-levelsup)6-4) of this FrameBase:
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
  } else
    assert(!mFeatVec.empty());
}

vector<size_t> FrameBase::GetFeaturesInArea(uint8_t cami, const float &x, const float &y, const float &r,
                                            const int minlevel, const int maxlevel) const {
  vector<size_t> vIndices;
  if (vgrids_.empty()) return vIndices;
  vIndices.reserve(N);
  assert(gridinfo_.fgrids_widthinv_.size() > cami && gridinfo_.fgrids_heightinv_.size() > cami);

  const int min_cellx = max(0, (int)floor((x - gridinfo_.minmax_xy_[cami][0] - r) * gridinfo_.fgrids_widthinv_[cami]));
  if (min_cellx >= gridinfo_.FRAME_GRID_COLS) return vIndices;

  const int max_cellx = min((int)gridinfo_.FRAME_GRID_COLS - 1,
                            (int)ceil((x - gridinfo_.minmax_xy_[cami][0] + r) * gridinfo_.fgrids_widthinv_[cami]));
  if (max_cellx < 0) return vIndices;

  const int min_celly = max(0, (int)floor((y - gridinfo_.minmax_xy_[cami][2] - r) * gridinfo_.fgrids_heightinv_[cami]));
  if (min_celly >= gridinfo_.FRAME_GRID_ROWS) return vIndices;

  const int max_celly = min((int)gridinfo_.FRAME_GRID_ROWS - 1,
                            (int)ceil((y - gridinfo_.minmax_xy_[cami][2] + r) * gridinfo_.fgrids_heightinv_[cami]));
  if (max_celly < 0) return vIndices;

  const bool bchecklevel = (minlevel > 0) || (maxlevel >= 0);  // minlevel==0&&maxlevel<0 don't need to judge

  assert(!mpCameras.empty());
  for (int ix = min_cellx; ix <= max_cellx; ++ix) {
    for (int iy = min_celly; iy <= max_celly; ++iy) {
      const vector<size_t> vCell = vgrids_[cami][ix * gridinfo_.FRAME_GRID_ROWS + iy];
      for (size_t j = 0, jend = vCell.size(); j < jend; ++j) {
        const cv::KeyPoint &kpUn = !usedistort_ ? mvKeysUn[vCell[j]] : mvKeys[vCell[j]];
        if (bchecklevel)  // if the octave is out of level range
        {
          //-1 is also ok,0 cannot be true
          if (kpUn.octave < minlevel) continue;
          // avoid for -1
          if (maxlevel >= 0)
            if (kpUn.octave > maxlevel) continue;
        }

        const float distx = kpUn.pt.x - x;
        const float disty = kpUn.pt.y - y;

        if (fabs(distx) < r && fabs(disty) < r) vIndices.push_back(vCell[j]);
      }
    }
  }

  return vIndices;
}
void FrameBase::AssignFeaturesToGrid() {
  size_t n_cams = mpCameras.size();
  assert(n_cams);
  vgrids_.resize(n_cams, vector<vector<size_t>>(gridinfo_.FRAME_GRID_COLS * gridinfo_.FRAME_GRID_ROWS));
  int nReserve = 0.5f * N / (gridinfo_.FRAME_GRID_COLS * gridinfo_.FRAME_GRID_ROWS * n_cams);
  for (size_t cami = 0; cami < n_cams; ++cami) {
    unsigned int k = 0;
    for (unsigned int i = 0; i < gridinfo_.FRAME_GRID_COLS; ++i)
      for (unsigned int j = 0; j < gridinfo_.FRAME_GRID_ROWS; ++j) vgrids_[cami][k++].reserve(nReserve);
  }
  for (int i = 0; i < N; ++i) {
    const cv::KeyPoint &kp = !usedistort_ ? mvKeysUn[i] : mvKeys[i];

    int ngridposx, ngridposy;
    auto cami = mapn2in_.size() <= i ? 0 : get<0>(mapn2in_[i]);
    if (PosInGrid(cami, kp, ngridposx, ngridposy)) {
      vgrids_[cami][ngridposx * gridinfo_.FRAME_GRID_ROWS + ngridposy].push_back(i);
    }
  }
}
bool FrameBase::PosInGrid(uint8_t cami, const cv::KeyPoint &kp, int &posX, int &posY) {
  posX = round((kp.pt.x - gridinfo_.minmax_xy_[cami][0]) * gridinfo_.fgrids_widthinv_[cami]);
  posY = round((kp.pt.y - gridinfo_.minmax_xy_[cami][2]) * gridinfo_.fgrids_heightinv_[cami]);

  // Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX < 0 || posX >= gridinfo_.FRAME_GRID_COLS || posY < 0 || posY >= gridinfo_.FRAME_GRID_ROWS) return false;
  return true;
}
bool FrameBase::IsInImage(uint8_t cami, const float &x, const float &y) const {
  return (x >= gridinfo_.minmax_xy_[cami][0] && x < gridinfo_.minmax_xy_[cami][1] &&
          y >= gridinfo_.minmax_xy_[cami][2] && y < gridinfo_.minmax_xy_[cami][3]);
}
void FrameBase::ComputeImageBounds(const vector<int> &wid_hei) {
  auto sz_cams = mpCameras.size();
  if (gridinfo_.fgrids_widthinv_.size() > sz_cams) {
    assert(gridinfo_.fgrids_heightinv_.size() > sz_cams);
    assert(gridinfo_.minmax_xy_.size() > sz_cams);
    return;
  }

  assert(wid_hei.size() == 2);
  for (int i = 0; i < 2; ++i) gridinfo_.sz_dims_[i] = wid_hei[i];
  gridinfo_.fgrids_widthinv_.resize(sz_cams + 1);
  gridinfo_.fgrids_heightinv_.resize(sz_cams + 1);
  gridinfo_.minmax_xy_.resize(sz_cams + 1);
  for (size_t icam = 0; icam < sz_cams; ++icam) {
    // if not usedistort_ we should calc undistorted 4 corner pts(when it's pinhole, they're the same)
    if (mpCameras[0]->GetType() != mpCameras[0]->CAM_PINHOLE && !usedistort_) {
      Eigen::Matrix<float, 4, 2> mat;
      mat << 0.f, 0.f, (float)gridinfo_.sz_dims_[0], 0.f, 0.f, (float)gridinfo_.sz_dims_[1],
          (float)gridinfo_.sz_dims_[0], (float)gridinfo_.sz_dims_[1];

      // Undistort corners
      for (int i = 0; i < 4; ++i) {
        cv::Mat mattmp = mpCameras[icam]->toKcv() * mpCameras[icam]->unprojectMat(cv::Point2f(mat(i, 0), mat(i, 1)));
        mat(i, 0) = mattmp.at<float>(0);
        mat(i, 1) = mattmp.at<float>(1);
      }

      gridinfo_.minmax_xy_[icam][0] = min(mat(0, 0), mat(2, 0));
      gridinfo_.minmax_xy_[icam][1] = max(mat(1, 0), mat(3, 0));
      gridinfo_.minmax_xy_[icam][2] = min(mat(0, 1), mat(1, 1));
      gridinfo_.minmax_xy_[icam][3] = max(mat(2, 1), mat(3, 1));
    } else {
      gridinfo_.minmax_xy_[icam][0] = 0.0f;
      gridinfo_.minmax_xy_[icam][1] = gridinfo_.sz_dims_[0];
      gridinfo_.minmax_xy_[icam][2] = 0.0f;
      gridinfo_.minmax_xy_[icam][3] = gridinfo_.sz_dims_[1];
    }

    // divide the img into ORB2SLAM like 64*48(rows) grids for features matching!
    gridinfo_.fgrids_widthinv_[icam] =
        gridinfo_.FRAME_GRID_COLS / (gridinfo_.minmax_xy_[icam][1] - gridinfo_.minmax_xy_[icam][0]);
    gridinfo_.fgrids_heightinv_[icam] =
        gridinfo_.FRAME_GRID_ROWS / (gridinfo_.minmax_xy_[icam][3] - gridinfo_.minmax_xy_[icam][2]);
  }
}

bool FrameBase::read(istream &is) {
  is.read((char *)&timestamp_, sizeof(timestamp_));
  ftimestamp_ = TS2S(timestamp_);

  auto usedistort = usedistort_;
  is.read((char *)&usedistort, sizeof(usedistort));
  if (busedist_set_)
    assert(usedistort == usedistort_);
  else {
    usedistort_ = usedistort;
    busedist_set_ = true;
  }
  uint8_t sz_cams;
  is.read((char *)&sz_cams, sizeof(sz_cams));
  mpCameras.resize(sz_cams);
  for (auto i = 0; i < sz_cams; ++i) {
    int cam_type;
    is.read((char *)&cam_type, sizeof(cam_type));
    uint8_t sz_params_tmp;
    is.read((char *)&sz_params_tmp, sizeof(sz_params_tmp));
    vector<float> params_tmp(sz_params_tmp);
    Serialize::readVec(is, params_tmp);
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

  is.read((char *)&N, sizeof(N));
  mvKeys.resize(N);
  Serialize::readVec(is, mvKeys);
  if (!usedistort) {
    mvKeysUn.resize(N);
    Serialize::readVec(is, mvKeysUn);
  }
  int n_tmp;
  is.read((char *)&n_tmp, sizeof(n_tmp));
  mapn2in_.resize(n_tmp);
  Serialize::readVec(is, mapn2in_);
  // we don't save old ID for it's useless in LoadMap()
  is.read((char *)&mThDepth, sizeof(mThDepth));
  mDescriptors = cv::Mat::zeros(N, 32, CV_8UC1);  // 256bit binary descriptors
  Serialize::readMat(is, mDescriptors);
  ComputeBoW();  // calculate mBowVec & mFeatVec, or we can do it by pKF
  stereoinfo_.vdepth_.resize(N);
  stereoinfo_.vuright_.resize(N);
  Serialize::readVec(is, stereoinfo_.vdepth_);
  Serialize::readVec(is, stereoinfo_.vuright_);
  is.read((char *)&n_tmp, sizeof(n_tmp));
  stereoinfo_.v3dpoints_.resize(n_tmp);
  Serialize::readVecEigMat(is, stereoinfo_.v3dpoints_);
  stereoinfo_.goodmatches_.resize(n_tmp);
  Serialize::readVec(is, stereoinfo_.goodmatches_);
  is.read((char *)&n_tmp, sizeof(n_tmp));
  vector<size_t> vec_tmp(n_tmp * 3);
  Serialize::readVec(is, vec_tmp);
  for (int i = 0; i < n_tmp; ++i) {
    stereoinfo_.mapcamidx2idxs_.emplace(make_pair(vec_tmp[i * 3], vec_tmp[i * 3 + 1]), vec_tmp[i * 3 + 2]);
  }
  is.read((char *)&stereoinfo_.baseline_bf_, sizeof(stereoinfo_.baseline_bf_[0]));
  // trans unit from meter to pixel
  stereoinfo_.baseline_bf_[1] = stereoinfo_.baseline_bf_[0] * mpCameras[0]->toK()(0, 0);

  is.read((char *)&n_tmp, sizeof(n_tmp));
  is.read((char *)&scalepyrinfo_.fscalefactor_, sizeof(scalepyrinfo_.fscalefactor_));
  {
    scalepyrinfo_.flogscalefactor_ = log(scalepyrinfo_.fscalefactor_);
    scalepyrinfo_.vscalefactor_.resize(n_tmp);
    scalepyrinfo_.vlevelsigma2_.resize(n_tmp);
    scalepyrinfo_.vinvlevelsigma2_.resize(n_tmp);
    scalepyrinfo_.vscalefactor_[0] = scalepyrinfo_.vlevelsigma2_[0] = 1.0f;
    for (int i = 1; i < n_tmp; ++i) {
      scalepyrinfo_.vscalefactor_[i] = scalepyrinfo_.vscalefactor_[i - 1] * scalepyrinfo_.fscalefactor_;
      // at 0 level sigma=1 pixel
      scalepyrinfo_.vlevelsigma2_[i] = scalepyrinfo_.vscalefactor_[i] * scalepyrinfo_.vscalefactor_[i];
      scalepyrinfo_.vinvlevelsigma2_[i] = 1.0f / scalepyrinfo_.vlevelsigma2_[i];
    }
  }

  // Now grid_info_ is the same as all Frames
  is.read((char *)&gridinfo_.sz_dims_, sizeof(gridinfo_.sz_dims_));
  ComputeImageBounds(vector<int>{gridinfo_.sz_dims_[0], gridinfo_.sz_dims_[1]});

  // load vgrids_[i][j]
  AssignFeaturesToGrid();

  // load mvpMapPoints,{mpParent,mbNotErase(mspLoopEdges)} in LoadMap for convenience
  return is.good();
}
bool FrameBase::write(ostream &os) const {
  os.write((char *)&timestamp_, sizeof(timestamp_));

  os.write((char *)&usedistort_, sizeof(usedistort_));
  uint8_t sz_cams = (uint8_t)mpCameras.size();
  os.write((char *)&sz_cams, sizeof(sz_cams));
  for (auto i = 0; i < sz_cams; ++i) {
    int cam_type = (int)mpCameras[i]->GetType();
    os.write((char *)&cam_type, sizeof(cam_type));
    vector<float> params_tmp = mpCameras[i]->getParameters();
    uint8_t sz_params_tmp = (uint8_t)params_tmp.size();
    os.write((char *)&sz_params_tmp, sizeof(sz_params_tmp));
    Serialize::writeVec(os, params_tmp);
  }

  os.write((char *)&N, sizeof(N));
  Serialize::writeVec(os, mvKeys);
  if (!usedistort_) {
    Serialize::writeVec(os, mvKeysUn);
  }
  int n_tmp = mapn2in_.size();
  os.write((char *)&n_tmp, sizeof(n_tmp));
  Serialize::writeVec(os, mapn2in_);
  // we don't save old ID for it's useless in LoadMap(), but we'll save old KF ID/nid_ in SaveMap()
  os.write((char *)&mThDepth, sizeof(mThDepth));
  Serialize::writeMat(os, mDescriptors);
  // we can directly ComputeBoW() from mDescriptors
  //   mBowVec.write(os);
  //   mFeatVec.write(os);
  Serialize::writeVec(os, stereoinfo_.vdepth_);
  Serialize::writeVec(os, stereoinfo_.vuright_);
  n_tmp = stereoinfo_.v3dpoints_.size();
  os.write((char *)&n_tmp, sizeof(n_tmp));
  Serialize::writeVecEigMat(os, stereoinfo_.v3dpoints_);
  assert(stereoinfo_.goodmatches_.size() == n_tmp);
  Serialize::writeVec(os, stereoinfo_.goodmatches_);
  n_tmp = stereoinfo_.mapcamidx2idxs_.size();
  os.write((char *)&n_tmp, sizeof(n_tmp));
  Serialize::writeVec(os, stereoinfo_.mapcamidx2idxs_);
  os.write((char *)&stereoinfo_.baseline_bf_, sizeof(stereoinfo_.baseline_bf_[0]));  // we can get bf from b & f

  n_tmp = scalepyrinfo_.vscalefactor_.size();
  os.write((char *)&n_tmp, sizeof(n_tmp));
  os.write((char *)&scalepyrinfo_.fscalefactor_, sizeof(scalepyrinfo_.fscalefactor_));
  // we can get left scalepyrinfo_ members from former 2 parameters

  // Now grid_info_ is the same as all Frames
  os.write((char *)&gridinfo_.sz_dims_, sizeof(gridinfo_.sz_dims_));

  // we can still get it from mvKeys(Un)
  //  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
  //    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) writeVec(os, vgrids_[i][j]);

  // save mvpMapPoints,{mpParent,mbNotErase(mspLoopEdges)} in LoadMap for convenience
  return os.good();
}

size_t FrameBase::GetMapn2idxs(size_t i) {
  if (mapn2in_.size() <= i) return -1;  // for no mpCameras mode sz is 0
  auto iteridx = stereoinfo_.mapcamidx2idxs_.find(mapn2in_[i]);
  if (iteridx == stereoinfo_.mapcamidx2idxs_.end())
    return -1;
  else
    return iteridx->second;
}
