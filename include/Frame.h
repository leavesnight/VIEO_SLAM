/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/leavesnight/VIEO_SLAM>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "FrameBase.h"
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace VIEO_SLAM {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class KeyFrame;
class MapPoint;
class GeometricCamera;

class Frame : public FrameBase {
  EncPreIntegrator *ppreint_enc_kf_;
  IMUPreintegrator *ppreint_imu_kf_;

 public:
  // const Tbc,Tce, so it can be used in multi threads
  static cv::Mat mTbc, mTce;
  static Eigen::Matrix3d meigRcb;
  static Eigen::Vector3d meigtcb;

  // For pose optimization/motion-only BA, use as prior and prior information(inverse covariance)
  Matrix<double, 15, 15> mMargCovInv;  // Sigmap in VIORBSLAM paper/prior Hessian matrix for next Frame, notice it's
                                       // unintialized(to infinity/fixedlast)
  NavState mNavStatePrior;             // needed by PoseOptimization twice, notice if no imu data, it's unintialized
  bool mbPrior;                        // meaning if mNavStatePrior&mMargCovInv exist

  const std::vector<MapPoint *> &GetMapPointMatches() const { return mvpMapPoints; }
  const NavState &GetNavState() const { return mNavState; }
  NavState &GetNavStateRef() { return mNavState; }
  // rewrite the one in FrameBase for efficiency
  const EncPreIntegrator &GetEncPreInt(void) const { return mOdomPreIntEnc; }
  const IMUPreintegrator &GetIMUPreInt(void) const { return mOdomPreIntIMU; }
  template <class OdomPreintegrator>
  void DeepMovePreintOdomFromLastKF(OdomPreintegrator &preint_odom) {
    preint_odom = *ppreint_enc_kf_;
    auto &lodom = ppreint_enc_kf_->GetRawDataRef();
    preint_odom.AppendFrontPreIntegrationList(lodom, lodom.begin(), lodom.end());
  }
  void UpdatePoseFromNS();  // replace SetPose(), directly update mNavState for efficiency and then please call this
                            // func. to update Tcw
  void UpdateNavStatePVRFromTcw();  // for imu data empty condition after imu's initialized(including bias recomputed)

  // Odom PreIntegration
  //[iteri,iterj) IMU preintegration, breset=false could make KF2KF preintegration time averaged to per frame &&
  // connect 2KFs preintegration by only preintegrating the final KF2KF period
  template <class OdomData>
  int PreIntegrationFromLastKF(FrameBase *plastkf, FrameBase *plastfb_kf,
                               const typename aligned_list<OdomData>::const_iterator &iteri,
                               const typename aligned_list<OdomData>::const_iterator &iterj, bool breset = false,
                               int8_t verbose = 0) {
    CV_Assert(ppreint_enc_kf_);
    NavState ns = plastkf->GetNavState();
    auto iterj_1 = iterj;
    --iterj_1;
    return FrameBase::PreIntegration<OdomData>(breset ? plastfb_kf->mTimeStamp : (*iteri).mtm, (*iterj_1).mtm, ns.mbg,
                                               ns.mba, iteri, iterj, breset, ppreint_enc_kf_, verbose);
  }

  // for LoadMap() in System.cc
  Frame(istream &is, ORBVocabulary *voc);
  bool read(istream &is, bool bOdomList = false);  // we use Frame::read(is,false) before KeyFrame's constructor
  bool write(ostream &os) const;                   // though we don't save Frame

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // for quaterniond in NavState && Matrix4d
  // created by zzh over.
  Frame();

  // Copy constructor. use default = func. is the mGrid[A][B] copy OK?
  Frame(const Frame &frame);

  // Constructor for stereo cameras.
  Frame(const vector<cv::Mat> &ims, const double &timeStamp, vector<ORBextractor *> extractors, ORBVocabulary *voc,
        cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
        IMUPreintegrator *ppreint_imu_kf = nullptr, EncPreIntegrator *ppreint_enc_kf = nullptr,
        const vector<GeometricCamera *> *pCamInsts = nullptr, bool usedistort = true);

  // Constructor for RGB-D cameras.
  Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor,
        ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
        IMUPreintegrator *ppreint_imu_kf = nullptr, EncPreIntegrator *ppreint_enc_kf = nullptr);

  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K,
        cv::Mat &distCoef, const float &bf, const float &thDepth, IMUPreintegrator *ppreint_imu_kf = nullptr,
        EncPreIntegrator *ppreint_enc_kf = nullptr);

  std::vector<MapPoint *> &GetMapPointsRef() { return mvpMapPoints; }

  // Extract ORB on the image. 0 for left image and 1 for right image.
  void ExtractORB(int flag, const cv::Mat &im, std::vector<int> *pvLappingArea = nullptr);

  // Compute Bag of Words representation.
  void ComputeBoW();  // compute mBowVec && mFeatVec

  // Set the camera pose.
  void SetPose(cv::Mat Tcw);

  // Computes rotation, translation and camera center matrices from the camera pose.
  void UpdatePoseMatrices();

  // Returns the camera center.
  inline cv::Mat GetCameraCenter() { return mOw.clone(); }

  // Returns inverse of rotation
  inline cv::Mat GetRotationInverse() { return mRwc.clone(); }

  // Check if a MapPoint is in the frustum of the camera
  // and fill variables of the MapPoint to be used by the tracking
  bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

  // Compute the cell of a keypoint (return false if outside the grid)
  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

  vector<size_t> GetFeaturesInArea(size_t cami, const float &x, const float &y, const float &r, const int minLevel = -1,
                                   const int maxLevel = -1) const;

  // Search a match for each keypoint in the left image to a keypoint in the right image.
  // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
  void ComputeStereoMatches();

  void ComputeStereoFishEyeMatches();

  // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
  void ComputeStereoFromRGBD(const cv::Mat &imDepth);

  // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
  cv::Mat UnprojectStereo(const int &i);

 public:
  // Vocabulary used for relocalization.
  ORBVocabulary *mpORBvocabulary;

  // Feature extractor. The right is used only in the stereo case.
  vector<ORBextractor *> mpORBextractors;

  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints.
  int N;
  // Number of Non Lapping Keypoints
  vector<size_t> num_mono = vector<size_t>(1);
  // For stereo matching
  vector<vector<size_t>> mvidxsMatches;
  vector<bool> goodmatches_;                          // keep same size with mvidxsMatches
  map<pair<size_t, size_t>, size_t> mapcamidx2idxs_;  // final size_t max < mvidxsMatches.size()
  size_t GetMapn2idxs(size_t i);
  vector<size_t> mapidxs2n_;
  // For stereo fisheye matching
  static cv::BFMatcher BFmatcher;
  // Triangulated stereo observations using as reference the left camera. These are
  // computed during ComputeStereoFishEyeMatches
  aligned_vector<Vector3d> mv3Dpoints;  // keep same size with mvidxsMatches

  // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
  // In the stereo case, mvKeysUn is redundant as images must be rectified.
  // In the RGB-D case, RGB images can be distorted.
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<cv::KeyPoint> mvKeysUn;
  std::vector<std::vector<cv::KeyPoint>> vvkeys_ = std::vector<std::vector<cv::KeyPoint>>(1);
  std::vector<std::vector<size_t>> mapin2n_;  // mapcamidx2n_ for addobs func.

  // Corresponding stereo coordinate and depth for each keypoint.
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // Bag of Words Vector structures.
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors;
  std::vector<cv::Mat> vdescriptors_ = std::vector<cv::Mat>(1);

  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;

  // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::vector<std::vector<std::vector<std::size_t>>>> vgrids_;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe.
  KeyFrame *mpReferenceKF;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  vector<float> mvScaleFactors;
  vector<float> mvInvScaleFactors;
  vector<float> mvLevelSigma2;
  vector<float> mvInvLevelSigma2;

  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;

  static bool usedistort_;

  cv::Mat &GetTcwRef() { return Tcw_; }
  inline const Sophus::SE3d GetTcwCst() const { return FrameBase::GetTcwCst(); }
  const cv::Mat &GetcvTcwCst() const;

 private:
  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case. Stereo must be already rectified!
  // (called in the constructor).
  void UndistortKeyPoints();

  // Computes image bounds for the undistorted image (called in the constructor).
  void ComputeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the constructor).
  void AssignFeaturesToGrid();

  // Rotation, translation and camera center
  cv::Mat mRcw;
  cv::Mat mtcw;
  cv::Mat mRwc;
  cv::Mat mOw;  //==mtwc, the center of the left camera in the world/cam0 frame
};

// created by zzh
template <>
void Frame::DeepMovePreintOdomFromLastKF(IMUPreintegrator &preint_odom);
template <>
int Frame::PreIntegrationFromLastKF<IMUData>(FrameBase *plastkf, FrameBase *plastfb_kf,
                                             const typename aligned_list<IMUData>::const_iterator &iteri,
                                             const typename aligned_list<IMUData>::const_iterator &iterj, bool breset,
                                             int8_t verbose);

}  // namespace VIEO_SLAM

#endif  // FRAME_H
