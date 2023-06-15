/**
 * This file is part of VIEO_SLAM
 */

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "FrameBase.h"
#include "MapPoint.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace VIEO_SLAM {
class KeyFrame;
class MapPoint;
class GeometricCamera;

class Frame : public FrameBase {
  EncPreIntegrator *ppreint_enc_kf_;
  IMUPreintegrator *ppreint_imu_kf_;

 public:
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
  int PreIntegrationFromLastKF(FrameBase *plastkf, double tmi, double tmj_1,
                               const typename aligned_list<OdomData>::const_iterator &iteri,
                               const typename aligned_list<OdomData>::const_iterator &iterj, bool breset = false,
                               int8_t verbose = 0) {
    CV_Assert(ppreint_enc_kf_);
    NavState ns = plastkf->GetNavState();
    if (breset) CV_Assert(plastkf->ftimestamp_ == tmi);
    return FrameBase::PreIntegration<OdomData>(tmi, tmj_1, ns.mbg, ns.mba, iteri, iterj, breset, ppreint_enc_kf_,
                                               verbose);
  }

  // for LoadMap() in System.cc
  Frame(istream &is, ORBVocabulary *voc);
  bool read(istream &is, bool bOdomList = false);  // we use Frame::read(is,false) before KeyFrame's constructor
  bool write(ostream &os) const;                   // though we don't save Frame

 public:
  // for quaterniond in NavState && Matrix4d
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame();

  // Copy constructor. for Tcw is cv::Mat, we need to deep copy it
  // =default for the vgrids_[A][B] copy is OK for it's expanded in this class as static memory allocate
  // though descriptors can be deep copied for safety here, but we'll pay attention, so it's shallow copied now
  explicit Frame(const Frame &frame, bool copy_shallow = false);

  // Constructor for stereo/RGB-D(ims.size()>extractors.size())/Monocular(ims.size()==1) cameras.
  Frame(const vector<cv::Mat> &ims, const double &timeStamp, const vector<ORBextractor *> &extractors,
        ORBVocabulary *voc, const vector<GeometricCamera *> &CamInsts, const float &bf, const float &thDepth,
        IMUPreintegrator *ppreint_imu_kf = nullptr, EncPreIntegrator *ppreint_enc_kf = nullptr, bool usedistort = true,
        const float th_far_pts = 0);

  std::vector<MapPoint *> &GetMapPointsRef() { return mvpMapPoints; }

  // Extract ORB on the image. 0 for left image and 1 for right image.
  void ExtractORB(int flag, const cv::Mat &im, std::vector<int> *pvLappingArea = nullptr);

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

  // Search a match for each keypoint in the left image to a keypoint in the right image.
  // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
  void ComputeStereoMatches();

  void ComputeStereoFishEyeMatches(const float th_far_pts = 0);

  // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
  void ComputeStereoFromRGBD(const cv::Mat &imDepth);

  // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
  cv::Mat UnprojectStereo(const int &i);

 public:
  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;

  // Current and Next Frame id.
  static long unsigned int nNextId;

  // Feature extractor. The right is used only in the stereo case.
  vector<ORBextractor *> mpORBextractors;

  // Number of Non Lapping Keypoints
  vector<size_t> num_mono = vector<size_t>(1);
  std::vector<std::vector<cv::KeyPoint>> vvkeys_ = std::vector<std::vector<cv::KeyPoint>>(1);
  std::vector<cv::Mat> vdescriptors_ = std::vector<cv::Mat>(1);

  // For stereo matching
  static cv::BFMatcher BFmatcher;  // for fisheye matching
  // Triangulated stereo observations using as reference the left camera. These are
  // computed during ComputeStereoFishEyeMatches
  vector<vector<size_t>> mvidxsMatches;
  // 1 ididxs-> n_cams n, here only pick one, now it's the latest one
  vector<size_t> mapidxs2n_;
  std::vector<std::vector<size_t>> mapin2n_;  // mapcamidx2n_ for addobs func.

  // Reference Keyframe.
  KeyFrame *mpReferenceKF = nullptr;

  // Scale pyramid info. to speed up for ComputeStereoMatches
  vector<float> mvInvScaleFactors;

  cv::Mat &GetTcwRef() { return Tcw_; }
  inline const Sophus::SE3d GetTcwCst() const { return FrameBase::GetTcwCst(); }
  const cv::Mat &GetcvTcwCst() const;

 private:
  void UndistortKeyPoints();

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
int Frame::PreIntegrationFromLastKF<IMUData>(FrameBase *plastkf, double tmi, double tmj_1,
                                             const typename aligned_list<IMUData>::const_iterator &iteri,
                                             const typename aligned_list<IMUData>::const_iterator &iterj, bool breset,
                                             int8_t verbose);

}  // namespace VIEO_SLAM

#endif  // FRAME_H
