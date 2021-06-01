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

#include "OdomPreIntegrator.h"//zzh
#include "NavState.h"

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace VIEO_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{ 
public:
  //const Tbc,Tce, so it can be used in multi threads
  static cv::Mat mTbc,mTce;
  static Eigen::Matrix3d meigRcb;static Eigen::Vector3d meigtcb;
  
  // Odom PreIntegration, j means this frame, i means last frame(not KF), if no measurements it will be cv::Mat()
  EncPreIntegrator mOdomPreIntEnc;
  IMUPreintegrator mOdomPreIntIMU;
  // state xi={Ri,pi,vi,bi}, not designed for multi threads/just used in Tracking thread
  NavState mNavState;
  // For pose optimization/motion-only BA, use as prior and prior information(inverse covariance)
  Matrix<double,15,15> mMargCovInv;//Sigmap in VIORBSLAM paper/prior Hessian matrix for next Frame, notice it's unintialized(to infinity/fixedlast)
  NavState mNavStatePrior;//needed by PoseOptimization twice, notice if no imu data, it's unintialized
  bool mbPrior;//meaning if mNavStatePrior&mMargCovInv exist
  
  const NavState& GetNavState(void){//cannot use const &(make mutex useless)
    return mNavState;//don't call copy constructor, just for template of PoseOptimization() & PreIntegration
  }
  void UpdatePoseFromNS();//replace SetPose(), directly update mNavState for efficiency and then please call this func. to update Tcw
  void UpdateNavStatePVRFromTcw();//for imu data empty condition after imu's initialized(including bias recomputed)
  
  // Odom PreIntegration
//   template <class _OdomData>
//   void SetPreIntegrationList(const typename std::list<_OdomData>::const_iterator &begin,const typename std::>::const_iterator &pback){//notice template definition should be written in the same file! & typename should be added before nested type!
//     mOdomPreIntEnc.SetPreIntegrationList(begin,pback);
//   }
  template <class _OdomData>//here if u use _Frame*, it can be automatically checked while vector<_Frame>::iterator won't do so! but partial specialized template function doesn't exist!
  void PreIntegration(Frame* pLastF,const typename listeig(_OdomData)::const_iterator &iteri,
		      typename listeig(_OdomData)::const_iterator iterjBack){//0th frame don't use this function
    mOdomPreIntEnc.PreIntegration(pLastF->mTimeStamp,mTimeStamp,iteri,++iterjBack);//here need ++!
  }
  template <class _OdomData>
  void PreIntegration(KeyFrame* pLastKF,const typename listeig(_OdomData)::const_iterator &iteri,
		      typename listeig(_OdomData)::const_iterator iterjBack);//0th frame don't use this function, just declare and we only use specialized 2 versions for KeyFrame cannot be directly used in this head file
  
  //for LoadMap() in System.cc
  Frame(istream &is,ORBVocabulary* voc);
  bool read(istream &is,bool bOdomList=false);//we use Frame::read(is,false) before KeyFrame's constructor
  bool write(ostream &os) const;//though we don't save Frame
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW//for quaterniond in NavState && Matrix4d
//created by zzh over.
  
public:
    Frame();

    // Copy constructor. use default = func. is the mGrid[A][B] copy OK?
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();//compute mBowVec && mFeatVec

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

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

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

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
    cv::Mat mOw;//==mtwc, the center of the left camera in the world/cam0 frame
};

//created by zzh
// template <>//specialized
// void Frame::SetPreIntegrationList<IMUData>(const typename std::list<IMUData>::const_iterator &begin,const typename std::list<IMUData>::const_iterator &pback);
template <>
void Frame::PreIntegration<IMUData>(Frame* pLastF,const listeig(IMUData)::const_iterator &iteri,listeig(IMUData)::const_iterator iterjBack);
template <>
void Frame::PreIntegration<EncData>(KeyFrame* pLastKF,const listeig(EncData)::const_iterator &iteri,listeig(EncData)::const_iterator iterjBack);
template <>
void Frame::PreIntegration<IMUData>(KeyFrame* pLastKF,const listeig(IMUData)::const_iterator &iteri,listeig(IMUData)::const_iterator iterjBack);

}// namespace ORB_SLAM

#endif // FRAME_H
