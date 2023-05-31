/**
 * This file is part of VIEO_SLAM
 */

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace VIEO_SLAM {

class ORBmatcher {
 public:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

  // Computes the Hamming distance between two ORB descriptors(1*256bit/32*8bit)
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

  enum ModeSBP { SBPFuseLater = 0x1, SBPMatchMultiCam = 0x2 };

  ORBmatcher(float nnratio = 0.6, bool checkOri = true);

  static void SearchByProjectionBase(const vector<MapPoint *> &vpMapPoints1, cv::Mat Rcrw, cv::Mat tcrw, KeyFrame *pKF,
                                     const float th_radius, const float th_bestdist, bool bCheckViewingAngle = false,
                                     const float *pbf = nullptr, int *pnfused = nullptr,
                                     char mode = (char)SBPMatchMultiCam,
                                     vector<vector<bool>> *pvbAlreadyMatched1 = nullptr,
                                     vector<set<int>> *pvnMatch1 = nullptr);

  // Search matches between Frame keypoints and projected MapPoints. Returns number of additional matches
  // Used to track the local map (Tracking)
  // rectify the F.mvpMapPoints
  int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints, const float th = 3,
                         const float th_far_pts = 0);

  // Project MapPoints tracked in last frame into the current frame and search matches.
  // Used to track from previous frame (Tracking)
  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono,
                         const float th_far_pts = 0);

  // Project MapPoints seen in KeyFrame into the Frame and search matches. Returns number of additional matches
  // Used in relocalisation (Tracking)
  // rectify CurrentFrame.mvpMapPoints
  int SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const std::set<MapPoint *> &sAlreadyFound, const float th,
                         const int ORBdist, const float th_far_pts = 0);

  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in loop detection (Loop Closing)
  // rectify vpMatched, return the number of additional matches,
  // like SBP(Frame,vec<MP*>)+SearchBySim3, notice additional match doesn't need pKF->mvpMapPoints[i] exists && this
  // func. doesn't check the orientation
  int SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints,
                         std::vector<MapPoint *> &vpMatched, int th);

  // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
  // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
  // Used in Relocalisation/TrackReferenceKeyFrame() and Loop Detection
  int SearchByBoW(KeyFrame *pKF, Frame &F,
                  std::vector<MapPoint *> &vpMapPointMatches);  // don't use pKF->mBowVec,juse use pKF->mFeatVec to
                                                                // match, rectify vpMapPointMatches(21)
  int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2,
                  std::vector<MapPoint *> &vpMatches12);  // rectify vpMatches12 by using pKF->mFeatVec to quickly
                                                          // match, corresponding to pKF1/mpCurrentKF in LoopClosing

  // Matching for the Map Initialization (only used in the monocular case)
  int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched,
                              std::vector<int> &vnMatches12, int windowSize = 10);

  // Matching to triangulate new MapPoints. Check Epipolar Constraint.
  // used in CreateNewMapPoints() in LocalMapping thread without checkOri(=false);return number of additional matches
  // which haven't been created as MapPoints;old_vesrion may return some vMatchedPairs with same it->second!
  int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<vector<vector<size_t>>> &vMatchedPairs,
                             const bool bOnlyStereo);

  // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
  // In the stereo and RGB-D case, s12=1
  int SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12,
                   const cv::Mat &R12, const cv::Mat &t12,
                   const float th);  // rectify vpMatches12, return number of additional matches by S12 && S21(project
                                     // pKF1's MPs into pKF2) SBP matching and validation

  // Project MapPoints into KeyFrame and search for duplicated MapPoints.
  // rectify pKF->mvpMapPoints, and also may rectify vpMapPoints when fusing a better MP by replace(),
  // Matching method is like SBP but lots of validation is used for safe
  int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th = 3.0);

  // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
  // return number <= the real fused/added matched MPs in pKF->mvpMapPoints,
  // rectify vpReplacePoint(to be replaced),
  // matching method is similar to SearchByProjection(KF*,cvScw,vec<MP*>,vec<MP*>), also need lots of validation but no
  // chi2 distr. error check in Fuse(KF*,vec<MP*>)
  int Fuse(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, float th,
           vector<MapPoint *> &vpReplacePoint);

 protected:
  bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12,
                             const KeyFrame *pKF);  // 95% confidence level when return true

  float RadiusByViewingCos(const float &viewCos);

  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

  float mfNNratio;
  bool mbCheckOrientation;
};

}  // namespace VIEO_SLAM

#endif  // ORBMATCHER_H
