/**
 * This file is part of VIEO_SLAM
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace VIEO_SLAM {

FrameDrawer::FrameDrawer(Map *pMap) : mpMap(pMap) {
  mState = Tracking::SYSTEM_NOT_READY;
  mIms.resize(1);
  mIms[0] = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

cv::Mat FrameDrawer::DrawFrame(int cami) {
  cv::Mat im;
  vector<cv::KeyPoint> vIniKeys;      // Initialization: KeyPoints in reference frame
  vector<int> vMatches;               // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> vCurrentKeys;  // KeyPoints in current frame
  vector<bool> vbVO, vbMap;           // Tracked MapPoints in current frame
  int state;                          // Tracking state
#ifdef DRAW_KP2MP_LINE
  vector<KptDraw> kpts_proj;
#endif

  // Copy variables within scoped mutex
  {
    unique_lock<mutex> lock(mMutex);
    state = mState;
    if (mState == Tracking::SYSTEM_NOT_READY) mState = Tracking::NO_IMAGES_YET;

    if (mIms[cami].empty()) return cv::Mat();
    mIms[cami].copyTo(im);

    if (mState == Tracking::NOT_INITIALIZED) {
      vCurrentKeys = mvCurrentKeys;
      vIniKeys = mvIniKeys;
      vMatches = mvIniMatches;
    } else if (mState == Tracking::OK) {
      vCurrentKeys = mvCurrentKeys;
#ifdef DRAW_KP2MP_LINE
      kpts_proj = kpts_proj_;
#endif
      vbVO = mvbVO;
      vbMap = mvbMap;
    } else if (mState == Tracking::LOST) {
      vCurrentKeys = mvCurrentKeys;
    }
  }  // destroy scoped mutex -> release mutex

  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // Draw
  if (state == Tracking::NOT_INITIALIZED)  // INITIALIZING
  {
    for (unsigned int i = 0; i < vMatches.size(); i++) {
      if (vMatches[i] >= 0) {
        cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt, cv::Scalar(0, 255, 0));
      }
    }
  } else if (state == Tracking::OK)  // TRACKING
  {
    mnTracked = 0;
    mnTrackedVO = 0;
    const float r = 5;
    const int n = vCurrentKeys.size();
    for (int i = 0; i < n; i++) {
      if (mapn2in_.size() > i && cami != get<0>(mapn2in_[i])) continue;
      if (vbVO[i] || vbMap[i]) {
        cv::Point2f pt1, pt2;
        pt1.x = vCurrentKeys[i].pt.x - r;
        pt1.y = vCurrentKeys[i].pt.y - r;
        pt2.x = vCurrentKeys[i].pt.x + r;
        pt2.y = vCurrentKeys[i].pt.y + r;

        // This is a match to a MapPoint in the map
        if (vbMap[i]) {
          cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
          cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
#ifdef DRAW_KP2MP_LINE
          assert(kpts_proj.size() > i);
          if (kpts_proj[i].valid) {
            cv::line(im, kpts_proj[i].pt, vCurrentKeys[i].pt, cv::Scalar(255, 0, 0), 2);
          }
#endif
          mnTracked++;
        } else  // This is match to a "visual odometry" MapPoint created in the last frame
        {
          cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
          cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
          mnTrackedVO++;
        }
      }
#ifdef DRAW_ALL_KPS
      else {
        cv::drawMarker(im, vCurrentKeys[i].pt, cv::Scalar(0, 255, 250), cv::MARKER_CROSS, 15, 1);
      }
#endif
    }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText) {
  stringstream s;
  if (nState == Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (nState == Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (nState == Tracking::OK) {
    if (!mbOnlyTracking)
      s << "SLAM MODE |  ";
    else
      s << "LOCALIZATION | ";
    int nKFs = mpMap->KeyFramesInMap();
    int nMPs = mpMap->MapPointsInMap();
    s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
    if (mnTrackedVO > 0) s << ", + VO matches: " << mnTrackedVO;
  } else if (nState == Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (nState == Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1,
              8);
}

void FrameDrawer::Update(Tracking *pTracker) {
  unique_lock<mutex> lock(mMutex);
  n_cams_ = pTracker->mImGrays.size();
  if (showallimages_) CV_Assert(n_cams_ >= pTracker->mCurrentFrame.mpCameras.size());
  mIms.resize(n_cams_);
  for (int i = 0; i < n_cams_; ++i) pTracker->mImGrays[i].copyTo(mIms[i]);
  mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
  N = mvCurrentKeys.size();
  mapn2in_ = pTracker->mCurrentFrame.mapn2in_;
  mvbVO = vector<bool>(N, false);
  mvbMap = vector<bool>(N, false);
  mbOnlyTracking = pTracker->mbOnlyTracking;

  if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
    mvIniKeys = pTracker->mInitialFrame.mvKeys;
    mvIniMatches = pTracker->mvIniMatches;
  } else if (pTracker->mLastProcessedState == Tracking::OK) {
    const auto &curfmps = pTracker->mCurrentFrame.GetMapPointMatches();
#ifdef DRAW_KP2MP_LINE
    kpts_proj_.clear();
    kpts_proj_.reserve(N);

    static double dist_max_tot = 0, dist_max_rmse[2] = {0}, dist_rmse_tot[2] = {0};
    static size_t num_rmse_tot = 0;
    double dist_rmse[2] = {0};
    size_t count = 0;
#endif
    for (int i = 0; i < N; i++) {
      MapPoint *pMP = curfmps[i];
      if (pMP) {
        if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
          if (pMP->Observations() > 0)
            mvbMap[i] = true;
          else
            mvbVO[i] = true;
        }
      }
#ifdef DRAW_KP2MP_LINE
      KptDraw pt;
      if (mvbMap[i]) {
        // bad one not used for lba opt. but used for motion_only ba opt.
        //          if (/*pMP->isBad() ||*/ mvCurrentKeys[i].octave) {
        //            kpts_proj_.push_back(pt);
        //            continue;
        //          }
        size_t cami = mapn2in_.size() <= i ? 0 : get<0>(mapn2in_[i]);
        if (pTracker->mCurrentFrame.mpCameras.size() <= cami) {
          kpts_proj_.push_back(pt);
          continue;
        }
        auto &pcami = pTracker->mCurrentFrame.mpCameras[cami];
        auto Tcw = pcami->GetTcr() * pTracker->mCurrentFrame.GetTcwCst();
        auto cX = Tcw * Converter::toVector3d(pMP->GetWorldPos());
        auto p2d = pcami->project(cX);
        pt.pt = cv::Point2f(p2d[0], p2d[1]);
        if (cv::norm(pt.pt - mvCurrentKeys[i].pt) > 10)
          cout << "check pt norm=" << cv::norm(pt.pt - mvCurrentKeys[i].pt) << ";" << pt.pt << "/"
               << mvCurrentKeys[i].pt << endl;
        if (pTracker->mCurrentFrame.IsInImage(cami, pt.pt.x, pt.pt.y)) pt.valid = true;

        auto dist = cv::norm(pt.pt - mvCurrentKeys[i].pt);
        dist_rmse[0] += dist * dist;
        if (dist_max_tot < dist) dist_max_tot = dist;
        Vector3d pt3dtest[2];
        pt3dtest[0] = pcami->unproject(p2d);
        pt3dtest[1] = pcami->unproject(Vector2d(mvCurrentKeys[i].pt.x, mvCurrentKeys[i].pt.y));
        dist = (pcami->toK() * (pt3dtest[0] - pt3dtest[1])).segment<2>(0).norm();
        dist_rmse[1] += dist * dist;
        ++count;
      }
      kpts_proj_.push_back(pt);
#endif
    }
#ifdef DRAW_KP2MP_LINE
    cout << "total max dist = " << dist_max_tot << "pixels" << endl;
    num_rmse_tot += count;
    for (int j = 0; j < 2; ++j) {
      dist_rmse_tot[j] += dist_rmse[j];
      dist_rmse[j] = sqrt(dist_rmse[j] / count);
      if (dist_max_rmse[j] < dist_rmse[j]) dist_max_rmse[j] = dist_rmse[j];
      if (j) cout << "undistort plane:";
      cout << "dist_rmse = " << dist_rmse[j] << ", max rmse dist = " << dist_max_rmse[j]
           << ", rmse dist = " << sqrt(dist_rmse_tot[j] / num_rmse_tot) << endl;
    }
#endif
  }
  mState = static_cast<int>(pTracker->mLastProcessedState);
}

}  // namespace VIEO_SLAM
