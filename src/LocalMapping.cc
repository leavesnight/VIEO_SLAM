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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "common/log.h"

#include <mutex>

//#define NO_GBA_THREAD
//#define NO_LOCALMAP_PROCESS

namespace VIEO_SLAM {

LocalMapping::LocalMapping(Map *pMap, const bool bMonocular, const string &strSettingPath)
    : mbMonocular(bMonocular),
      mbResetRequested(false),
      mbFinishRequested(false),
      mbFinished(true),
      mpMap(pMap),
      mbAbortBA(false),
      mbStopped(false),
      mbStopRequested(false),
      mbNotStop(false),
      mbAcceptKeyFrames(true),
      mnLastOdomKFId(0),
      mpLastCamKF(NULL)  // added by zzh
{                        // zzh
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  cv::FileNode fnSize = fSettings["LocalMapping.LocalWindowSize"];
  if (fnSize.empty()) {
    mnLocalWindowSize = 0;
    cout << redSTR "No LocalWindowSize, then don't enter VIORBSLAM2 or Odom(Enc/IMU) mode!" << whiteSTR << endl;
  } else {
    mnLocalWindowSize = fnSize;  // notice it can <1
  }
}

void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) { mpLoopCloser = pLoopCloser; }

void LocalMapping::SetTracker(Tracking *pTracker) { mpTracker = pTracker; }

void LocalMapping::Run() {
  mbFinished = false;

  while (1)  // every 3ms until mbFinishRequested==true
  {
    // Tracking will see that Local Mapping is busy
    SetAcceptKeyFrames(false);

    // Check if there are keyframes in the queue
    if (CheckNewKeyFrames()) {
      chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
      // BoW conversion and insertion in Map
      PRINT_DEBUG_INFO_MUTEX("Processing New KF...", imu_tightly_debug_path, "debug.txt");
      ProcessNewKeyFrame();
      PRINT_DEBUG_INFO_MUTEX(mpCurrentKeyFrame->mnId << " Over" << endl, imu_tightly_debug_path, "debug.txt");
      mpIMUInitiator->SetCurrentKeyFrame(mpCurrentKeyFrame);  // zzh
      PRINT_INFO_FILE(blueSTR "Used time in ProcessNewKF()="
                          << chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t0).count()
                          << whiteSTR << endl,
                      imu_tightly_debug_path, "localmapping_thread_debug.txt");

#ifndef NO_LOCALMAP_PROCESS
      // Check recent added MapPoints
      MapPointCulling();
      PRINT_INFO_FILE(blueSTR "Used time in MapCulling()="
                          << chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t0).count()
                          << whiteSTR << endl,
                      imu_tightly_debug_path, "localmapping_thread_debug.txt");

      // Triangulate new MapPoints
      CreateNewMapPoints();
      PRINT_INFO_FILE(blueSTR "Used time in CreateNewMP()="
                          << chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t0).count()
                          << whiteSTR << endl,
                      imu_tightly_debug_path, "localmapping_thread_debug.txt");

      if (!CheckNewKeyFrames())  // if the newKFs list is idle
      {
        // Find more matches in neighbor keyframes and fuse point duplications
        SearchInNeighbors();
        PRINT_INFO_FILE(blueSTR "Used time in SIN()="
                            << chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t0).count()
                            << whiteSTR << endl,
                        imu_tightly_debug_path, "localmapping_thread_debug.txt");
      }
#endif

      mbAbortBA = false;

      if ((!CheckNewKeyFrames()) &&
          !stopRequested())  // if the newKFs list is idle and not requested stop by LoopClosing/localization mode
      {
        // Local BA
        // at least 3 KFs in mpMap, we add Odom condition: 1+1=2 is the threshold of the left
        // &&mpCurrentKeyFrame->mnId>mnLastOdomKFId+1
        if (mpMap->KeyFramesInMap() > 2) {
          chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
          if (!mpIMUInitiator->GetVINSInited()) {
            if (mpCurrentKeyFrame->mnId > mnLastOdomKFId + 1) {
              if (!mpIMUInitiator->GetSensorEnc())
                Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap);  // local BA
              else
                Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap,
                                                 mnLocalWindowSize);  // local BA
            }
          } else {
            // maybe it needs transition when initialized with a few imu edges<N
            const bool bno_imu_lba = false;  // true;  //
            if (!bno_imu_lba) {
              // bLarge/bRecInit ref from ORB3
              const bool bLarge = false;
              //                  mpTracker->Getnum_track_inliers_() > mpTracker->mSensor == System::MONOCULAR ? 75 :
              //                  100;
              const bool bRecInit = false;  //!(mpIMUInitiator->GetInitGBA2() && mpIMUInitiator->GetInitGBAOver());
              Optimizer::LocalBundleAdjustmentNavStatePRV(mpCurrentKeyFrame, mnLocalWindowSize, &mbAbortBA, mpMap,
                                                          mpIMUInitiator->GetGravityVec(), bLarge, bRecInit);
              // Optimizer::LocalBAPRVIDP(mpCurrentKeyFrame,mnLocalWindowSize,&mbAbortBA, mpMap, mGravityVec);
            }
          }
          static double dt_olba_avg = 0;
          static unsigned long num_olba_avg = 0;
          double dt_olba = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t1).count();
          dt_olba_avg += dt_olba;
          ++num_olba_avg;
          PRINT_INFO_FILE(
              blueSTR "Used time in localBA=" << dt_olba << ",avg=" << dt_olba_avg / num_olba_avg << whiteSTR << endl,
              imu_tightly_debug_path, "localmapping_thread_debug.txt");
        }

#ifndef NO_LOCALMAP_PROCESS
        // Check redundant local Keyframes
        KeyFrameCulling();
#endif
      }

#ifndef NO_GBA_THREAD
      mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
#endif
      static double dt_lbathread_avg = 0;
      static unsigned long num_lbathread_avg = 0;
      double dt_lbathread = chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t0).count();
      dt_lbathread_avg += dt_lbathread;
      ++num_lbathread_avg;
      PRINT_INFO_FILE(blueSTR "Used time in localmapping=" << dt_lbathread << ",avg="
                                                           << dt_lbathread_avg / num_lbathread_avg << whiteSTR << endl,
                      imu_tightly_debug_path, "localmapping_thread_debug.txt");
    } else if (Stop()) {
      // Safe area to stop
      while (isStopped() && !CheckFinish())  // maybe stopped for localization mode or LoopClosing thread's correction
      {
        usleep(3000);
      }
      if (CheckFinish())  // is this useless?
        break;
    }

    ResetIfRequested();

    // Tracking will see that Local Mapping is idle/not busy
    SetAcceptKeyFrames(true);

    if (CheckFinish()) break;

    usleep(3000);
  }

  SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexNewKFs);
  mlNewKeyFrames.push_back(pKF);
  mbAbortBA = true;  // stop localBA
}

bool LocalMapping::CheckNewKeyFrames() {
  unique_lock<mutex> lock(mMutexNewKFs);
  return (!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame() {
  {
    unique_lock<mutex> lock(mMutexNewKFs);
    mpCurrentKeyFrame = mlNewKeyFrames.front();
    mlNewKeyFrames.pop_front();
  }

  // added by zzh, it can also be put in InsertKeyFrame()
  PRINT_INFO_FILE("state=" << (int)mpCurrentKeyFrame->getState() << ",tm=" << fixed << setprecision(9)
                           << mpCurrentKeyFrame->mTimeStamp << endl,
                  imu_tightly_debug_path, "localmapping_thread_debug.txt");
  if (mpCurrentKeyFrame->getState() == (char)Tracking::ODOMOK) {
    // 5 is the threshold of Reset() soon after initialization in Tracking, here we will clean these middle state==OK
    // KFs for a better map
    if (mnLastOdomKFId > 0 && mpCurrentKeyFrame->mnId <= mnLastOdomKFId + 5) {
      // one kind of Reset() during the copying KFs' stage in IMU Initialization, don't cull any KF!
      if (mpIMUInitiator->SetCopyInitKFs(true)) {
        KeyFrame *pLastKF = mpCurrentKeyFrame;
        vector<KeyFrame *> vecEraseKF;
        if (mpIMUInitiator->GetVINSInited()) {
          PRINT_INFO_MUTEX("KF->SetBadFlag() in ProcessNewKeyFrame()!" << endl);
          double tmNewest = pLastKF->mTimeStamp;
          bool bLastCamKF = false;
          char state;
          do {
            pLastKF = pLastKF->GetPrevKeyFrame();
            state = pLastKF->getState();
            if (tmNewest - pLastKF->GetPrevKeyFrame()->mTimeStamp > 0.5) {
              tmNewest = pLastKF->mTimeStamp;
              if (!bLastCamKF && state == (char)Tracking::OK) {
                mpLastCamKF = pLastKF;
                bLastCamKF = true;  // pLastKF->getState() is OK
              }
            } else
              vecEraseKF.push_back(pLastKF);  // keep tmNewest unchanged
          } while (state != (char)Tracking::ODOMOK);
          if (!bLastCamKF) {
            do {
              pLastKF = pLastKF->GetPrevKeyFrame();
            } while (pLastKF->getState() != (char)Tracking::OK);
            mpLastCamKF = pLastKF;
          }
        } else {
          int count = mpCurrentKeyFrame->mnId - mnLastOdomKFId;
          do {
            pLastKF = pLastKF->GetPrevKeyFrame();
            vecEraseKF.push_back(pLastKF);
          } while (--count > 0);
          assert(pLastKF != NULL && pLastKF->getState() == (char)Tracking::ODOMOK);
          do {
            pLastKF = pLastKF->GetPrevKeyFrame();  // we cannot directly use this for consecutive ODOMOK KFs may happen
                                                   // when LoopClosing lock it by SetNotErase()
          } while (pLastKF->getState() != (char)Tracking::OK);
          mpLastCamKF = pLastKF;
        }
        PRINT_INFO_MUTEX(vecEraseKF.size() << " ");
        // the last one is the before ODOMOK(delete the former consecutive OdomOK KF as soon as possible, it seems to
        // have a better effect)
        for (int i = 0; i < vecEraseKF.size(); ++i) {
          PRINT_INFO_MUTEX(i << " ");
          vecEraseKF[i]->SetBadFlag();  // it may be SetNotErase() by LoopClosing thread
        }
        PRINT_INFO_MUTEX("Over" << endl);
        // 	  if (pLastKF!=NULL&&pLastKF->getState()==Tracking::ODOMOK){//&&pLastKF->GetParent()!=NULL
        assert(mpLastCamKF != NULL && mpLastCamKF->getState() == (char)Tracking::OK);
        mpIMUInitiator->SetCopyInitKFs(false);
      }
    }
    mnLastOdomKFId = mpCurrentKeyFrame->mnId;
  } else {  // OK
    mpLastCamKF = mpCurrentKeyFrame;
  }

  // Compute Bags of Words structures, maybe already computed by TrackReferenceKeyFrame()
  mpCurrentKeyFrame->ComputeBoW();

  // Associate MapPoints to the new keyframe and update normal and descriptor
  const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

  for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
    MapPoint *pMP = vpMapPointMatches[i];
    if (pMP) {
      if (!pMP->isBad()) {
        // to solve the problem 2features in the same frame could see the same mp, caused by replace op.
        if (!pMP->IsInKeyFrame(mpCurrentKeyFrame, i))  // when this MP is not created by mpCurrentKeyFrame
        {
          // the only pMP->AddObservation() except new MP() && LoopClosing, it means pMP->mObservations/covisibility
          // graph only have local KFs' info and no loop KFs' info
          pMP->AddObservation(mpCurrentKeyFrame, i);
          pMP->UpdateNormalAndDepth();
          pMP->ComputeDistinctiveDescriptors();
        } else  // this can only happen for new stereo points inserted by the Tracking
        {
          mlpRecentAddedMapPoints.push_back(pMP);
        }
      }
    }
  }
  // Update links in the Covisibility Graph
  mpCurrentKeyFrame->UpdateConnections(mpLastCamKF);
  //     mpCurrentKeyFrame->UpdateConnections();

  // Insert Keyframe in Map
  mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling() {
  // Check Recent Added MapPoints
  list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
  const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

  int nThObs;
  if (mbMonocular)
    nThObs = 2;
  else
    nThObs = 3;
  const int cnThObs = nThObs;

  while (lit != mlpRecentAddedMapPoints.end()) {
    MapPoint *pMP = *lit;
    if (pMP->isBad()) {
      lit = mlpRecentAddedMapPoints.erase(lit);  // already bad MPs don't need to be culled any more
    } else if (pMP->GetFoundRatio() <
               0.25f)  // if only 1/4 visibles(matched by TrackWithMM()/TrackRefKF()/mCurrentFrame.isInFrustum(local
                       // MPs)) can be regarded as inliers(found)
    {
      pMP->SetBadFlag();
      lit = mlpRecentAddedMapPoints.erase(lit);
    } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 &&
               pMP->Observations() <= cnThObs)  // long age(>=2 frames) new unimportant(<=3 monocular KFs observation)
                                                // MapPoints are discarded as bad ones
    {
      pMP->SetBadFlag();  // firstly cannot be consecutively(3KFs) observed far MPs will be deleted
      lit = mlpRecentAddedMapPoints.erase(lit);
    } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >=
               3)  // too long ago(>=3 frames) new MapPoints are not processed/culled any more
      lit = mlpRecentAddedMapPoints.erase(lit);
    else
      lit++;
  }
}

static inline void PrepareDataForTraingulate(const vector<GeometricCamera *> &pcams_in, KeyFrame *pKF1,
                                             const vector<size_t> &idxs1, vector<GeometricCamera *> &pcams,
                                             aligned_vector<Sophus::SE3d> &Twrs, aligned_vector<Eigen::Vector2d> &kps2d,
                                             vector<cv::KeyPoint> &kps, vector<float> &sigma_lvs,
                                             vector<vector<float>> &urbfs, bool &bStereos, float &cosdisparity) {
  bStereos = false;
  cosdisparity = 1.1;  // >1 designed for future inifity point
  pcams.clear();
  Twrs.clear();
  kps.clear();
  sigma_lvs.clear();
  urbfs.clear();
  CV_Assert(pcams_in.size() == idxs1.size());
  bool usedistort = Frame::usedistort_ && pKF1->mpCameras.size();
  for (size_t ididxs = 0; ididxs < idxs1.size(); ++ididxs) {
    size_t idx = idxs1[ididxs];
    if (-1 != idx) {
      size_t cami = pKF1->mapn2in_.size() <= idx ? 0 : get<0>(pKF1->mapn2in_[idx]);
      pcams.push_back(pcams_in[cami]);
      Twrs.push_back(pKF1->GetTwc() * pKF1->GetTcr());
      const auto &kp = (!usedistort) ? pKF1->mvKeysUn[idx] : pKF1->mvKeys[idx];
      kps.push_back(kp);
      kps2d.push_back(Eigen::Vector2d(kp.pt.x, kp.pt.y));
      sigma_lvs.push_back(pKF1->mvLevelSigma2[kp.octave]);
      urbfs.push_back(vector<float>({pKF1->mvuRight[idx], pKF1->mbf}));
      // TODO: record cosdisparity in Frame.cc for StereoDistort one
      if (!bStereos) {
        if (0 <= urbfs.back()[0]) bStereos = true;
      } else
        CV_Assert(0 <= urbfs.back()[0]);
      if (bStereos) {
        // this cos value is the min stereo parallax value, (the point with certain depth has max stereo parallax angle
        // when its Xc is at the centre of baseline), here stereo parallax!=Rays parallax
        const float cosParallaxRays = cos(2 * atan2(pKF1->mb / 2., pKF1->mvDepth[idx]));
        if (cosdisparity > cosParallaxRays) cosdisparity = cosParallaxRays;
      }
    }
  }
}
static inline bool PrepareDatasForTraingulate(const vector<GeometricCamera *> *pcams_in, const vector<KeyFrame *> &pKFs,
                                              const vector<vector<size_t>> &idxs, vector<GeometricCamera *> &pcams,
                                              aligned_vector<Sophus::SE3d> &Twrs,
                                              aligned_vector<Eigen::Vector2d> &kps2d, vector<cv::KeyPoint> *kps,
                                              vector<float> &sigma_lvs, vector<vector<float>> &urbfs, bool *bStereos,
                                              float &cosdisparity, float *cosdisparities) {
  cosdisparity = 1.1;
  vector<GeometricCamera *> vpcams[2];
  aligned_vector<Sophus::SE3d> vTwrs[2];
  aligned_vector<Eigen::Vector2d> vkps[2];
  vector<float> vsigma_lvs[2];
  vector<vector<float>> vurbfs[2];
  for (int i = 0; i < 2; ++i)
    PrepareDataForTraingulate(pcams_in[i], pKFs[i], idxs[i], vpcams[i], vTwrs[i], vkps[i], kps[i], vsigma_lvs[i],
                              vurbfs[i], bStereos[i], cosdisparities[i]);
  if (!vpcams[0].size() || !vpcams[1].size()) return false;
  Eigen::Matrix3d Rwc[2] = {Converter::toMatrix3d(pKFs[0]->GetRotation().t()),
                            Converter::toMatrix3d(pKFs[1]->GetRotation().t())};
  for (size_t i1 = 0; i1 < vpcams[0].size(); ++i1) {
    auto &pcam1 = vpcams[0][i1];
    auto xn1 = pcam1->GetTrc() * pcam1->unproject(vkps[0][i1]);
    auto ray1 = Rwc[0] * xn1;
    for (size_t i2 = 0; i2 < vpcams[1].size(); ++i2) {
      auto &pcam2 = vpcams[1][i2];
      auto xn2 = pcam2->GetTrc() * pcam2->unproject(vkps[1][i2]);
      auto ray2 = Rwc[1] * xn2;
      // the Rays parallax angle must be in [0,180) for depth >0 (TODO: when angle >= 180, rectify here)
      const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());
      if (cosdisparity > cosParallaxRays) cosdisparity = cosParallaxRays;
    }
  }
  pcams = std::move(vpcams[0]);
  pcams.insert(pcams.end(), vpcams[1].begin(), vpcams[1].end());
  Twrs = std::move(vTwrs[0]);
  Twrs.insert(Twrs.end(), vTwrs[1].begin(), vTwrs[1].end());
  kps2d = std::move(vkps[0]);
  kps2d.insert(kps2d.end(), vkps[1].begin(), vkps[1].end());
  sigma_lvs = std::move(vsigma_lvs[0]);
  sigma_lvs.insert(sigma_lvs.end(), vsigma_lvs[1].begin(), vsigma_lvs[1].end());
  urbfs = std::move(vurbfs[0]);
  urbfs.insert(urbfs.end(), vurbfs[1].begin(), vurbfs[1].end());
  return true;
}
void LocalMapping::CreateNewMapPoints() {
  // Retrieve neighbor keyframes in covisibility graph
  int nn = 10;
  if (mbMonocular) nn = 20;
  vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

  // ref from ORB3
  // if (mpIMUInitiator->GetSensorIMU()) {  // || mpIMUInitiator->GetSensorEnc()) {
  if (mpIMUInitiator->GetVINSInited()) {
    KeyFrame *pKF = mpCurrentKeyFrame;
    int count = 0;
    while ((vpNeighKFs.size() <= nn) && (pKF->GetPrevKeyFrame()) && (count++ < nn)) {
      vector<KeyFrame *>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->GetPrevKeyFrame());
      if (it == vpNeighKFs.end()) vpNeighKFs.push_back(pKF->GetPrevKeyFrame());
      pKF = pKF->GetPrevKeyFrame();
    }
  }

  ORBmatcher matcher(0.6, false);

  cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
  cv::Mat Rwc1 = Rcw1.t();
  cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
  cv::Mat Tcw1(3, 4, CV_32F);
  Rcw1.copyTo(Tcw1.colRange(0, 3));
  tcw1.copyTo(Tcw1.col(3));
  cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

  const float &fx1 = mpCurrentKeyFrame->fx;
  const float &fy1 = mpCurrentKeyFrame->fy;
  const float &cx1 = mpCurrentKeyFrame->cx;
  const float &cy1 = mpCurrentKeyFrame->cy;
  const float &invfx1 = mpCurrentKeyFrame->invfx;
  const float &invfy1 = mpCurrentKeyFrame->invfy;

  const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;  // 1.5*1.2=1.8

  int nnew = 0;  // unused here

  // Search matches with epipolar restriction and triangulate
  for (size_t i = 0; i < vpNeighKFs.size(); i++) {
    if (i > 0 && CheckNewKeyFrames())  // if it's busy then just triangulate the best covisible KF
      return;

    KeyFrame *pKF2 = vpNeighKFs[i];
    KeyFrame *&pKF1 = mpCurrentKeyFrame;

    // Check first that baseline is not too short
    cv::Mat Ow2 = pKF2->GetCameraCenter();
    cv::Mat vBaseline = Ow2 - Ow1;
    const float baseline = cv::norm(vBaseline);

    if (!mbMonocular) {
      if (baseline < pKF2->mb)  // for RGBD, if moved distance < mb(equivalent baseline), it's not wise to process maybe
                                // for it cannot see farther than depth camera
        continue;
    } else {
      const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
      const float ratioBaselineDepth = baseline / medianDepthKF2;

      if (ratioBaselineDepth < 0.01)  // at least baseline>=0.08m/8m(medianDepth)
        continue;
    }

    // Search matches that fullfil epipolar constraint(with 2 sigma rule)
    vector<vector<vector<size_t>>> vMatchedIndices;
    matcher.SearchForTriangulation(pKF1, pKF2, vMatchedIndices, false);  // matching method is like SBBoW

    shared_ptr<Pinhole> pcaminst[2];
    vector<GeometricCamera *> pcams_in[2];
    bool usedistort[2] = {pKF1->mpCameras.size() && Frame::usedistort_, pKF2->mpCameras.size() && Frame::usedistort_};
    if (!usedistort[0]) {
      CV_Assert(!usedistort[1]);
      pcaminst[0] = make_shared<Pinhole>(vector<float>({pKF1->fx, pKF1->fy, pKF1->cx, pKF1->cy}));
      pcaminst[1] = make_shared<Pinhole>(vector<float>({pKF2->fx, pKF2->fy, pKF2->cx, pKF2->cy}));
      pcams_in[0].push_back(pcaminst[0].get());
      pcams_in[1].push_back(pcaminst[1].get());
    } else {
      CV_Assert(usedistort[1]);
      pcams_in[0] = pKF1->mpCameras;
      pcams_in[1] = pKF2->mpCameras;
    }

    // Triangulate each match
    const int nmatches = vMatchedIndices.size();
    for (int ikp = 0; ikp < nmatches; ikp++) {
      const auto &idxs1 = vMatchedIndices[ikp][0];
      const auto &idxs2 = vMatchedIndices[ikp][1];

      vector<GeometricCamera *> pcams;
      aligned_vector<Sophus::SE3d> Twrs;
      vector<cv::KeyPoint> kps[2];
      aligned_vector<Eigen::Vector2d> kps2d;
      vector<float> sigma_lvs;
      vector<vector<float>> urbfs;
      bool bStereos[2];
      float cosParallaxRays;
      //+1 && cosParallaxRays>0 -> always choosing stereo parallax(if exists) cos value as the cosParallaxStereo
      float cosParallaxStereo = cosParallaxRays + 1;
      float cosParallaxStereos[2];
      if (!PrepareDatasForTraingulate(pcams_in, vector<KeyFrame *>{pKF1, pKF2}, vector<vector<size_t>>{idxs1, idxs2},
                                      pcams, Twrs, kps2d, kps, sigma_lvs, urbfs, bStereos, cosParallaxRays,
                                      cosParallaxStereos))
        continue;
      // cout << "check pcams.size="<<pcams.size()<<endl;

      cosParallaxStereo = min(cosParallaxStereos[0], cosParallaxStereos[1]);

      // use triangulation method when it's 2 monocular points with enough parallax or at least 1 stereo point with less
      // accuracy in depth data
      cv::Mat x3D;
      // if >=1 stereo point -> if Rays parallax angle is >= angleParallaxStereo1(!bStereo1->2)(will get better x3D
      // result) && its Rays parallax angle <90 degrees(over will make 1st condition some problem && make feature
      // matching unreliable?) if both monocular then parallax angle must be in [1.15,90) degrees
      if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
          (bStereos[0] || bStereos[1] || cosParallaxRays < 0.9998)) {
        const double thresh_cosdisparity = 1. - 1e-6;
        if (pcams[0]->TriangulateMatches(pcams, kps2d, sigma_lvs, &x3D, thresh_cosdisparity, &urbfs, &Twrs).empty())
          continue;
      } else if (cosParallaxStereos[0] < cosParallaxStereos[1]) {
        CV_Assert(bStereos[0]);
        x3D = pKF1->UnprojectStereo(idxs1.front());
        if (pcams[0]->TriangulateMatches(pcams, kps2d, sigma_lvs, &x3D, 1., &urbfs, &Twrs, true).empty()) continue;
      } else if (cosParallaxStereos[1] < cosParallaxStereos[0]) {
        CV_Assert(bStereos[1]);
        x3D = pKF2->UnprojectStereo(idxs2.front());
        if (pcams[0]->TriangulateMatches(pcams, kps2d, sigma_lvs, &x3D, 1., &urbfs, &Twrs, true).empty()) continue;
      } else
        continue;  // No stereo and very low(or >=90 degrees) parallax, but here sometimes may introduce Rays parallax
                   // angle>=90 degrees with >=1 stereo point

      // Check scale consistency, is this dist not depth very good?
      cv::Mat normal1 = x3D - Ow1;
      float dist1 = cv::norm(normal1);
      cv::Mat normal2 = x3D - Ow2;
      float dist2 = cv::norm(normal2);
      if (dist1 == 0 || dist2 == 0)  // it seems impossible for zi>0, if possible it maybe numerical error
        continue;
      const float ratioDist = dist2 / dist1;
      float ratioOctave[2] = {INFINITY, -INFINITY};
      for (auto kp1 : kps[0]) {
        for (auto kp2 : kps[1]) {
          // here FIX ORBSLAM2/3 bug! if dist2 larger, scale2 and ratioOctave should also be larger
          float rat_tmp = pKF2->mvScaleFactors[kp2.octave] / pKF1->mvScaleFactors[kp1.octave];
          if (rat_tmp < ratioOctave[0]) ratioOctave[0] = rat_tmp;
          if (rat_tmp > ratioOctave[1]) ratioOctave[1] = rat_tmp;
        }
      }
      /*if(fabs(ratioDist-ratioOctave)>ratioFactor) continue;*/
      // ratioOctave must be in [ratioDist/ratioFactor,ratioDist*ratioFactor], notice ratioFactor
      // is 1.5*mpCurrentKeyFrame->mfScaleFactor
      if (ratioDist * ratioFactor < ratioOctave[1] || ratioDist > ratioOctave[0] * ratioFactor) {
        //        cout << "check ratio="<<ratioDist <<","<<"octave="<<ratioOctave[0]<<","<<ratioOctave[1]<<endl;
        continue;
      }

      // Triangulation is succesfull
      MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);  // notice pMp->mnFirstKFid=mpCurrentKeyFrame->mnID

      PRINT_DEBUG_INFO_MUTEX("addmp1" << endl, imu_tightly_debug_path, "debug.txt");
      for (auto idx : idxs1) {
        if (-1 == idx) continue;
        pMP->AddObservation(pKF1, idx);
        pKF1->AddMapPoint(pMP, idx);
      }
      for (auto idx : idxs2) {
        if (-1 == idx) continue;
        pMP->AddObservation(pKF2, idx);
        pKF2->AddMapPoint(pMP, idx);
      }
      pMP->ComputeDistinctiveDescriptors();
      pMP->UpdateNormalAndDepth();

      mpMap->AddMapPoint(pMP);
      mlpRecentAddedMapPoints.push_back(pMP);
      nnew++;
    }
  }
  // cout << "newmpsnum=" << nnew << endl;
}

void LocalMapping::SearchInNeighbors() {
  // Retrieve neighbor keyframes
  int nn = 10;  // RGBD
  if (mbMonocular) nn = 20;
  const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
  vector<KeyFrame *> vpTargetKFs;
  for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++) {
    KeyFrame *pKFi = *vit;
    if (pKFi->isBad() ||
        pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)  // bad or entered(avoid duplications), cannot be itself
      continue;
    vpTargetKFs.push_back(pKFi);
    pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
  }

  // Add some covisible of covisible
  // Extend to some second neighbors
  for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++) {
    const vector<KeyFrame *> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(5);  // ORB3 uses 20
    for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end();
         vit2 != vend2; vit2++) {
      KeyFrame *pKFi2 = *vit2;
      // avoid bad,duplications && itself(KF now)
      if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
          pKFi2->mnId == mpCurrentKeyFrame->mnId)
        continue;
      pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;  // fixed efficiency bug in ORB2
      vpTargetKFs.push_back(pKFi2);
    }
//#define ORB3_STRATEGY
#ifdef ORB3_STRATEGY
    if (mbAbortBA) return;
#endif
  }

  // ref from ORB3
  // Extend to temporal neighbors
  // if (mpIMUInitiator->GetSensorIMU()) {  // || mpIMUInitiator->GetSensorEnc()) {
  if (mpIMUInitiator->GetVINSInited()) {
    KeyFrame *pKFi = mpCurrentKeyFrame->GetPrevKeyFrame();
    while (vpTargetKFs.size() < 20 && pKFi) {
      if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId) {
        pKFi = pKFi->GetPrevKeyFrame();
        continue;
      }
      vpTargetKFs.push_back(pKFi);
      pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
      pKFi = pKFi->GetPrevKeyFrame();
    }
  }

  // bijection search matches
  //  Search matches by projection from current KF in target KFs
  ORBmatcher matcher;  // 0.6,true
  vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
  size_t num_fused = 0;
  for (vector<KeyFrame *>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++) {
    KeyFrame *pKFi = *vit;
    num_fused = matcher.Fuse(pKFi, vpMapPointMatches);
  }
  PRINT_DEBUG_INFO("over2 fused num = " << num_fused << endl, imu_tightly_debug_path, "localmapping_thread_debug.txt");

#ifdef ORB3_STRATEGY
  if (mbAbortBA) return;
#endif

  // Search matches by projection from target KFs in current KF
  vector<MapPoint *> vpFuseCandidates;
  vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

  for (vector<KeyFrame *>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF; vitKF++) {
    KeyFrame *pKFi = *vitKF;

    vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

    for (vector<MapPoint *>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end(); vitMP != vendMP;
         vitMP++) {
      MapPoint *pMP = *vitMP;
      if (!pMP)  // avoid no corresponding/empty MapPoints
        continue;
      if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)  // avoid bad,duplications
        continue;
      pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
      vpFuseCandidates.push_back(pMP);
    }
  }

  num_fused = matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);
  PRINT_DEBUG_INFO("over3, fused2= " << num_fused << endl, imu_tightly_debug_path, "localmapping_thread_debug.txt");

  // Update MapPoints' descriptor&&normal in mpCurrentKeyFrame
  vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
  size_t num_pts_good = 0;
  for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++) {
    MapPoint *pMP = vpMapPointMatches[i];
    if (pMP) {
      if (!pMP->isBad()) {
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
        ++num_pts_good;
      }
    }
  }
  PRINT_DEBUG_INFO(
      "curkf good pts num= " << num_pts_good << ":" << (float)num_pts_good / vpMapPointMatches.size() << endl,
      imu_tightly_debug_path, "localmapping_thread_debug.txt");

  // Update connections in covisibility graph, for possible changed MapPoints in fuse by projection from target KFs
  // incurrent KF
  mpCurrentKeyFrame->UpdateConnections();
}

void LocalMapping::RequestStop() {
  unique_lock<mutex> lock(mMutexStop);
  mbStopRequested = true;
  unique_lock<mutex> lock2(mMutexNewKFs);
  mbAbortBA = true;
}

bool LocalMapping::Stop() {
  unique_lock<mutex> lock(mMutexStop);
  if (mbStopRequested && !mbNotStop) {
    mbStopped = true;
    PRINT_INFO_MUTEX("Local Mapping STOP"
                     << endl);  // if LocalMapping is stopped for CorrectLoop()/GBA, this word should appear!
    return true;
  }

  return false;
}

bool LocalMapping::isStopped() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopped;
}

bool LocalMapping::stopRequested() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopRequested;
}

void LocalMapping::Release() {
  unique_lock<mutex> lock(mMutexStop);
  unique_lock<mutex> lock2(mMutexFinish);
  if (mbFinished) return;
  mbStopped = false;
  mbStopRequested = false;
  // we don't need to lock mMutexNewKFs for stopRequested() && Stop() will be called before calling release()
  for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
    delete *lit;  //for it's originally in localization mode, so these KFs should be delete when it's deactivated without regarding the mpMap/mlpRecentAddedMapPoints, \
    if its original state is stopped by LoopClosing, mlNewKeyFrames is already empty
  mlNewKeyFrames.clear();

  PRINT_INFO_MUTEX("Local Mapping RELEASE"
                   << endl);  // if LocalMapping is recovered from CorrectLoop()/GBA, this notice should appear!
}

bool LocalMapping::AcceptKeyFrames() {
  unique_lock<mutex> lock(mMutexAccept);
  return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag) {
  unique_lock<mutex> lock(mMutexAccept);
  mbAcceptKeyFrames = flag;
}

bool LocalMapping::SetNotStop(bool flag) {
  unique_lock<mutex> lock(mMutexStop);

  if (flag && mbStopped) return false;

  mbNotStop = flag;

  return true;
}

void LocalMapping::InterruptBA() { mbAbortBA = true; }

void LocalMapping::KeyFrameCulling() {
  // during the copying KFs' stage in IMU Initialization, don't cull any KF!
  if (!mpIMUInitiator->SetCopyInitKFs(true)) return;

  // Check redundant keyframes (only local keyframes)
  // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
  // in at least other 3 keyframes (in the same or finer scale)
  // We only consider close stereo points
  // get all 1st layer covisibility KFs as localKFs, notice no mpCurrentKeyFrame
  vector<KeyFrame *> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

  // get last Nth KF or the front KF of the local window
  KeyFrame *pLastNthKF = mpCurrentKeyFrame;
  double tmNthKF = -1;  // pLastNthKF==NULL then -1
  vector<bool> vbEntered;
  int nRestrict = 1;                                 // for not VIO mode
  bool bSensorIMU = mpIMUInitiator->GetSensorIMU();  // false;
  if (mnLocalWindowSize < 1 && mpIMUInitiator->GetVINSInited())
    bSensorIMU = false;  // for pure-vision+IMU Initialization mode!
  if (bSensorIMU) {
    int Nlocal = mnLocalWindowSize;
    while (--Nlocal > 0 && pLastNthKF != NULL) {  // maybe less than N KFs in pMap
      pLastNthKF = pLastNthKF->GetPrevKeyFrame();
    }
    if (pLastNthKF != NULL) tmNthKF = pLastNthKF->mTimeStamp;  // N starts from 1 & notice mTimeStamp>=0

    vbEntered.resize(vpLocalKeyFrames.size(), false);
    if (mpIMUInitiator->GetVINSInited()) {
      // notice when during IMU Initialization: we use all KFs' timespan restriction of 0.5s like JW, for MH04 has
      // problem with 0.5/3s strategy!
      nRestrict = 2;
    }
  }

  // k==0 for strict restriction then k==1 do loose restriction only for outer LocalWindow KFs
  for (int k = 0; k < nRestrict; ++k) {
    int vi = 0;
    PRINT_INFO_MUTEX("LocalKFs:" << vpLocalKeyFrames.size() << endl);
    for (vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend;
         ++vit, ++vi) {
      KeyFrame *pKF = *vit;
      // pKF is bad check for loop closing thread setnoterase and check can speed up
      if (pKF->mnId == 0 || pKF->isBad()) continue;  // cannot erase the initial KF

      // timespan restriction is implemented as the VIORBSLAM paper III-B
      double tmNext = -1;
      if (k == 0) {
        if (bSensorIMU) {  // restriction is only for VIO
          assert(pKF != NULL);
          // assert(pKF->GetPrevKeyFrame()!=NULL);
          // solved old bug: for there exists unidirectional edge in covisibility graph, so a bad KF may still exist in
          // other's connectedKFs
          if (pKF->GetPrevKeyFrame() == NULL) {
            int bkfbad = (int)pKF->isBad();
            PRINT_INFO_MUTEX(pKF->mnId << " " << bkfbad << endl);
            vbEntered[vi] = true;
            continue;
          }
          tmNext = pKF->GetNextKeyFrame()->mTimeStamp;
          if (tmNext - pKF->GetPrevKeyFrame()->mTimeStamp > 0.5)
            continue;
          else
            vbEntered[vi] = true;

          // if
          // (pKF==pLastNthKF||pLastNthKF!=NULL&&pKF==pLastNthKF->GetNextKeyFrame()||pKF->GetNextKeyFrame()==mpCurrentKeyFrame)
          // {vbEntered[vi]=true;continue;}
        }
      } else {  // loose restriction when k==1
        if (vbEntered[vi]) continue;
        assert(pKF != NULL && pKF->GetPrevKeyFrame() != NULL);
        tmNext = pKF->GetNextKeyFrame()->mTimeStamp;
        // this KF is in next time's local window or N+1th
        if (tmNext > tmNthKF || tmNext - pKF->GetPrevKeyFrame()->mTimeStamp > 3)
          continue;  // normal restriction to perform full BA
      }

      // cannot erase last ODOMOK & first ODOMOK's parent!
      KeyFrame *pNextKF = pKF->GetNextKeyFrame();
      if (pNextKF == NULL) {
        int bkfbad = (int)pKF->isBad();
        PRINT_INFO_MUTEX("NoticeNextKF==NULL: " << pKF->mnId << " " << bkfbad << endl);
        continue;
      }
      // solved old bug
      //  for simple(but a bit wrong) Map Reuse, we avoid segmentation fault for the last KF of the
      //  loaded map
      // if (pNextKF!=NULL){
      if (pNextKF->getState() == Tracking::ODOMOK) {
        // 2 consecutive ODOMOK KFs then delete the former one for a better quality map
        if (pKF->getState() == Tracking::ODOMOK) {
          // this KF in next time's local window or N+1th & its prev-next<=0.5 then we should move tmNthKF forward 1 KF
          if (tmNext > tmNthKF && pLastNthKF != NULL) {
            pLastNthKF = pLastNthKF->GetPrevKeyFrame();
            tmNthKF = pLastNthKF == NULL ? -1 : pLastNthKF->mTimeStamp;
          }  // must done before pKF->SetBadFlag()!
          PRINT_INFO_MUTEX(greenSTR << "OdomKF->SetBadFlag()!" << whiteSTR << endl);
          pKF->SetBadFlag();
        }  // else next is OK then continue
        continue;
      } else {
        // next KF is OK(we keep at least 1 ODOMOK between OK KFs, maybe u can use it for a better PoseGraph
        // Optimization?)
        if (pKF->getState() == Tracking::ODOMOK) continue;
      }
      // }

      const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();
      int nObs = 3;
      const int thObs = nObs;  // can directly use const 3
      int nRedundantObservations =
          0;         // the number of redundant(seen also by at least 3 other KFs) close stereo MPs seen by pKF
      int nMPs = 0;  // the number of close stereo MPs seen by pKF
      for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
        MapPoint *pMP = vpMapPoints[i];
        if (pMP && !pMP->isBad()) {
          // if RGBD/Stereo
          if (!mbMonocular) {
            // only consider close stereo points(exclude far or monocular points)
            if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0) continue;
          }

          nMPs++;
          // at least here 3 observations(3 monocular KFs, 1 stereo KF+1 stereo/monocular KF), or cannot satisfy that at
          // least other 3 KFs have seen 90% MPs
          if (pMP->Observations() > thObs) {
            const int &scaleLevel = pKF->mvKeys[i].octave;  // Un
            const map<KeyFrame *, set<size_t>> observations = pMP->GetObservations();
            int nObs = 0;
            for (map<KeyFrame *, set<size_t>>::const_iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
              KeyFrame *pKFi = mit->first;
              //"other"
              if (pKFi == pKF) continue;
              auto idxs = mit->second;
              int scaleLeveli = INT_MAX;
              for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
                auto idx = *iter;
                // Un
                if (scaleLeveli > pKFi->mvKeys[idx].octave) {
                  scaleLeveli = pKFi->mvKeys[idx].octave;
                }
              }

              if (scaleLeveli <= scaleLevel + 1)  //"in the same(+1 for error) or finer scale"
              {
                nObs++;
                if (nObs >= thObs) break;
              }
            }
            if (nObs >= thObs)  // if the number of same/better observation KFs >= 3(here)
            {
              nRedundantObservations++;
            }
          }
        }
      }

      if (nRedundantObservations > 0.9 * nMPs) {
        if (tmNext > tmNthKF && pLastNthKF != NULL) {  // this KF in next time's local window or N+1th & its
                                                       // prev-next<=0.5 then we should move tmNthKF forward 1 KF
          pLastNthKF = pLastNthKF->GetPrevKeyFrame();
          tmNthKF = pLastNthKF == NULL ? -1 : pLastNthKF->mTimeStamp;
        }  // must done before pKF->SetBadFlag()!

        PRINT_INFO_MUTEX(pKF->mnId << "badflag" << endl);
        PRINT_DEBUG_INFO("badflag kfid=" << pKF->mnId << ",tm=" << fixed << setprecision(9) << pKF->timestamp_ << ":"
                                         << (float)nRedundantObservations / nMPs << "," << tmNthKF << endl,
                         imu_tightly_debug_path, "localmapping_thread_debug.txt");
        pKF->SetBadFlag();
      }
    }
  }

  mpIMUInitiator->SetCopyInitKFs(false);
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v) {
  return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1), v.at<float>(2), 0, -v.at<float>(0),
          -v.at<float>(1), v.at<float>(0), 0);
}

void LocalMapping::RequestReset() {
  {
    unique_lock<mutex> lock(mMutexReset);
    mbResetRequested = true;
  }

  while (1) {
    {
      unique_lock<mutex> lock2(mMutexReset);
      if (!mbResetRequested) break;
    }
    usleep(3000);
  }
}

void LocalMapping::ResetIfRequested() {
  unique_lock<mutex> lock(mMutexReset);
  if (mbResetRequested) {
    mlNewKeyFrames.clear();
    mlpRecentAddedMapPoints.clear();
    mbResetRequested = false;

    mnLastOdomKFId = 0;
    mpLastCamKF = NULL;  // added by zzh
  }
}

void LocalMapping::RequestFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

bool LocalMapping::CheckFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

void LocalMapping::SetFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinished = true;
  unique_lock<mutex> lock2(mMutexStop);
  mbStopped = true;
}

bool LocalMapping::isFinished() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinished;
}

}  // namespace VIEO_SLAM
