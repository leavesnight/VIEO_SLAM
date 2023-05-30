/**
 * This file is part of VIEO_SLAM
 */

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <stdint-gcc.h>

#include "Pinhole.h"
#include "Converter.h"
#include "common/mlog/log.h"
#include "common/config.h"

using namespace std;

namespace VIEO_SLAM {

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri) : mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

void ORBmatcher::SearchByProjectionBase(const vector<MapPoint *> &vpMapPoints1, cv::Mat Rcrw, cv::Mat tcrw,
                                        KeyFrame *pKF, const float th_radius, const float th_bestdist,
                                        bool bCheckViewingAngle, const float *pbf, int *pnfused, char mode,
                                        vector<vector<bool>> *pvbAlreadyMatched1, vector<set<int>> *pvnMatch1) {
  bool usedistort = pKF->mpCameras.size() && Frame::usedistort_;
  bool only1match = !(SBPMatchMultiCam & mode), fuselater = SBPFuseLater & mode;
  CV_Assert(!fuselater || pvnMatch1);
  PRINT_DEBUG_INFO_MUTEX("SBPB" << (int)fuselater << (int)only1match << endl, mlog::vieo_slam_debug_path, "debug.txt");
  size_t N1 = vpMapPoints1.size();
  if (pvnMatch1) {
    pvnMatch1->clear();
    pvnMatch1->resize(N1, set<int>());
  }
  cv::Mat twcr = pKF->GetCameraCenter();
  // Transform from KF1 to KF2 and search
  for (int i1 = 0; i1 < N1; i1++) {
    MapPoint *pMP = vpMapPoints1[i1];

    if (!pMP || pMP->isBad()) continue;
#define ORB3_STRATEGY
#ifdef ORB3_STRATEGY
    if (!fuselater && pMP->IsInKeyFrame(pKF)) continue;
#endif

    // Get 3D Coords.
    cv::Mat p3Dw = pMP->GetWorldPos();
    cv::Mat Pn = pMP->GetNormal();

    cv::Mat Pcr = Rcrw * p3Dw + tcrw;
    size_t n_cams = !pKF->mpCameras.size() ? 1 : pKF->mpCameras.size();
    // wP could only correspond to pKF's one MP, but maybe n_cams features
    int bestDistInCams = INT_MAX;
    int bestIdxInCams = -1;
    for (size_t cami = 0; cami < n_cams; ++cami) {
      // if pKF1->mvpMapPoints[i1] exists and hasn't been matched to pKF2 before(in vpMatches12) then go on
      if (pvbAlreadyMatched1 && (*pvbAlreadyMatched1)[i1][cami]) continue;
#ifndef ORB3_STRATEGY
      if (!fuselater && pMP->IsInKeyFrame(pKF, -1, cami)) continue;
#endif

      Vector3d Pc = Converter::toVector3d(Pcr);
      GeometricCamera *pcam1 = nullptr;
      cv::Mat twc = twcr.clone();  // wO
      if (pKF->mpCameras.size() > cami) {
        pcam1 = pKF->mpCameras[cami];
        Pc = pcam1->GetTcr() * Pc;
        twc += Rcrw.t() * pcam1->Getcvtrc();
      }
      // Depth must be positive
      if (Pc(2) <= 0.0)  //== rectified by zzh
        continue;

      // Project into Image
      const float invz = 1 / Pc(2);
      float u, v;
      if (!usedistort) {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;
        u = fx * Pc(0) * invz + cx;
        v = fy * Pc(1) * invz + cy;
      } else {
        CV_Assert(pcam1);
        auto pt = pcam1->project(Pc);
        u = pt[0];
        v = pt[1];
      }

      // Point must be inside the image
      if (!pKF->IsInImage(u, v)) continue;

      cv::Mat PO = p3Dw - twc;
      const float dist3D = cv::norm(PO);
      const float maxDistance = pMP->GetMaxDistanceInvariance();
      const float minDistance = pMP->GetMinDistanceInvariance();
      // Depth must be inside the scale invariance region
      if (dist3D < minDistance || dist3D > maxDistance) continue;
      if (bCheckViewingAngle) {
        // Viewing angle must be less than 60 deg
        if (PO.dot(Pn) < 0.5 * dist3D) continue;
      }
      // Compute predicted octave
      const int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

      // Search in a radius
      const float radius = th_radius * pKF->mvScaleFactors[nPredictedLevel];

      const vector<size_t> vIndices = pKF->GetFeaturesInArea(cami, u, v, radius);

      if (vIndices.empty()) continue;

      // Match to the most similar keypoint in the radius
      const cv::Mat dMP = pMP->GetDescriptor();

      int bestDist = INT_MAX;  // wP could correspond to different feature in different cam
      int bestIdx = -1;
      for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
        const size_t idx = *vit;

        const cv::KeyPoint &kp = !usedistort ? pKF->mvKeysUn[idx] : pKF->mvKeys[idx];
        const int &kpLevel = kp.octave;

        // check the min/maxlevel here instead of GetFeaturesInArea()
        // scale check, kpLevel must be in [nPredictedLevel-1,nPredictedLevel], like SBP(Frame,vec<MP*>)
        if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;

        if (pbf) {
          // chi2 check
          // stereo feature points(have depth data)
          if (pKF->mvuRight[idx] >= 0) {
            // Check reprojection error in stereo
            const float &kpx = kp.pt.x;
            const float &kpy = kp.pt.y;
            const float &kpr = pKF->mvuRight[idx];
            const float ex = u - kpx;  // ref-rectiying
            const float ey = v - kpy;
            const float ur = u - (*pbf) * invz;
            const float er = ur - kpr;
            const float e2 = ex * ex + ey * ey + er * er;

            // chi2(0.05,3), suppose e^2 has sigma^2 then e2/simga2 has 1^2, then it has the chi2 standard distribution
            if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 7.8) continue;
          } else {
            // monocular feature points(tend to have no depth data)
            const float &kpx = kp.pt.x;
            const float &kpy = kp.pt.y;
            const float ex = u - kpx;
            const float ey = v - kpy;
            const float e2 = ex * ex + ey * ey;

            // chi2(0.05,2)
            if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99) continue;
          }
        }

        CV_Assert(!pKF->mapn2in_.size() || get<0>(pKF->mapn2in_[idx]) == cami);
        const cv::Mat &dKF = pKF->mDescriptors.row(idx);

        const int dist = DescriptorDistance(dMP, dKF);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdx = idx;
          //          auto pMP2 = pKF->GetMapPoint(bestIdx);
          //          if (pMP2 && !pMP2->isBad()) {
          //            auto idxsOf1mp = pMP2->GetIndexInKeyFrame(pKF);
          //            bool check = false;
          //            for (auto iter = idxsOf1mp.begin(), iterend = idxsOf1mp.end(); iter != iterend; ++iter) {
          //              if (bestIdx == *iter) check = true;
          //            }
          //            if (!check) {
          //              for (auto iter = idxsOf1mp.begin(), iterend = idxsOf1mp.end(); iter != iterend; ++iter) {
          //                PRINT_DEBUG_INFO_MUTEX("iter=" << *iter << " ", mlog::vieo_slam_debug_path, "debug.txt");
          //              }
          //              PRINT_DEBUG_INFO_MUTEX(
          //                  pKF->mnId << "kfidx" << idx << "mpid" << pMP2->mnId << ";check kf bad=" <<
          //                  (int)pKF->isBad()
          //                            << ";check mpobs=" << pMP2->Observations() << "/" <<
          //                            pMP2->GetObservations().size() << endl,
          //                  mlog::vieo_slam_debug_path, "debug.txt");
          //            }
          //            CV_Assert(check);
          //          }
        }
      }

      if (!only1match) {
        if (bestDist <= th_bestdist)  // SBP standard threshold
        {
          if (pvnMatch1) (*pvnMatch1)[i1].insert(bestIdx);
          if (!fuselater) {
            // bool check0 = pMP->IsInKeyFrame(pKF, -1, cami);
            pKF->FuseMP(bestIdx, pMP);
            // CV_Assert(!check0);
          }
          if (pnfused) ++*pnfused;
        }
      } else if (bestDist < bestDistInCams) {
        auto pMP2 = pKF->GetMapPoint(bestIdx);
        if (pMP2 && !pMP2->isBad()) {
          bestDistInCams = bestDist;
          bestIdxInCams = bestIdx;
        }
      }
    }
    if (only1match && bestDistInCams <= th_bestdist) {
      auto pMP2 = pKF->GetMapPoint(bestIdxInCams);
      if (pMP2 && !pMP2->isBad()) {
        auto idxsOf1mp = pMP2->GetIndexInKeyFrame(pKF);
        bool check = false;
        for (auto iter = idxsOf1mp.begin(), iterend = idxsOf1mp.end(); iter != iterend; ++iter) {
          if (bestIdxInCams == *iter) check = true;
          if (pvnMatch1) (*pvnMatch1)[i1].insert(*iter);
          if (!fuselater) pKF->FuseMP(bestIdxInCams, vpMapPoints1[i1]);
          if (pnfused) ++*pnfused;
        }
        CV_Assert(check);
      }
    }
  }
}

int ORBmatcher::SearchByProjection(
    Frame &F, const vector<MapPoint *> &vpMapPoints,
    const float th)  // should use F.isInFrustum(pMP,0.5) first, it coarsely judges the scale&&rotation invariance
{
  int nmatches = 0;
  vector<int> nmatches_cami(F.mpCameras.size() ? F.mpCameras.size() : 1);

  const bool bFactor = th != 1.0;

  // TODO: check if ORB3 close point judge needed and in LBAInertial
  for (size_t iMP = 0; iMP < vpMapPoints.size(); ++iMP) {
    MapPoint *pMP = vpMapPoints[iMP];
    auto &trackinfo = pMP->GetTrackInfoRef();
    // false when it's already in mCurrentFrame.mvpMapPoints or this local MapPoint is not in frustum of the
    // mCurrentFrame
    if (!trackinfo.btrack_inview_) continue;

    if (pMP->isBad()) continue;

    auto iter_scale_level = trackinfo.vtrack_scalelevel_.begin();
    auto iter_viewcos = trackinfo.vtrack_viewcos_.begin();
    list<float>::iterator iter_proj[trackinfo.NUM_PROJ];
    for (int k = 0; k < trackinfo.NUM_PROJ; ++k) iter_proj[k] = trackinfo.vtrack_proj_[k].begin();
    auto IncIter = [&]() {
      for (int k = 0; k < trackinfo.NUM_PROJ; ++k) ++iter_proj[k];
      ++iter_scale_level;
      ++iter_viewcos;
    };
    for (auto iter_cami = trackinfo.vtrack_cami_.begin(), iter_cami_end = trackinfo.vtrack_cami_.end();
         iter_cami != iter_cami_end; ++iter_cami, IncIter()) {
      size_t cami = *iter_cami;
      const int &nPredictedLevel = *iter_scale_level;

      // The size of the window will depend on the viewing direction
      float r = RadiusByViewingCos(*iter_viewcos);

      if (bFactor) r *= th;

      const vector<size_t> vIndices = F.GetFeaturesInArea(
          cami, *iter_proj[0], *iter_proj[1], r * F.mvScaleFactors[nPredictedLevel], nPredictedLevel - 1,
          nPredictedLevel);  //-1 is for mnTrackScaleLevel uses ceil(), ceil() can also give a larger r'

      if (vIndices.empty()) continue;

      const cv::Mat MPdescriptor = pMP->GetDescriptor();  // get the best descriptor of the MP

      // do MATCH_KNN_IN_EACH_IMG here for wP could correspond to different feature in different cam
      int bestDist = 256;
      int bestLevel = -1;
      int bestDist2 = 256;
      int bestLevel2 = -1;
      int bestIdx = -1;

      // Get best and second matches with near keypoints
      const auto &frame_mps = F.GetMapPointMatches();
      for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
        const size_t idx = *vit;

        if (frame_mps[idx])  // if this keypoint has already corresponding MapPoint(by
                             // TrackWithMotionModel/TrackReferenceKeyFrame() or by this function)
          if (frame_mps[idx]->Observations() > 0) continue;

        if (F.mvuRight[idx] > 0) {
          const float er = fabs(*iter_proj[2] - F.mvuRight[idx]);
          if (er > r * F.mvScaleFactors[nPredictedLevel])  // if right virtual image's error is too large(>r')
            continue;
        }

        // const cv::Mat &d = !F.mapn2ijn_.size() ? F.mDescriptors.row(idx) :
        // F.vdescriptors_[cami].row(get<2>(F.mapn2ijn_[idx]));
        const cv::Mat &d = F.mDescriptors.row(idx);

        const int dist = DescriptorDistance(MPdescriptor, d);

        if (dist < bestDist) {
          bestDist2 = bestDist;
          bestDist = dist;
          bestLevel2 = bestLevel;
          bestLevel = F.mvKeys[idx].octave;  // Un
          bestIdx = idx;
        } else if (dist < bestDist2) {
          bestLevel2 = F.mvKeys[idx].octave;  // Un
          bestDist2 = dist;
        }
      }

      // Apply ratio to second match (only if best and second are in the same scale level), there are 2 possible levels
      if (bestDist <= TH_HIGH) {
        // if bestDist/bestDist2 <= threshold then this bestIdx can be matched with this MP
        if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2) continue;

        PRINT_DEBUG_INFO_MUTEX("bestidx" << bestIdx << "," << pMP->mnId << " ", mlog::vieo_slam_debug_path, "debug.txt");
        F.AddMapPoint(pMP, bestIdx);
        nmatches++;
        nmatches_cami[cami]++;
      }
    }
  }
  for (size_t cami = 0; cami < nmatches_cami.size(); ++cami)
    PRINT_DEBUG_INFO("nmatches_cami[]" << cami << "=" << nmatches_cami[cami] << endl, mlog::vieo_slam_debug_path,
                     "tracking_thread_debug.txt");

  return nmatches;  // this is not all the matches in mCurrentFrame.mvpMapPoints, just the addition part by local map
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos) {
  if (viewCos > 0.998)  // in +- 3.6 degrees, the searching window will be smaller
    return 2.5;
  else
    return 4.0;
}

int ORBmatcher::SearchByBoW(KeyFrame *pKF, Frame &F, vector<MapPoint *> &vpMapPointMatches) {
  const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches();  // for pKF->mvpMapPoints is protected

  vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));

  const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

  int nmatches = 0;

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
  DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
  DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
  DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
  DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

  map<pair<MapPoint *, size_t>, tuple<int, size_t, pair<size_t, size_t>>> mapmpcami2distkpidhist;
  vector<size_t> rothist2erase[HISTO_LENGTH];
  while (KFit != KFend && Fit != Fend) {
    if (KFit->first ==
        Fit->first)  // like window search in SBP(), now Frame.features are in KF.features' neighborhood area
    {
      const vector<unsigned int> vIndicesKF = KFit->second;
      const vector<unsigned int> vIndicesF = Fit->second;

      for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {
        const unsigned int realIdxKF = vIndicesKF[iKF];

        MapPoint *pMP = vpMapPointsKF[realIdxKF];

        if (!pMP) continue;

        if (pMP->isBad()) continue;

        const cv::Mat &dKF = pKF->mDescriptors.row(realIdxKF);

        vector<int> vbestDist1 = vector<int>(1, 256);
        vector<int> vbestDist2 = vector<int>(1, 256);
        vector<int> vbestIdxF = vector<int>(1, -1);

        for (size_t iF = 0; iF < vIndicesF.size(); iF++) {
          const unsigned int realIdxF = vIndicesF[iF];

          // avoid duplicate matching in this function()
          size_t img_id = 0;
          if (vpMapPointMatches[realIdxF]) continue;

          const cv::Mat &dF = F.mDescriptors.row(realIdxF);

          const int dist = DescriptorDistance(dKF, dF);

#define MATCH_KNN_IN_EACH_IMG  // we should use this to avoid 1mp seen by 4 cams with similar descriptors
#ifdef MATCH_KNN_IN_EACH_IMG
          if (F.mapn2in_.size() > realIdxF) {
            img_id = get<0>(F.mapn2in_[realIdxF]);
            if (vbestDist1.size() <= img_id) {
              size_t n_size = img_id + 1;
              vbestDist1.resize(n_size, 256);
              vbestDist2.resize(n_size, 256);
              vbestIdxF.resize(n_size, -1);
            }
          }
#endif

          // cout << "dist="<<dist<<";";
          if (dist < vbestDist1[img_id]) {
            vbestDist2[img_id] = vbestDist1[img_id];
            vbestDist1[img_id] = dist;
            vbestIdxF[img_id] = realIdxF;
          } else if (dist < vbestDist2[img_id]) {
            vbestDist2[img_id] = dist;
          }
        }

        for (int img_id = 0; img_id < vbestDist1.size(); ++img_id) {
          if (vbestDist1[img_id] <= TH_LOW)  // SBBow uses TH_LOW,while SBP uses TH_HIGH
          {
            // similar in SearchByProjection(Frame,vector<MapPoint*>&,...)
            if (static_cast<float>(vbestDist1[img_id]) < mfNNratio * static_cast<float>(vbestDist2[img_id])) {
              // we only change mp match in the same cami when the dist is smaller
              auto mpcami = make_pair(pMP, img_id);
              auto iterdist = mapmpcami2distkpidhist.find(mpcami);
              if (mapmpcami2distkpidhist.end() != iterdist) {
                if (get<0>(iterdist->second) <= vbestDist1[img_id])
                  continue;
                else {
                  vpMapPointMatches[get<1>(iterdist->second)] = nullptr;
                  --nmatches;
                  if (mbCheckOrientation) {
                    const auto &hist2erase = get<2>(iterdist->second);
                    const auto &bin2erase = get<0>(hist2erase);
                    CV_Assert(-1 != bin2erase);
                    rothist2erase[bin2erase].push_back(get<1>(hist2erase));
                  }
                }
              }

              vpMapPointMatches[vbestIdxF[img_id]] = pMP;

              const cv::KeyPoint &kp = pKF->mvKeys[realIdxKF];  // Un

              tuple<int, size_t, pair<size_t, size_t>> tuptmp;
              if (mbCheckOrientation)  // the same in SearchByProjection(Frame,const Frame,...)
              {
                // zzh change here, old ORBSLAM2 uses mvKeys[], whose angle should be the same
                // referenceFrame.angle-CurrentFrame.angle,Un
                float rot = kp.angle - F.mvKeys[vbestIdxF[img_id]].angle;
                if (rot < 0.0) rot += 360.0f;
                int bin = round(rot * factor);
                if (bin == HISTO_LENGTH) bin = 0;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                tuptmp = make_tuple(vbestDist1[img_id], vbestIdxF[img_id], make_pair(bin, rotHist[bin].size()));
                rotHist[bin].push_back(vbestIdxF[img_id]);
              } else
                tuptmp = make_tuple(vbestDist1[img_id], vbestIdxF[img_id], make_pair(-1, -1));
              mapmpcami2distkpidhist.emplace(mpcami, tuptmp);

              nmatches++;
            }
          }
        }
      }

      KFit++;
      Fit++;
    } else if (KFit->first < Fit->first) {
      KFit = vFeatVecKF.lower_bound(Fit->first);  // jump lots of ++KFit
    } else {
      Fit = F.mFeatVec.lower_bound(KFit->first);
    }
  }

  if (mbCheckOrientation)  // the same in SearchByProjection(Frame,const Frame,...)
  {
    vector<int> rotHist2[HISTO_LENGTH];
    for (size_t i = 0, iend = HISTO_LENGTH; i < iend; ++i) {
      for (size_t j = 0, jend = rothist2erase[i].size(); j < jend; ++j) rotHist[i][rothist2erase[i][j]] = -1;
      size_t jend = rotHist[i].size();
      rotHist2[i].reserve(jend);
      for (size_t j = 0; j < jend; ++j)
        if (-1 != rotHist[i][j]) rotHist2[i].push_back(rotHist[i][j]);
    }
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist2, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist2[i].size(); j < jend; j++) {
        vpMapPointMatches[rotHist2[i][j]] = static_cast<MapPoint *>(NULL);
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints,
                                   vector<MapPoint *> &vpMatched, int th) {
  // Decompose Scw
  cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);      // s*Rcw
  const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));  // s=sqrt(s^2*[(R*R.t())(0,0) or norm(Rcw.row(0))]^2)
  cv::Mat Rcrw = sRcw / scw;                             // Rcw(in Tcw)
  cv::Mat tcrw = Scw.rowRange(0, 3).col(3) /
                 scw;  // for scw=sc/1 then tcw_trueScale/tcw(in Tcw)=(tcw in Scw)/sc=Scw.rowRange(0,3).col(3)/scw
  cv::Mat twcr = -Rcrw.t() * tcrw;  // Ow=twc(in Twc)=-Rcw.t()*tcw(in Tcw) or Tcw^(-1).rowRange(0,3).col(3)

  // Set of MapPoints already found in the KeyFrame, notice the MPs is the matched MPs not the pKF->mvpMapPoints
  set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());  // vpMatched may has nullptr
  spAlreadyFound.erase(static_cast<MapPoint *>(NULL));                 // so need to erase nullptr

  int nmatches = 0;  // additional matches between vpPoints and pKF->mvpMapPoints

  // For each Candidate MapPoint Project and Match
  for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
    MapPoint *pMP = vpPoints[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP)) continue;

    // Get 3D Coords.
    cv::Mat p3Dw = pMP->GetWorldPos();
    cv::Mat Pn = pMP->GetNormal();

    // Transform into Camera Coords.
    cv::Mat Pcr = Rcrw * p3Dw + tcrw;  // Xc=(Tcw*Xw)(0:2)
    size_t n_cams = !pKF->mpCameras.size() ? 1 : pKF->mpCameras.size();
    for (size_t cami = 0; cami < n_cams; ++cami) {
      Vector3d Pc = Converter::toVector3d(Pcr);
      cv::Mat twc = twcr.clone();
      GeometricCamera *pcam1 = nullptr;
      if (pKF->mpCameras.size() > cami) {
        pcam1 = pKF->mpCameras[cami];
        Pc = pcam1->GetTcr() * Pc;
        twc += Rcrw.t() * pcam1->Getcvtrc();
      }
      // Depth must be positive
      if (Pc(2) <= 0.0)  //== rectified by zzh
        continue;

      // Project into Image
      const float invz = 1 / Pc(2);
      float u, v;
      if (!pcam1 || !Frame::usedistort_) {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;
        u = fx * Pc(0) * invz + cx;
        v = fy * Pc(1) * invz + cy;
      } else {
        auto pt = pcam1->project(Pc);
        u = pt[0];
        v = pt[1];
      }

      // Point must be inside the image
      if (!pKF->IsInImage(u, v)) continue;

      cv::Mat PO = p3Dw - twc;
      const float dist = cv::norm(PO);
      // Depth must be inside the scale invariance region of the point
      const float maxDistance = pMP->GetMaxDistanceInvariance();
      const float minDistance = pMP->GetMinDistanceInvariance();
      if (dist < minDistance || dist > maxDistance) continue;
      // Viewing angle must be less than 60 deg, 60 deg also used in mCurrentFrame.isInFrustum() in SearchLocalPoints()
      if (PO.dot(Pn) < 0.5 * dist) continue;
      int nPredictedLevel = pMP->PredictScale(dist, pKF);

      // Search in a radius
      const float radius =
          th * pKF->mvScaleFactors[nPredictedLevel];  // here use th=10 in ComputeSim3() in LoopClosing, same as the
                                                      // first/coarse trial threshold adding inliers by SBP in
                                                      // Relocalization() in Tracking

      const vector<size_t> vIndices = pKF->GetFeaturesInArea(cami, u, v, radius);

      if (vIndices.empty()) continue;

      // Match to the most similar keypoint in the radius
      const cv::Mat dMP = pMP->GetDescriptor();

      int bestDist = 256;
      int bestIdx = -1;
      for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
        const size_t idx = *vit;
        if (vpMatched[idx])  // here vpMatched[idx]!=pMP(for !spAlreadyFound.count(pMP)) but if it's not nullptr meaning
                             // it/pKF->mvpMapPoints[idx] has already been matched(pMP is not matched)
          continue;

        const int &kpLevel = pKF->mvKeys[idx].octave;  // Un

        if (kpLevel < nPredictedLevel - 1 ||
            kpLevel > nPredictedLevel)  // check nPredictedLevel error here not in pKF->GetFeaturesInArea(), same as
                                        // SearchBySim3(), like SBP(Frame,vec<MP*>)
          continue;

        // const cv::Mat &dKF = !pKF->mapn2ijn_.size() ? pKF->mDescriptors.row(idx) :
        // pKF->vdescriptors_[cami].row(get<2>(pKF->mapn2ijn_[idx]));
        KeyFrame &F = *pKF;
        const cv::Mat &d = F.mDescriptors.row(idx);

        const int dist = DescriptorDistance(dMP, d);

        if (dist < bestDist) {
          bestDist = dist;
          bestIdx = idx;
        }
      }

      if (bestDist <= TH_LOW)  // but use SBBoW standard threshold for a stricter addition of matches(for validation of
                               // nTotalMatches) for LoopClosing
      {
        vpMatched[bestIdx] = pMP;  // rectify vpMatched(add matches)
        nmatches++;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched,
                                        vector<int> &vnMatches12, int windowSize) {
  int nmatches = 0;
  vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
  vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

  for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++) {
    cv::KeyPoint kp1 = F1.mvKeysUn[i1];
    int level1 = kp1.octave;
    if (level1 > 0) continue;

    vector<size_t> vIndices2 =
        F2.GetFeaturesInArea(0, vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

    if (vIndices2.empty()) continue;

    cv::Mat d1 = F1.mDescriptors.row(i1);

    int bestDist = INT_MAX;
    int bestDist2 = INT_MAX;
    int bestIdx2 = -1;

    for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
      size_t i2 = *vit;

      cv::Mat d2 = F2.mDescriptors.row(i2);

      int dist = DescriptorDistance(d1, d2);

      if (vMatchedDistance[i2] <= dist) continue;

      if (dist < bestDist) {
        bestDist2 = bestDist;
        bestDist = dist;
        bestIdx2 = i2;
      } else if (dist < bestDist2) {
        bestDist2 = dist;
      }
    }

    if (bestDist <= TH_LOW) {
      if (bestDist < (float)bestDist2 * mfNNratio) {
        if (vnMatches21[bestIdx2] >= 0) {
          vnMatches12[vnMatches21[bestIdx2]] = -1;
          nmatches--;
        }
        vnMatches12[i1] = bestIdx2;
        vnMatches21[bestIdx2] = i1;
        vMatchedDistance[bestIdx2] = bestDist;
        nmatches++;

        if (mbCheckOrientation) {
          float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
          if (rot < 0.0) rot += 360.0f;
          int bin = round(rot * factor);
          if (bin == HISTO_LENGTH) bin = 0;
          assert(bin >= 0 && bin < HISTO_LENGTH);
          rotHist[bin].push_back(i1);
        }
      }
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        int idx1 = rotHist[i][j];
        if (vnMatches12[idx1] >= 0) {
          vnMatches12[idx1] = -1;
          nmatches--;
        }
      }
    }
  }

  // Update prev matched
  for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
    if (vnMatches12[i1] >= 0) vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;

  return nmatches;
}

// TODO: unify to one SBBBase
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12) {
  const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeys;  // Un
  const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
  const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  const cv::Mat &Descriptors1 = pKF1->mDescriptors;

  const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeys;  // Un
  const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
  const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
  const cv::Mat &Descriptors2 = pKF2->mDescriptors;

  vpMatches12 = vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
  vector<bool> vbMatched2(vpMapPoints2.size(), false);

  vector<int> rotHist[HISTO_LENGTH];  // angle filter
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  int nmatches = 0;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  map<pair<MapPoint *, size_t>, tuple<int, size_t, size_t, pair<size_t, size_t>>> mapmpcami2distkp12idhist;
  vector<size_t> rothist2erase[HISTO_LENGTH];
  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++)  // refKF's features' ID
      {
        const size_t idx1 = f1it->second[i1];

        MapPoint *pMP1 = vpMapPoints1[idx1];
        if (!pMP1) continue;
        if (pMP1->isBad()) continue;

        const cv::Mat &d1 = Descriptors1.row(idx1);

        vector<int> vbestDist1 = vector<int>(1, 256);
        vector<int> vbestDist2 = vector<int>(1, 256);
        vector<int> vbestIdx2 = vector<int>(1, -1);

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++)  // rectifying KF's features' ID
        {
          const size_t idx2 = f2it->second[i2];

          MapPoint *pMP2 = vpMapPoints2[idx2];

          if (vbMatched2[idx2] ||
              !pMP2)  // avoid duplications && need pKF2->mvpMapPoints[idx2] exists, different from another SBB
            continue;

          if (pMP2->isBad()) continue;

          const cv::Mat &d2 = Descriptors2.row(idx2);

          int dist = DescriptorDistance(d1, d2);

          size_t img_id = 0;
          // TODO: check if needed
#undef MATCH_KNN_IN_EACH_IMG
#ifdef MATCH_KNN_IN_EACH_IMG
          if (pKF2->mapn2in_.size() > idx2) {
            img_id = get<0>(pKF2->mapn2in_[idx2]);
            if (vbestDist1.size() <= img_id) {
              size_t n_size = img_id + 1;
              vbestDist1.resize(n_size, 256);
              vbestDist2.resize(n_size, 256);
              vbestIdx2.resize(n_size, -1);
            }
          }
#endif

          if (dist < vbestDist1[img_id]) {
            vbestDist2[img_id] = vbestDist1[img_id];
            vbestDist1[img_id] = dist;
            vbestIdx2[img_id] = idx2;
          } else if (dist < vbestDist2[img_id]) {
            vbestDist2[img_id] = dist;
          }
        }

        for (int img_id = 0; img_id < vbestDist1.size(); ++img_id) {
          if (vbestDist1[img_id] < TH_LOW)  // SBB uses TH_LOW as the threshold of Hamming distance
          {
            // mfNNratio used in ComputeSim3() in LoopClosing is 0.75
            if (static_cast<float>(vbestDist1[img_id]) < mfNNratio * static_cast<float>(vbestDist2[img_id])) {
              // we only change mp match in the same cami when the dist is smaller
              auto mpcami = make_pair(pMP1, img_id);
              auto iterdist = mapmpcami2distkp12idhist.find(mpcami);
              if (mapmpcami2distkp12idhist.end() != iterdist) {
                if (get<0>(iterdist->second) <= vbestDist1[img_id])
                  continue;
                else {
                  vpMatches12[get<1>(iterdist->second)] = nullptr;
                  vbMatched2[get<2>(iterdist->second)] = false;
                  --nmatches;
                  if (mbCheckOrientation) {
                    const auto &hist2erase = get<3>(iterdist->second);
                    const auto &bin2erase = get<0>(hist2erase);
                    CV_Assert(-1 != bin2erase);
                    rothist2erase[bin2erase].push_back(get<1>(hist2erase));
                  }
                }
              }

              // notice here is not vpMatches21[bestIdx2]=pMP1, unlike another SBB, this SBB rectifying the vpMatches12
              // instead of vpMatches21
              vpMatches12[idx1] = vpMapPoints2[vbestIdx2[img_id]];
              // but we can still think it's also rectifying pKF2's corresponding vbMatched2
              vbMatched2[vbestIdx2[img_id]] = true;

              tuple<int, size_t, size_t, pair<size_t, size_t>> tuptmp;
              if (mbCheckOrientation)  // true in ComputeSim3()
              {
                float rot = vKeysUn1[idx1].angle - vKeysUn2[vbestIdx2[img_id]].angle;  // ref - rectifying(vbMatched2)
                if (rot < 0.0) rot += 360.0f;
                int bin = round(rot * factor);
                if (bin == HISTO_LENGTH) bin = 0;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                tuptmp = make_tuple(vbestDist1[img_id], idx1, vbestIdx2[img_id], make_pair(bin, rotHist[bin].size()));
                rotHist[bin].push_back(idx1);
              } else
                tuptmp = make_tuple(vbestDist1[img_id], idx1, vbestIdx2[img_id], make_pair(-1, -1));
              mapmpcami2distkp12idhist.emplace(mpcami, tuptmp);

              nmatches++;
            }
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);  // jump
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    vector<int> rotHist2[HISTO_LENGTH];
    for (size_t i = 0, iend = HISTO_LENGTH; i < iend; ++i) {
      for (size_t j = 0, jend = rothist2erase[i].size(); j < jend; ++j) rotHist[i][rothist2erase[i][j]] = -1;
      size_t jend = rotHist[i].size();
      rotHist2[i].reserve(jend);
      for (size_t j = 0; j < jend; ++j)
        if (-1 != rotHist[i][j]) rotHist2[i].push_back(rotHist[i][j]);
    }

    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist2, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist2[i].size(); j < jend; j++) {
        vpMatches12[rotHist2[i][j]] = static_cast<MapPoint *>(NULL);
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, vector<vector<vector<size_t>>> &vMatchedPairs,
                                       const bool bOnlyStereo) {
  size_t vn_cams[2] = {pKF1->mpCameras.size(), pKF2->mpCameras.size()};
  for (int i = 0; i < 2; ++i)
    if (0 >= vn_cams[i]) vn_cams[i] = 1;
  bool usedistort[2] = {pKF1->mpCameras.size() && Frame::usedistort_, pKF2->mpCameras.size() && Frame::usedistort_};
  const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
  const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

  // Compute epipole in second image
  cv::Mat Cw = pKF1->GetCameraCenter();
  cv::Mat R2w = pKF2->GetRotation();
  cv::Mat t2w = pKF2->GetTranslation();
  cv::Mat C2 =
      R2w * Cw + t2w;  //(Tc2w*Twc1).col(3).copyTo(Tc2c1.col(3)), don't consider Tc2c1.col(3)=(Tc2w*Twc1).col(3)!
  float ex, ey;
  auto Tr1r2 = pKF1->GetTcw() * pKF2->GetTwc();
  if (!usedistort[1]) {
    const float invz = 1.0f / C2.at<float>(2);
    ex = pKF2->fx * C2.at<float>(0) * invz + pKF2->cx;
    ey = pKF2->fy * C2.at<float>(1) * invz + pKF2->cy;
  } else {
    auto pt = pKF2->mpCameras[0]->project(C2);
    ex = pt.x;
    ey = pt.y;
  }

  // Find matches between not tracked keypoints
  // Matching speed-up by ORB Vocabulary
  // Compare only ORB that share the same node

  int nmatches = 0;

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);

  const float factor = 1.0f / HISTO_LENGTH;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  shared_ptr<Pinhole> pcaminst[2];
  if (!usedistort[0]) {
    CV_Assert(!usedistort[1]);
    pcaminst[0] = make_shared<Pinhole>(vector<float>({pKF1->fx, pKF1->fy, pKF1->cx, pKF1->cy}));
    pcaminst[1] = make_shared<Pinhole>(vector<float>({pKF2->fx, pKF2->fy, pKF2->cx, pKF2->cy}));
  } else
    CV_Assert(usedistort[1]);

#ifdef USE_STRATEGY_MIN_DIST
  vector<vector<double>> lastdists;
#endif
  vector<vector<size_t>> vidxs_matches;
  vector<bool> goodmatches;
  map<pair<size_t, size_t>, size_t> mapcamidx2idxs;  // tot 8 cams
  size_t n_cams = vn_cams[0] + vn_cams[1];
  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];

        MapPoint *pMP1 = pKF1->GetMapPoint(idx1);

        // If there is already a MapPoint skip
        if (pMP1) continue;

        const bool bStereo1 = pKF1->mvuRight[idx1] >= 0;  // can be optimized in RGBD by using mvDepth[idx1]>0

        // in CreateNewMapPoints() in LocalMapping it's false, means triangulate even monocular point
        // without stereo matches(may happen in RGBD even with depth>0)
        if (bOnlyStereo)
          if (!bStereo1) continue;

        const cv::KeyPoint &kp1 = !usedistort[0] ? pKF1->mvKeysUn[idx1] : pKF1->mvKeys[idx1];

        const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

        vector<int> vbestDist = vector<int>(1, TH_LOW);
        vector<int> vbestIdx2 = vector<int>(1, -1);

        size_t cam1 = pKF1->mapn2in_.size() <= idx1 ? 0 : get<0>(pKF1->mapn2in_[idx1]);
        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          size_t idx2 = f2it->second[i2];

          MapPoint *pMP2 = pKF2->GetMapPoint(idx2);

          // If we have already matched or there is a MapPoint skip, avoid for replicated match
          if (pMP2) continue;
          size_t cam2 = (pKF2->mapn2in_.size() <= idx2 ? 0 : get<0>(pKF2->mapn2in_[idx2])) + vn_cams[0];
          // notice here we avoid multi2one match done in knnmatch in computestereofisheye(), to ensure injection match
          auto iterj = mapcamidx2idxs.find(make_pair(cam2, idx2));
          if (mapcamidx2idxs.end() != iterj && -1 != vidxs_matches[iterj->second][cam1]) continue;

          const bool bStereo2 = pKF2->mvuRight[idx2] >= 0;

          if (bOnlyStereo)
            if (!bStereo2) continue;

          const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

          const int dist = DescriptorDistance(d1, d2);

          size_t img_id = 0;
#define MATCH_KNN_IN_EACH_IMG
#ifdef MATCH_KNN_IN_EACH_IMG
          if (pKF2->mapn2in_.size() > idx2) {
            img_id = get<0>(pKF2->mapn2in_[idx2]);
            if (vbestDist.size() <= img_id) {
              size_t n_size = img_id + 1;
              vbestDist.resize(n_size, TH_LOW);
              vbestIdx2.resize(n_size, -1);
            }
          }
#endif

          // use hamming distance to match is right for the sparse creation, no need to use patch matching method
          if (dist > vbestDist[img_id])  // old dist>TH_LOW || is redundant
            continue;

          const cv::KeyPoint &kp2 = !usedistort[1] ? pKF2->mvKeysUn[idx2] : pKF2->mvKeys[idx2];

          if (!bStereo1 &&
              !bStereo2)  // both monocular points, just allow at least 7~14 square pixels away from c1,but why?
          {
            const float distex = ex - kp2.pt.x;
            const float distey = ey - kp2.pt.y;
            if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave]) continue;
          }

          GeometricCamera *pcam1, *pcam2;
          if (!usedistort[0]) {
            pcam1 = pcaminst[0].get();
            pcam2 = pcaminst[1].get();
          } else {
            pcam1 = pKF1->mpCameras[get<0>(pKF1->mapn2in_[idx1])];
            pcam2 = pKF2->mpCameras[get<0>(pKF2->mapn2in_[idx2])];
          }
          auto T12 = pcam1->GetTcr() * Tr1r2 * pcam2->GetTrc();
          cv::Mat R12 = Converter::toCvMat(T12.rotationMatrix());
          // Tc1c2 = Tc1r2 * Tr2c2, tc1c2 = Rc1r2 * tr2c2 - Rc1r2 * (Rr2r1 * tr1c1 + tr2r1)
          cv::Mat t12 = Converter::toCvMat(T12.translation());
          if (pcam1->epipolarConstrain(pcam2, kp1, kp2, R12, t12, pKF1->mvLevelSigma2[kp1.octave],
                                       pKF2->mvLevelSigma2[kp2.octave], usedistort[0])) {
            vbestIdx2[img_id] = idx2;
            vbestDist[img_id] = dist;
          }
        }

        for (int img_id = 0; img_id < vbestDist.size(); ++img_id) {
          if (vbestIdx2[img_id] >= 0)  // if exist good epipolar constraint match
          {
            const cv::KeyPoint &kp2 = pKF2->mvKeys[vbestIdx2[img_id]];  // Un
            auto &idx2 = vbestIdx2[img_id];
            pair<size_t, size_t> kfidxi = make_pair(0, idx1), kfidxj = make_pair(1, idx2);
            size_t cami[2] = {cam1, (pKF2->mapn2in_.size() <= idx2 ? 0 : get<0>(pKF2->mapn2in_[idx2])) + vn_cams[0]};
            uint8_t checkdepth[2] = {0};  // 1 means create, 2 means replace
            GeometricCamera *pcam1, *pcam2;
            if (!usedistort[0]) {
              pcam1 = pcaminst[0].get();
              pcam2 = pcaminst[1].get();
            } else {
              pcam1 = pKF1->mpCameras[cami[0]];
              pcam2 = pKF2->mpCameras[cami[1] - vn_cams[0]];
            }
            if (pcam1->FillMatchesFromPair(
                    vector<GeometricCamera *>(1, pcam2), n_cams,
                    vector<pair<size_t, size_t>>{make_pair(cami[0], idx1), make_pair(cami[1], idx2)}, vbestDist[img_id],
                    vidxs_matches, goodmatches, mapcamidx2idxs, 1. - 1.e-6, nullptr, nullptr, nullptr
#ifdef USE_STRATEGY_MIN_DIST
                    ,
                    &lastdists
#else
                    ,
                    nullptr
#endif
                    ))
              ++nmatches;

            if (mbCheckOrientation) {
              float rot = kp1.angle - kp2.angle;  // kp ref - kp rectifying(vbMatched2)
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(idx1);
            }
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3) continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        const auto &idx1 = rotHist[i][j];
        size_t cam1 = pKF1->mapn2in_.size() <= idx1 ? 0 : get<0>(pKF1->mapn2in_[idx1]);
        auto iteri = mapcamidx2idxs.find(make_pair(cam1, idx1));
        if (mapcamidx2idxs.end() == iteri) continue;
        goodmatches[iteri->second] = false;
        nmatches--;
      }
    }
  }

  vMatchedPairs.clear();
  vMatchedPairs.reserve(nmatches);

  //  set<pair<size_t, size_t>> checkred;
  for (size_t i = 0, iend = vidxs_matches.size(); i < iend; i++) {
    // CV_Assert(vidxs_matches[i].size() == n_cams);
#ifdef USE_STRATEGY_MIN_DIST
    size_t count_num = 0;
    for (size_t itmp = 0; itmp < n_cams; ++itmp) {
      if (-1 == vidxs_matches[i][itmp]) continue;
      ++count_num;
      //      pair<size_t, size_t> pairtmp = make_pair(itmp, vidxs_matches[i][itmp]);
      //      if (checkred.end() != checkred.find(pairtmp))
      //        CV_Assert(0 && "checkred fail");
      //      else
      //        checkred.insert(pairtmp);
    }
    if (count_num < 2) goodmatches[i] = false;
#endif
    if (!goodmatches[i]) continue;
    vector<vector<size_t>> matchedpair(2);
    matchedpair[0].insert(matchedpair[0].begin(), vidxs_matches[i].begin(), vidxs_matches[i].begin() + vn_cams[0]);
    matchedpair[1].insert(matchedpair[1].begin(), vidxs_matches[i].begin() + vn_cams[0], vidxs_matches[i].end());
    vMatchedPairs.push_back(matchedpair);
  }
  return nmatches;
}

int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th) {
  cv::Mat Rcw = pKF->GetRotation();
  cv::Mat tcw = pKF->GetTranslation();

  int nFused = 0;

  vector<set<int>> vnMatch;
  const float &bf = pKF->mbf;
  // like the threshold in SBBoW, though this is like a SBP method maybe for it should be stricter when used in far
  // position matching (fuse in LocalMapping, 0.6, true)
  SearchByProjectionBase(vpMapPoints, Rcw, tcw, pKF, th, TH_LOW, true, &bf, &nFused);

  return nFused;  // this is near the number of fused MPs
}

int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th,
                     vector<MapPoint *> &vpReplacePoint) {
  // Decompose Scw
  cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
  const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));  // s=sqrt(s^2*[(R*R.t())(0,0) or norm(Rcw.row(0))]^2)
  cv::Mat Rcrw = sRcw / scw;
  cv::Mat tcrw = Scw.rowRange(0, 3).col(3) / scw;  // get tcw in T from tcw in S
  cv::Mat twcr = -Rcrw.t() * tcrw;

  // Set of MapPoints already found in the KeyFrame
  const set<pair<MapPoint *, size_t>> spAlreadyFound = pKF->GetMapPointsCami();
  size_t N1 = vpPoints.size();
  size_t n_cams = !pKF->mpCameras.size() ? 1 : pKF->mpCameras.size();
  vector<vector<bool>> vbAlreadyMatched1(N1, vector<bool>(n_cams, false));
  for (int iMP = 0; iMP < N1; iMP++) {
    MapPoint *pMP = vpPoints[iMP];
    size_t cami = pKF->mapn2in_.size() <= iMP ? 0 : get<0>(pKF->mapn2in_[iMP]);
    if (spAlreadyFound.end() != spAlreadyFound.find(make_pair(pMP, cami))) {
      vbAlreadyMatched1[iMP][cami] = true;
    }
  }

  vector<set<int>> vnMatch;
  SearchByProjectionBase(vpPoints, Rcrw, tcrw, pKF, th, TH_LOW, true, nullptr, nullptr,
                         (char)(SBPFuseLater | SBPMatchMultiCam), &vbAlreadyMatched1, &vnMatch);
  int nFused = 0;
  for (int iMP = 0; iMP < vnMatch.size(); ++iMP) {
    auto bestIdxs = vnMatch[iMP];
    for (auto iter = bestIdxs.begin(); iter != bestIdxs.end(); ++iter) {
      auto bestIdx = *iter;
      if (-1 == bestIdx) continue;

      MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
      MapPoint *pMP = vpPoints[iMP];
      if (pMPinKF)  // if pKF->mvpMapPoints[bestIdx] exists
      {
        if (!pMPinKF->isBad())  // good
          // vPoints[iMP] is used to replace pKF->mvpMapPoints[bestIdx]/vpReplacePoint[iMP] in SearchAndFuse() in
          // LoopClosing
          vpReplacePoint[iMP] = pMPinKF;
        // TODO: when it's bad, could we just add mp?
      } else  // pKF->mvpMapPoints[bestIdx]==nullptr
      {       // directly add pMP into pKF and update pMP's mObservations
        pMP->AddObservation(pKF, bestIdx);
        PRINT_DEBUG_INFO_MUTEX("addmp2" << endl, mlog::vieo_slam_debug_path, "debug.txt");
        pKF->AddMapPoint(pMP, bestIdx);
      }
      nFused++;  // notice vpReplacePoint[idx] may appear many times, so nFuse <= the real fused/added matched MPs in
                 // pKF->mvpMapPoints
    }
  }

  return nFused;
}

int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12, const float &s12,
                             const cv::Mat &R12, const cv::Mat &t12, const float th) {
  // Camera 1 from world
  cv::Mat R1w = pKF1->GetRotation();
  cv::Mat t1w = pKF1->GetTranslation();

  // Camera 2 from world
  cv::Mat R2w = pKF2->GetRotation();
  cv::Mat t2w = pKF2->GetTranslation();

  // Transformation between cameras
  cv::Mat sR12 = s12 * R12;
  cv::Mat sR21 = (1.0 / s12) * R12.t();  // s21=s2/s1=1/(s1/s2)=1/s12, notice si means use the right/real/true scale
                                         // ti(Tiw) to get saved/calculated tiw(Siw)=si*ti(Tiw)
  cv::Mat t21 = -sR21 * t12;             // S12*S21=I/S21=S12^(-1) => s12*R12*s21*R21=I,s12R12*t21+t12=0 =>
                                         // sR21=s21*R21=1/s12*R12.t()/s21,t21=-(s12R12)^(-1)*t12=-s21*R21*t12=-sR21*t12

  const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  const int N1 = vpMapPoints1.size();

  const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
  const int N2 = vpMapPoints2.size();

  size_t n_cams = !pKF2->mpCameras.size() ? 1 : pKF2->mpCameras.size();
  vector<vector<bool>> vbAlreadyMatched1(N1, vector<bool>(n_cams, false));
  vector<vector<bool>> vbAlreadyMatched2(N2, vector<bool>(n_cams, false));
  for (int i = 0; i < N1; i++) {
    MapPoint *pMP = vpMatches12[i];  // corresponding to pKF1
    if (pMP) {
      size_t cami = pKF1->mapn2in_.size() <= i ? 0 : get<0>(pKF1->mapn2in_[i]);
      vbAlreadyMatched1[i][cami] = true;
      auto idxs2 = pMP->GetIndexInKeyFrame(pKF2);
      for (auto iter = idxs2.begin(), iterend = idxs2.end(); iter != iterend; ++iter) {
        auto idx2 = *iter;
        CV_Assert(idx2 < N2 && idx2 >= 0);
        cami = pKF2->mapn2in_.size() <= idx2 ? 0 : get<0>(pKF2->mapn2in_[idx2]);
        vbAlreadyMatched2[idx2][cami] = true;
      }
    }
  }

  vector<set<int>> vnMatch[2];
  // Transform from KF1 to KF2 and search
  SearchByProjectionBase(vpMapPoints1, sR21 * R1w, sR21 * t1w + t21, pKF2, th, TH_HIGH, false, nullptr, nullptr,
#ifdef CHECK_REPLACE_ALL
                         (char)~SBPMatchMultiCam,
#else
                         (char)~SBPMatchMultiCam,
#endif
                         &vbAlreadyMatched1, &vnMatch[0]);

  // Transform from KF2 to KF1 and search
  SearchByProjectionBase(vpMapPoints2, sR12 * R2w, sR12 * t2w + t12, pKF1, th, TH_HIGH, false, nullptr, nullptr,
#ifdef CHECK_REPLACE_ALL
                         (char)~SBPMatchMultiCam,
#else
                         (char)~SBPMatchMultiCam,
#endif
                         &vbAlreadyMatched2, &vnMatch[1]);

  // Check agreement
  int nFound = 0;

  for (int i1 = 0; i1 < N1; i1++) {
    auto idx2s = vnMatch[0][i1];
    for (auto iter = idx2s.begin(); iter != idx2s.end(); ++iter) {
      auto idx2 = *iter;
      if (idx2 >= 0) {
        if (vnMatch[1][idx2].end() != vnMatch[1][idx2].find(i1))  // rectified by zzh
        {
          vpMatches12[i1] = vpMapPoints2[idx2];  // vpMatches12[i1](before is nullptr)=pKF2->mvpMapPoints[idx2]
          nFound++;                              // vnMatch2[vnMatch1[i1]]==i1/agreement
          break;
        }
      }
    }
  }

  return nFound;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th,
                                   const bool bMono)  // rectify the CurrentFrame.mvpMapPoints
{
  int nmatches = 0;

  // Rotation Histogram (to check rotation consistency)
  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);
  const float factor = 1.0f / HISTO_LENGTH;

  const auto &Tcrw = CurrentFrame.GetTcwCst();
  const auto &Tlrw = LastFrame.GetTcwCst();
  const auto &Tlrcr = Tlrw * Tcrw.inverse();
  const auto &tlrcr = Tlrcr.translation();

  const bool bForward = tlrcr(2) > CurrentFrame.mb && !bMono;    // delta z >0.08m
  const bool bBackward = -tlrcr(2) > CurrentFrame.mb && !bMono;  // delta z<-0.08m

  size_t num_mp = 0;
  set<MapPoint *> sMP;
  const auto &lfmps = LastFrame.GetMapPointMatches();
  for (int i = 0; i < LastFrame.N; i++) {
    MapPoint *pMP = lfmps[i];
    if (pMP) {
      if (!LastFrame.mvbOutlier[i]) {
        /*if (sMP.count(pMP)) continue;
        else sMP.insert(pMP);*/
        PRINT_DEBUG_INFO_MUTEX("i" << i << "lfmpid=" << pMP->mnId << ":", mlog::vieo_slam_debug_path, "debug.txt");
        ++num_mp;
        // Project
        Eigen::Vector3d x3Dw = Converter::toVector3d(pMP->GetWorldPos());
        Eigen::Vector3d x3Dr = Tcrw * x3Dw, x3Dc;
        size_t n_camsj = !CurrentFrame.mpCameras.size() ? 1 : CurrentFrame.mpCameras.size();
        // CV_Assert(LastFrame.mapn2in_.size() > i);
        // size_t camj = get<0>(LastFrame.mapn2in_[i]);
        // CV_Assert(CurrentFrame.mpCameras.size() > camj;)
        for (size_t camj = 0; camj < n_camsj; ++camj) {
          if (CurrentFrame.mpCameras.size() > camj) {
            x3Dc = CurrentFrame.mpCameras[camj]->GetTcr() * x3Dr;
          } else
            x3Dc = x3Dr;

          const float xc = x3Dc(0);
          const float yc = x3Dc(1);
          const float invzc = 1.0 / x3Dc(2);  // inverse depth

          if (invzc < 0)  // behind the focus, cannot be photoed
            continue;

          float u, v;
          if (CurrentFrame.mpCameras.size() > camj && Frame::usedistort_) {
            auto pt = CurrentFrame.mpCameras[camj]->project(x3Dc);
            u = pt[0];
            v = pt[1];
          } else {
            u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;  // K*Xc
            v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;
          }

          if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)  // out of img range,cannot be photoed
            continue;
          if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY) continue;

          int nLastOctave = LastFrame.mvKeys[i].octave;

          // Search in a window. Size depends on scale
          float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];  // input th should be considered as the same
                                                                         // level with MapPoint[i].octave, but rectangle
                                                                         // window searching vIndices is in level==0

          vector<size_t> vIndices2;

          // use Image Pyramid(&& find minDist) to get the scale consistency
          if (bForward)
            vIndices2 = CurrentFrame.GetFeaturesInArea(
                camj, u, v, radius,
                nLastOctave);  // if camera is going forward, the old features(with PatchSize,level) should be found in
                               // (PatchSize,level+)(its area seems larger in level==0)
          else if (bBackward)
            vIndices2 = CurrentFrame.GetFeaturesInArea(camj, u, v, radius, 0, nLastOctave);
          else
            vIndices2 = CurrentFrame.GetFeaturesInArea(
                camj, u, v, radius, nLastOctave - 1,
                nLastOctave + 1);  // if cannot be sure of camera's motion, adjust level+/- 1

          if (vIndices2.empty()) continue;

          const cv::Mat dMP = pMP->GetDescriptor();  // get the best descriptor for the MapPoint

          int bestDist = 256;
          int bestIdx2 = -1;

          const auto &curfmps = CurrentFrame.GetMapPointMatches();
          for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend; vit++) {
            const size_t i2 = *vit;
            // avoid for rectifying same keypoint's MapPoint in CurrentFrame,for theoretically one-to-one match for
            // keypoints in Last&CurrentFrame
            if (curfmps[i2])
              if (curfmps[i2]->Observations() > 0) continue;

            if (CurrentFrame.mvuRight[i2] > 0) {
              const float ur = u - CurrentFrame.mbf * invzc;  // leftKP.x-mbf/leftKP.depth
              const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
              if (er >
                  radius)  // rectangle window should also be suitable in virtual right camera for RGBD,can use >= here
                continue;
            }

            // const cv::Mat &d = !CurrentFrame.mapn2ijn_.size() ? CurrentFrame.mDescriptors.row(i2) :
            // CurrentFrame.vdescriptors_[cami].row(get<2>(CurrentFrame.mapn2ijn_[i2]));
            Frame &F = CurrentFrame;
            auto idx = i2;
            CV_Assert(!F.mapn2in_.size() || get<0>(F.mapn2in_[idx]) == camj);
            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP, d);

            // do MATCH_KNN_IN_EACH_IMG here for wP could correspond to different feature in different cam
            if (dist < bestDist) {
              bestDist = dist;
              bestIdx2 = i2;
            }
          }

          if (bestDist <= TH_HIGH)  // 256>100 so bestIdx2!=-1
          {
            PRINT_DEBUG_INFO_MUTEX("cfbestidx" << bestIdx2 << ":" << pMP->mnId, mlog::vieo_slam_debug_path, "debug.txt");
            CurrentFrame.AddMapPoint(pMP, bestIdx2);
            nmatches++;

            if (mbCheckOrientation) {
              // it's degrees,Un
              float rot = LastFrame.mvKeys[i].angle - CurrentFrame.mvKeys[bestIdx2].angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);  // max=360./30=12<30
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(bestIdx2);
            }
          }
        }
      }
    }
  }

  // Apply rotation consistency
  if (mbCheckOrientation)  // use histogram distribution and only retain centre +- 18 degrees(special case) or one tenth
                           // of 360
  {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3)  // only max 3 sizes' vectors will be retained
      {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.EraseMapPointMatch(rotHist[i][j]);  // other matches will be deleted
          nmatches--;
        }
      }
    }
  }

  return nmatches;
}

// TODO: use SBPBase
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint *> &sAlreadyFound,
                                   const float th,
                                   const int ORBdist)  // a combination of SBP(Frame,Frame)&&SBP(Frame,vec<MP*>)
{
  int nmatches = 0;

  const auto &Tcrw = CurrentFrame.GetTcwCst();
  const auto &Twcr = Tcrw.inverse();
  const auto &twcr = Twcr.translation();
  const auto Rwcr = Twcr.rotationMatrix();

  // Rotation Histogram (to check rotation consistency)
  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++) rotHist[i].reserve(500);  // half of the 1000 nFeatures parameter in yaml
  const float factor = 1.0f / HISTO_LENGTH;

  const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
    MapPoint *pMP = vpMPs[i];

    if (pMP) {
      if (!pMP->isBad() && !sAlreadyFound.count(pMP))  // if this MP is good and not found in sFound
      {
        // Project
        Eigen::Vector3d x3Dw = Converter::toVector3d(pMP->GetWorldPos());

        Eigen::Vector3d x3Dcr = Tcrw * x3Dw;
        size_t n_cams = !CurrentFrame.mpCameras.size() ? 1 : CurrentFrame.mpCameras.size();
        for (size_t cami = 0; cami < n_cams; ++cami) {
          Vector3d Pc = x3Dcr;
          Eigen::Vector3d twc = twcr;
          GeometricCamera *pcam1 = nullptr;
          if (CurrentFrame.mpCameras.size() > cami) {
            pcam1 = CurrentFrame.mpCameras[cami];
            Pc = pcam1->GetTcr() * Pc;
            twc += Rwcr * pcam1->GetTrc().translation();
          }
          const float invzc = 1.0 / Pc(2);

          float u, v;
          if (!pcam1 || !Frame::usedistort_) {
            const float xc = Pc(0);
            const float yc = Pc(1);
            u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
            v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;
          } else {
            auto pt = CurrentFrame.mpCameras[cami]->project(Pc);
            u = pt[0];
            v = pt[1];
          }

          if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX) continue;
          if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY) continue;

          // Compute predicted scale level
          Eigen::Vector3d PO = x3Dw - twc;
          float dist3D = PO.norm();
          const float maxDistance = pMP->GetMaxDistanceInvariance();
          const float minDistance = pMP->GetMinDistanceInvariance();
          // Depth must be inside the scale pyramid of the image
          // if it's out of the frustum, image pyramid is not effective
          if (dist3D < minDistance || dist3D > maxDistance) continue;
          int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

          // Search in a window
          const float radius = th * CurrentFrame.mvScaleFactors[nPredictedLevel];

          const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(
              cami, u, v, radius, nPredictedLevel - 1,
              nPredictedLevel + 1);  // use maxLevel=nPredictedLevel+1, looser than SBP(Frame&,vector<MapPoint*>&)

          if (vIndices2.empty()) continue;

          const cv::Mat dMP = pMP->GetDescriptor();

          int bestDist = 256;  // TODO: no MATCH_KNN_IN_EACH_IMG
          int bestIdx2 = -1;

          const auto &curfmps = CurrentFrame.GetMapPointMatches();
          for (vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
            const size_t i2 = *vit;
            if (curfmps[i2])  // avoid replicate matching
              continue;

            // const cv::Mat &d = !CurrentFrame.mapn2ijn_.size() ? CurrentFrame.mDescriptors.row(i2) :
            // CurrentFrame.vdescriptors_[cami].row(get<2>(CurrentFrame.mapn2ijn_[i2]));
            Frame &F = CurrentFrame;
            auto idx = i2;
            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP, d);

            if (dist < bestDist) {
              bestDist = dist;
              bestIdx2 = i2;
            }
          }

          if (bestDist <= ORBdist)  // ORBdist must < 256, notice here don't use TH_HIGH(for SBP) or TH_LOW(for SBBoW)
          {
            CurrentFrame.AddMapPoint(pMP, bestIdx2);
            nmatches++;

            if (mbCheckOrientation) {
              // RefFrame.keypoint[i].angle-CurrentFrame.keypoint[i].angle,Un
              float rot = pKF->mvKeys[i].angle - CurrentFrame.mvKeys[bestIdx2].angle;
              if (rot < 0.0) rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH) bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(bestIdx2);
            }
          }
        }
      }
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.EraseMapPointMatch(rotHist[i][j]);
          nmatches--;
        }
      }
    }
  }

  return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2,
                                    int &ind3)  // find the max 3 sizes' vectors(max2,3>=0.1max1) and their indices
{
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    } else if (s > max3) {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1) {
    ind2 = -1;
    ind3 = -1;
  } else if (max3 < 0.1f * (float)max1) {
    ind3 = -1;
  }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(
    const cv::Mat &a,
    const cv::Mat &b)  // fastest(12 operators) way to calculate the hamming distance of two descriptors
{
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  // get hamming distance in the fastest way, also explained in https://www.91r.net/ask/8144833.html
  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v -
        ((v >> 1) & 0x55555555);  // use 2bit to show the hamming dis. of 2bit, only 4 situations:00,01,10,11->0,1,1,2
    v = (v & 0x33333333) +
        ((v >> 2) & 0x33333333);  // use 4 bit to show the ham. dis. of 4bit, only possible results are 0~4 < 16
    dist +=
        (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;  // use 8 bit to show the ham. dis of 8bit, 0~8<256 and
                                                           // then get A+B+C+D B+C+D C+D D>>24 or the result A+B+C+D!
  }

  return dist;
}

}  // namespace VIEO_SLAM
