/**
 * This file is part of VIEO_SLAM
 */

#include <mutex>
#include "KeyFrame.h"
#include "common/serialize/serialize.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "common/config.h"
#include "common/mlog/log.h"

namespace VIEO_SLAM {

std::mutex KeyFrame::mstMutexPNChanging;

void KeyFrame::UpdatePoseFromNS()  // same as Frame::UpdatePoseFromNS()
{
  cv::Mat Rbc = Frame::mTbc.rowRange(0, 3).colRange(0, 3);  // don't need clone();
  cv::Mat Pbc = Frame::mTbc.rowRange(0, 3).col(3);          // or tbc

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

void KeyFrame::UpdateNavStatePVRFromTcw() {
  unique_lock<mutex> lock(mMutexNavState);
  cv::Mat Twb;
  {
    unique_lock<mutex> lock(mMutexPose);  // important for using Tcw for this func. is multi threads!
    Twb = Converter::toCvMatInverse(Frame::mTbc * Tcw_);
  }
  Eigen::Matrix3d Rwb = Converter::toMatrix3d(Twb.rowRange(0, 3).colRange(0, 3));
  Eigen::Vector3d Pwb = Converter::toVector3d(Twb.rowRange(0, 3).col(3));

  Eigen::Matrix3d Rw1 = mNavState.getRwb();  // Rwbj_old/Rwb1
  Eigen::Vector3d Vw1 = mNavState.mvwb;  // Vw1/wV1=wvbj-1bj_old now bj_old/b1 is changed to bj_new/b2, wV2=wvbj-1bj_new
  Eigen::Vector3d Vw2 = Rwb * Rw1.transpose() * Vw1;  // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

  mNavState.mpwb = Pwb;
  mNavState.setRwb(Rwb);
  mNavState.mvwb = Vw2;
}

template <>  // specialized
void KeyFrame::SetPreIntegrationList<IMUData>(const listeig(IMUData)::const_iterator &begin,
                                              const listeig(IMUData)::const_iterator &end) {
  unique_lock<mutex> lock(mMutexOdomData);
  mOdomPreIntIMU.SetPreIntegrationList(begin, end);
}
template <>  // splice operation (like move) for fast append
void KeyFrame::AppendFrontPreIntegrationList(aligned_list<IMUData> &x,
                                             const typename aligned_list<IMUData>::const_iterator &begin,
                                             const typename aligned_list<IMUData>::const_iterator &end) {
  unique_lock<mutex> lock(mMutexOdomData);
  auto stop = end;
  if (mOdomPreIntIMU.getlOdom().size()) {
    double tm_ref = mOdomPreIntIMU.getlOdom().begin()->mtm;
    auto iter = end;
    for (; iter != begin;) {
      stop = iter--;
      if (iter->mtm >= tm_ref) continue;
      break;
    }
    if (iter != end && iter->mtm >= tm_ref) {
      CV_Assert(0 && "check AppendFrontPreIntegrationList usage!");
      stop = begin;
    }
  }
  mOdomPreIntIMU.AppendFrontPreIntegrationList(x, begin, stop);
  if (stop != end) x.erase(stop, end);
}
template <>
void KeyFrame::PreIntegration<IMUData>(KeyFrame *pLastKF) {
  Eigen::Vector3d bgi_bar = pLastKF->GetNavState().mbg, bai_bar = pLastKF->GetNavState().mba;
  unique_lock<mutex> lock(mMutexOdomData);
#ifndef TRACK_WITH_IMU
  mOdomPreIntIMU.PreIntegration(pLastKF->ftimestamp_, ftimestamp_);
#else
  // mOdomPreIntIMU.PreIntegration(pLastKF->ftimestamp_,ftimestamp_,bgi_bar,bai_bar);
  FrameBase::PreIntegration<IMUData>(pLastKF, mOdomPreIntIMU.getlOdom().begin(), mOdomPreIntIMU.getlOdom().end());
#endif
}
template <>
void KeyFrame::PreIntegrationFromLastKF<IMUData>(FrameBase *plastkf, double tmi,
                                                 const typename aligned_list<IMUData>::const_iterator &iteri,
                                                 const typename aligned_list<IMUData>::const_iterator &iterj,
                                                 bool breset, int8_t verbose) {
  NavState ns = plastkf->GetNavState();
  unique_lock<mutex> lock(mMutexOdomData);
  FrameBase::PreIntegration<IMUData, IMUPreintegrator>(tmi, ftimestamp_, ns.mbg, ns.mba, iteri, iterj, breset);
}

// for LoadMap()
// we don't update bias for convenience in LoadMap(), though we can do it as mOdomPreIntOdom is updated in read()
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, KeyFrame *pPrevKF, istream &is)
    : FrameBase(F),
      mnFrameId(F.nid_),
      mnTrackReferenceForFrame(0),
      mnFuseTargetForKF(0),
      mnBALocalForKF(0),
      mnBAFixedForKF(0),
      mnLoopQuery(0),
      mnLoopWords(0),
      mnRelocQuery(0),
      mnRelocWords(0),
      mnBAGlobalForKF(0),
      mpKeyFrameDB(pKFDB),  // for mK won't be changed, it cannot be used as clone()
      mbFirstConnection(false),
      mpParent(nullptr),
      mbNotErase(false),  // important false when LoadMap()!
      mbToBeErased(false),
      mpMap(pMap),
      mbPrior(false) {
  mDescriptors = F.mDescriptors.clone();

  if (pPrevKF) pPrevKF->SetNextKeyFrame(this);
  mpPrevKeyFrame = pPrevKF;
  mpNextKeyFrame = nullptr;  // zzh, constructor doesn't need to lock mutex

  Tcw_.release();
  SetPose(F.GetTcwRef());  // we have already used UpdatePoseFromNS() in Frame

  read(is);  // set odom list & mState

  nid_ = nNextId++;
}
bool KeyFrame::read(istream &is) {
  // we've done a lot in Frame Constructor with is!

  {  // load odom lists
    listeig(EncData) lenc;
    size_t NOdom;
    is.read((char *)&NOdom, sizeof(NOdom));
    lenc.resize(NOdom);
    Serialize::readVecread(is, lenc);
    SetPreIntegrationList<EncData>(lenc.begin(), lenc.end());
    listeig(IMUData) limu;
    is.read((char *)&NOdom, sizeof(NOdom));
    limu.resize(NOdom);
    Serialize::readVecread(is, limu);
    SetPreIntegrationList<IMUData>(limu.begin(), limu.end());
  }
  // Compute/Recover mOdomPreIntOdom, mpPrevKeyFrame already exists for KFs of mpMap is sorted through nid_
  if (mpPrevKeyFrame) {
    PreIntegration<EncData>(mpPrevKeyFrame);
    PreIntegration<IMUData>(mpPrevKeyFrame);
  }
  is.read(&mState, sizeof(mState));
  // we've loaded mNavState in Frame
  return is.good();
}
bool KeyFrame::write(ostream &os) const {
  if (!FrameBase::write(os)) return false;

  {  // save mNavState
    const double *pdData;
    unique_lock<mutex> lock(mMutexNavState);
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

  // we add extra info for KF at the end for KeyFrame::write & Frame::read+KeyFrame::read
  {  // save odom lists
    unique_lock<mutex> lock(mMutexOdomData);
    const listeig(EncData) &lenc = mOdomPreIntEnc.getlOdom();
    size_t NOdom = lenc.size();
    os.write((char *)&NOdom, sizeof(NOdom));
    Serialize::writeVecwrite(os, lenc);
    const listeig(IMUData) &limu = mOdomPreIntIMU.getlOdom();
    NOdom = limu.size();
    os.write((char *)&NOdom, sizeof(NOdom));
    Serialize::writeVecwrite(os, limu);
    // we don't save mOdomPreIntOdom for it can be computed from the former odom list
  }
  os.write(&mState, sizeof(mState));
  return os.good();
}

// created by zzh over.

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, bool copy_shallow, KeyFrame *pPrevKF, const char state)
    : FrameBase(F),
      mnFrameId(F.nid_),
      mnTrackReferenceForFrame(0),
      mnFuseTargetForKF(0),
      mnBALocalForKF(0),
      mnBAFixedForKF(0),
      mnLoopQuery(0),
      mnLoopWords(0),
      mnRelocQuery(0),
      mnRelocWords(0),
      mnBAGlobalForKF(0),
      mpKeyFrameDB(pKFDB),  // for mK won't be changed, it cannot be used as clone()
      mbFirstConnection(true),
      mpParent(nullptr),
      mbNotErase(false),
      mbToBeErased(false),
      mpMap(pMap),
      mState(state),
      mbPrior(false) {
  if (!copy_shallow) mDescriptors = F.mDescriptors.clone();

  if (pPrevKF) pPrevKF->SetNextKeyFrame(this);
  mpPrevKeyFrame = pPrevKF;
  mpNextKeyFrame = nullptr;  // zzh, constructor doesn't need to lock mutex
  // Set bias as bias+delta_bias, and reset the delta_bias term
  mNavState.mbg += mNavState.mdbg;
  mNavState.mba += mNavState.mdba;
  // update bi (bi=bi+dbi) for a better PreIntegration of nextKF(localBA) & fixedlastKF
  // motion-only BA of next Frame(this won't optimize lastKF.mdbi any more)
  mNavState.mdbg = mNavState.mdba = Eigen::Vector3d::Zero();

  Tcw_.release();
  SetPose(F.GetTcwRef());
  PRINT_DEBUG_FILE_MUTEX("checkkf" << nid_ << " ", mlog::vieo_slam_debug_path, "debug.txt");
  size_t i = 0;
  for (auto iter : mvpMapPoints) {
    if (iter) PRINT_DEBUG_FILE_MUTEX(i << ":" << iter->mnId << ",", mlog::vieo_slam_debug_path, "debug.txt");
    ++i;
  }

  // move preint_odom_
  F.DeepMovePreintOdomFromLastKF(mOdomPreIntEnc);
  F.DeepMovePreintOdomFromLastKF(mOdomPreIntIMU);
  // created by zzh over

  nid_ = nNextId++;
}

void KeyFrame::SetPose(const cv::Mat &Tcw) {
  unique_lock<mutex> lock(mMutexPose);
  Tcw.copyTo(Tcw_);
  cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
  cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);
  cv::Mat Rwc = Rcw.t();
  Ow = -Rwc * tcw;

  Twc = cv::Mat::eye(4, 4, Tcw_.type());
  Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
  Ow.copyTo(Twc.rowRange(0, 3).col(3));
  cv::Mat center = (cv::Mat_<float>(4, 1) << stereoinfo_.baseline_bf_[0] / 2.f, 0, 0, 1);
  Cw = Twc * center;  // 4cm right of the Ow for kinect2
}

cv::Mat KeyFrame::GetPose() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw_.clone();
}

cv::Mat KeyFrame::GetPoseInverse() {
  unique_lock<mutex> lock(mMutexPose);
  return Twc.clone();
}
const Sophus::SE3d KeyFrame::GetTwc() {
  //  unique_lock<mutex> lock(mMutexPose);
  //  return FrameBase::GetTwc();
  return GetTcw().inverse();
}
const Sophus::SE3d KeyFrame::GetTcw() {
  unique_lock<mutex> lock(mMutexPose);
  return FrameBase::GetTcw();
}

cv::Mat KeyFrame::GetCameraCenter() {
  unique_lock<mutex> lock(mMutexPose);
  return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter() {
  unique_lock<mutex> lock(mMutexPose);
  return Cw.clone();
}

cv::Mat KeyFrame::GetRotation() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw_.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation() {
  unique_lock<mutex> lock(mMutexPose);
  return Tcw_.rowRange(0, 3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF] = weight;
    else if (mConnectedKeyFrameWeights[pKF] != weight)
      mConnectedKeyFrameWeights[pKF] = weight;
    else
      return;
  }

  UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
  unique_lock<mutex> lock(mMutexConnections);
  vector<pair<int, KeyFrame *>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
       mit != mend; mit++)
    vPairs.push_back(make_pair(mit->second, mit->first));

  sort(vPairs.begin(), vPairs.end());
  list<KeyFrame *> lKFs;
  list<int> lWs;
  for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  set<KeyFrame *> s;
  for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end();
       mit++)
    s.insert(mit->first);
  return s;
}

// notice now may get bad kfs
vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
  unique_lock<mutex> lock(mMutexConnections);
  return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
  unique_lock<mutex> lock(mMutexConnections);
  if ((int)mvpOrderedConnectedKeyFrames.size() < N)
    return mvpOrderedConnectedKeyFrames;
  else
    return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);
}

vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
  unique_lock<mutex> lock(mMutexConnections);

  if (mvpOrderedConnectedKeyFrames.empty()) return vector<KeyFrame *>();

  vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                                         KeyFrame::weightComp);  // first > w, here is first < w for weightComp is >
  if (it == mvOrderedWeights.end())
    return vector<KeyFrame *>();
  else {
    int n = it - mvOrderedWeights.begin();
    return vector<KeyFrame *>(
        mvpOrderedConnectedKeyFrames.begin(),
        mvpOrderedConnectedKeyFrames.begin() + n);  // n is right for the number of element whose value>=w
  }
}

int KeyFrame::GetWeight(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexConnections);
  if (mConnectedKeyFrameWeights.count(pKF))
    return mConnectedKeyFrameWeights[pKF];
  else
    return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  FrameBase::AddMapPoint(pMP, idx);
}

void KeyFrame::EraseMapPointMatch(const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  FrameBase::EraseMapPointMatch(idx);
}

void KeyFrame::EraseMapPointMatch(MapPoint *pMP) {
  set<size_t> idxs = pMP->GetIndexInKeyFrame(this);
  for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
    auto idx = *iter;
    mvpMapPoints[idx] = static_cast<MapPoint *>(nullptr);
    // PRINT_INFO_FILE_MUTEX("KFid" << nid_ << "Erase MP" << idx << endl, mlog::vieo_slam_debug_path, "debug.txt");
  }
}
void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) {
  unique_lock<mutex> lock(mMutexFeatures);
  FrameBase::ReplaceMapPointMatch(idx, pMP);
}
std::set<std::pair<MapPoint *, size_t>> KeyFrame::GetMapPointsCami() {
  unique_lock<mutex> lock(mMutexFeatures);
  return FrameBase::GetMapPointsCami();
}

int KeyFrame::TrackedMapPoints(const int &minObs) {
  unique_lock<mutex> lock(mMutexFeatures);

  int nPoints = 0;
  const bool bCheckObs = minObs > 0;
  for (int i = 0; i < N; i++) {
    MapPoint *pMP = mvpMapPoints[i];
    if (pMP) {
      if (!pMP->isBad()) {
        if (bCheckObs) {
          if (mvpMapPoints[i]->Observations() >= minObs) nPoints++;
        } else
          nPoints++;
      }
    }
  }

  return nPoints;
}

vector<MapPoint *> KeyFrame::GetMapPointMatches() {
  unique_lock<mutex> lock(mMutexFeatures);
  return FrameBase::GetMapPointMatches();
}

MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  return mvpMapPoints[idx];
}

void KeyFrame::FuseMP(size_t idx, MapPoint *pMP) {
  PRINT_DEBUG_FILE_MUTEX(nid_ << "fusemp", mlog::vieo_slam_debug_path, "debug.txt");
  // not wise to search replaced too deep if this replace is outlier or max_depth too large
#ifdef USE_SIMPLE_REPLACE
  if (!pMP || pMP->isBad()) return;
#else
  int depth = 0, depth_thresh = 5;
  while (pMP && pMP->isBad()) {
    pMP = pMP->GetReplaced();
    if (++depth >= depth_thresh) break;
  }
  if (!pMP || pMP->isBad()) return;
#endif

  // If there is already a MapPoint replace otherwise add new measurement
  MapPoint *pMPinKF = GetMapPoint(idx);
  if (pMPinKF) {
    if (!pMPinKF->isBad()) {
      // if pMP in pKF is better then discard pMP and use pMPinKF instead
      if (pMPinKF->Observations() > pMP->Observations())
        pMP->Replace(pMPinKF);
      else  // else replace pMPinKF with pMP
        pMPinKF->Replace(pMP);
    }     // TODO: maybe when it's bad, can fuse it as well, just add?
  } else  // if best feature match hasn't corresponding MP, then directly use the one in vec<MP*>
  {
    pMP->AddObservation(this, idx);
    PRINT_DEBUG_FILE_MUTEX("addmp3" << endl, mlog::vieo_slam_debug_path, "debug.txt");
    AddMapPoint(pMP, idx);
  }
}

void KeyFrame::UpdateConnections(KeyFrame *pLastKF) {
  map<KeyFrame *, int> KFcounter;

  vector<MapPoint *> vpMP;

  {
    unique_lock<mutex> lockMPs(mMutexFeatures);
    vpMP = mvpMapPoints;
  }

  // For all map points in keyframe check in which other keyframes are they seen
  // Increase counter for those keyframes
  for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
    MapPoint *pMP = *vit;

    if (!pMP) continue;

    if (pMP->isBad()) continue;

    map<KeyFrame *, set<size_t>> observations = pMP->GetObservations();
    for (auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      if (mit->first->nid_ == nid_) continue;
      KFcounter[mit->first]++;
    }
  }

  // This should not happen
  if (KFcounter.empty()) {  // ODOMOK;||mState!=2&&pLastKF!=NULL
    PRINT_INFO_MUTEX("Failed to update spanning tree! " << nid_ << " " << mnFrameId << endl);
    if (!pLastKF) {
      if (!mpParent) {
        assert(nid_ == 0);
      } else
        PRINT_INFO_MUTEX("but has 1 parent and " << mConnectedKeyFrameWeights.size() << " covisibility KFs" << endl);
    } else {
      // 	  pLastKF->AddConnection(this,0);//add the link from pLastKF to this
      // add the link from this to pLastKF
      // 	  KFcounter[pLastKF]=0;
      unique_lock<mutex> lockCon(mMutexConnections);
      // 	  mConnectedKeyFrameWeights=KFcounter;//?
      // 	  mvpOrderedConnectedKeyFrames.clear();
      // 	  mvOrderedWeights.clear();
      // 	  mvpOrderedConnectedKeyFrames.push_back(pLastKF);
      // 	  mvOrderedWeights.push_back(0);//0 means it's an Odom link

      // if first connected then update spanning tree
      // nid_!=0/this!=plastkf is important for 0th F/KF to ensure its parent is NULL!
      if (mbFirstConnection && nid_ != 0) {
        assert(this != pLastKF);
        mbFirstConnection = false;
        mpParent = pLastKF;  // the closer, the first connection is better
        mpParent->AddChild(this);
      }
    }
    return;
  }

  // If the counter is greater than threshold add connection
  // In case no keyframe counter is over threshold add the one with maximum counter
  int nmax = 0;
  KeyFrame *pKFmax = NULL;
  int th = 15;  // the least number of covisible MapPoints between two KFs to add connection, old 15

  vector<pair<int, KeyFrame *>> vPairs;
  vPairs.reserve(KFcounter.size());
  // finally we keep the unidirectional edge strategy for we don't want to change the pure RGBD part!
  for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; ++mit) {
    if (mit->second > nmax) {
      nmax = mit->second;
      pKFmax = mit->first;
    }
    if (mit->second >= th) {
      vPairs.push_back(make_pair(mit->second, mit->first));
      // notice here when <th but vPairs is not empty, it's only one directed edge in the covisibility graph!!! is this
      // right? I think it's wrong, so I added a revision
      (mit->first)->AddConnection(this, mit->second);
      // 	    ++mit;
    } /*else if (mit->first!=pKFmax){//we avoid one directional edge!
       mit=KFcounter.erase(mit);//revised by zzh, one original bug of ORBSLAM2!
     }else ++mit;*/
      //         (mit->first)->AddConnection(this,mit->second);
  }

  if (vPairs.empty()) {
    vPairs.push_back(make_pair(nmax, pKFmax));
    pKFmax->AddConnection(this, nmax);
  }

  sort(vPairs.begin(), vPairs.end());
  list<KeyFrame *> lKFs;
  list<int> lWs;
  for (size_t i = 0; i < vPairs.size(); i++) {
    lKFs.push_front(vPairs[i].second);  // notice here push_front not push_back!!!
    lWs.push_front(vPairs[i].first);
  }

  {
    unique_lock<mutex> lockCon(mMutexConnections);
    mConnectedKeyFrameWeights = KFcounter;
    mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

    if (mbFirstConnection && nid_ != 0) {
      mpParent = mvpOrderedConnectedKeyFrames.front();  // the closer, the first connection is better
      mpParent->AddChild(this);
      mbFirstConnection = false;
    }
  }
}

void KeyFrame::AddChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mpParent = pKF;
  pKF->AddChild(this);
}

set<KeyFrame *> KeyFrame::GetChilds() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens;
}

KeyFrame *KeyFrame::GetParent() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
  unique_lock<mutex> lockCon(mMutexConnections);
  mbNotErase = true;
  mspLoopEdges.insert(pKF);
}

set<KeyFrame *> KeyFrame::GetLoopEdges() {
  unique_lock<mutex> lockCon(mMutexConnections);
  return mspLoopEdges;
}

void KeyFrame::SetNotErase() {
  unique_lock<mutex> lock(mMutexConnections);
  mbNotErase = true;
}

void KeyFrame::SetErase() {
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mspLoopEdges.empty())  // if the pair of loop edges doesn't include this KF, it can be erased
    {
      mbNotErase = false;
    }
  }

  if (mbToBeErased) {
    SetBadFlag();  // if it's not the loop edges, then erased here when SetBadFlag() called during mbNotErase==true
  }
}

// this will be released in UpdateLocalKeyFrames() in Tracking, no memory leak(not be deleted) for
// bad KFs may be used by some Frames' trajectory retrieve
void KeyFrame::SetBadFlag(const int8_t mode) {
  assert(!mbBad);
  {
    unique_lock<mutex> lock(mMutexConnections);
    // for vkeys_/GetMapFrameMatches won't be changed in BA after set, so lock(mutex_features_) is useless
    // cannot erase the initial/fixed KF
    if (!(mode & ForceErase) && !nid_ && bcam_fixed_) return;
    // mbNotErase may be set true by LoopClosing
    if (mbNotErase) {
      mbToBeErased = true;  // wait to be erased in SetErase() by LoopClosing
      return;
    }
    // must set bad after id&not_erase check and before all data to be changed,
    // then we could 0:just isBad() after accessing all data may be changed when setbadflag or
    // 1: use some extra judgements
    mbBad = true;
  }
  assert(nid_);

  // Update Prev/Next KeyFrame in prev/next, mbBad is not absolutely related to its existence
  {
    // cout << "LockST..";
    unique_lock<mutex> lock(mstMutexPNChanging);
    // cout << "LockPN..";
    unique_lock<mutex> lock2(mMutexPNConnections);
    if (!mpPrevKeyFrame || !mpNextKeyFrame) {
      cerr << "Prev/Next KF is NULL!!!Please be aware of the reason!!!" << endl;
      mpMap->EraseKeyFrame(this);
      mpKeyFrameDB->erase(this);
      return;
    }
    assert(mpPrevKeyFrame->GetNextKeyFrame() == this && mpNextKeyFrame->GetPrevKeyFrame() == this);  // check 2!!!
    mpPrevKeyFrame->SetNextKeyFrame(
        mpNextKeyFrame);  // mpNextKeyFrame here cannot be NULL for mpCurrentKF cannot be erased in KFCulling()
    mpNextKeyFrame->SetPrevKeyFrame(mpPrevKeyFrame);  // 0th KF cannot be erased so mpPrevKeyFrame cannot be NULL
    // AppendIMUDataToFront, qIMU can speed up!
    {
      unique_lock<mutex> lock(mMutexOdomData);
      auto &lodom = mOdomPreIntIMU.GetRawDataRef();
      mpNextKeyFrame->AppendFrontPreIntegrationList<IMUData>(lodom, lodom.begin(), lodom.end());
    }
    // AppendEncDataToFront
    {
      unique_lock<mutex> lock(mMutexOdomData);
      auto &lodom = mOdomPreIntEnc.GetRawDataRef();
      mpNextKeyFrame->AppendFrontPreIntegrationList<EncData>(lodom, lodom.begin(), lodom.end());
    }
    // ComputePreInt
    auto ns_preint_new = mpPrevKeyFrame->GetNavState();
    {
      unique_lock<mutex> lock(mMutexNavState);
      ns_preint_new.mbg = mNavState.mbg + mNavState.mdbg;
      ns_preint_new.mba = mNavState.mba + mNavState.mdba;
    }
    ns_preint_new.mdbg.setZero();
    ns_preint_new.mdba.setZero();
    mpPrevKeyFrame->SetNavState(ns_preint_new);
    mpNextKeyFrame->PreIntegration<IMUData>(mpPrevKeyFrame);
    mpNextKeyFrame->PreIntegration<EncData>(mpPrevKeyFrame);
    mpPrevKeyFrame = mpNextKeyFrame = nullptr;  // clear this KF's pointer, to check if its prev/next is deleted
  }

  // erase features
  {
    unique_lock<mutex> lock(mMutexFeatures);
    for (size_t i = 0; i < mvpMapPoints.size(); i++)
      if (mvpMapPoints[i]) mvpMapPoints[i]->EraseObservation(this);  // erase this observation in this->mvpMapPoints
  }

  if (bcam_fixed_) {
    // erase the relation with this(&KF)
    for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
         mit != mend; mit++)
      // erase the directed edge from others to this (1 undirected edge <==> 2 directed edges)
      mit->first->EraseConnection(this);

    unique_lock<mutex> lock(mMutexConnections);

    // erase the directed edge from this to others in covisibility graph, this is also used as the interface
    mConnectedKeyFrameWeights.clear();
    // erase the directed edge for the interface GetVectorCovisibleKeyFrames() will use mvpOrderedConnectedKeyFrames,
    // but no mvOrderedWeights will be used as public functions
    mvpOrderedConnectedKeyFrames.clear();

    // KeepTree for LoadMap(), we don't change bad KFs' parent or Tcp for recovering in SaveTrajectoryTUM()
    auto &&pparent = mpParent;
    if (!pparent) {
      if (!(mode & ForceErase) || nid_) {
        PRINT_ERR_MUTEX("Error in update spanning tree");
        exit(-1);
      }
    } else {
      if (!(mode & KeepTree)) {
        // Update Spanning Tree
        set<KeyFrame *> sparent_candidates;
        sparent_candidates.insert(pparent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        auto &pchilds = mspChildrens;
        while (!mspChildrens.empty())  // if empty/all Bad -> no need to adjust the spanning tree more
        {
          bool bcontinue = false;

          int max = -1;
          KeyFrame *pchild;
          KeyFrame *pparent;

          for (set<KeyFrame *>::iterator sit = pchilds.begin(), send = pchilds.end(); sit != send; sit++) {
            KeyFrame *pkf = *sit;
            // here no need to consider multithread problem for SetBadFlag() is not for multithread,
            // which uses SetErase tech. in loop closing thread
            if (pkf->isBad()) continue;

            // Check if a parent candidate is connected to the keyframe (children of this)
            vector<KeyFrame *> vpconnected = pkf->GetVectorCovisibleKeyFrames();
            for (size_t i = 0, iend = vpconnected.size(); i < iend; i++) {
              for (set<KeyFrame *>::iterator spcit = sparent_candidates.begin(), spcend = sparent_candidates.end();
                   spcit != spcend; spcit++) {
                if (vpconnected[i]->nid_ == (*spcit)->nid_) {
                  // notice vpConnected[i]->GetWeight(pKF) may not exist for not in time
                  // vpConnected[i]->UpdateConnections()
                  int w = pkf->GetWeight(vpconnected[i]);
                  // the pair(pC,pP) highest covisibility weight
                  if (w > max) {
                    pchild = pkf;
                    pparent = vpconnected[i];
                    max = w;
                    bcontinue = true;
                  }
                }
              }
            }
          }

          // this updation(connecting culled KF's children with the KF's parent/children) is same as
          // mbFirstConnection(the closest covisibility KF)
          if (bcontinue) {
            // connect pchild to its new parent pparent(max covisibility in sparent_candidates)
            pchild->ChangeParent(pparent);
            // put pchild(max covisibility child correspoding to sparent_candidates) into sparent_candidates
            sparent_candidates.insert(pchild);
            pchilds.erase(pchild);
          } else  // if left children's 1st layer covisibility KFs have no sparent_candidates(max==-1) -> break
            break;
        }

        // The childs with no covisibility links/edges with any parent candidate, assign it to this kf's parent
        if (!pchilds.empty())
          for (auto sit = pchilds.begin(); sit != pchilds.end(); sit++) {
            (*sit)->ChangeParent(pparent);
          }
      } else
        assert(mode == KeepTree);
      // notice here mspChildrens may not be empty, and it doesn't take part in the propagation in LoopClosing thread
      pparent->EraseChild(this);
      if (!(mode & KeepTree)) {
        // maybe for bad kf's Tcw_ changes small, old code here no lock(mMutexPose); but for safety, we add it here
        unique_lock<mutex> lock(mMutexPose);
        mTcp = Tcw_ * pparent->GetPoseInverse();  // the inter spot/link of Frames with its refKF in spanning tree
      }
    }
  }

  if (bcam_fixed_) {
    mpKeyFrameDB->erase(this);
    if (static_cast<bool>(mode & MapNoErase)) return;
    mpMap->EraseKeyFrame(this);
  }
}

bool KeyFrame::isBad() {
  unique_lock<mutex> lock(mMutexConnections);
  return FrameBase::isBad();
}

void KeyFrame::EraseConnection(KeyFrame *pKF) {
  bool bUpdate = false;
  {
    unique_lock<mutex> lock(mMutexConnections);
    if (mConnectedKeyFrameWeights.count(pKF)) {
      mConnectedKeyFrameWeights.erase(pKF);
      bUpdate = true;
    }
  }

  if (bUpdate) UpdateBestCovisibles();
}

cv::Mat KeyFrame::UnprojectStereo(int i) {
  if (mapn2in_.size() > i) {
    const float z = stereoinfo_.vdepth_[i];
    if (z > 0) {
      auto ididxs = GetMapn2idxs(i);
      CV_Assert(-1 != ididxs && stereoinfo_.goodmatches_[ididxs]);

      unique_lock<mutex> lock(mMutexPose);
      Vector3d x3Dw =
          Converter::toMatrix3d(Twc.rowRange(0, 3).colRange(0, 3)) * (GetTcr() * stereoinfo_.v3dpoints_[ididxs]) +
          Converter::toVector3d(Twc.rowRange(0, 3).col(3));
      return Converter::toCvMat(x3Dw);
    } else {
      return cv::Mat();
    }
  }

  const float z = stereoinfo_.vdepth_[i];
  if (z > 0) {
    assert(!usedistort_);
    const float u = mvKeysUn[i].pt.x;
    const float v = mvKeysUn[i].pt.y;
    Vector3f uv_normal = mpCameras[0]->toK().cast<float>().inverse() * Vector3f(u, v, 1);
    const float x = uv_normal[0] * z;
    const float y = uv_normal[1] * z;
    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

    unique_lock<mutex> lock(mMutexPose);
    return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
  } else
    return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
  vector<MapPoint *> vpMapPoints;
  cv::Mat Tcw;
  {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPose);
    vpMapPoints = mvpMapPoints;
    Tcw = Tcw_.clone();
  }

  vector<float> vDepths;
  vDepths.reserve(N);
  cv::Mat Rcw2 = Tcw.row(2).colRange(0, 3);
  Rcw2 = Rcw2.t();
  float zcw = Tcw.at<float>(2, 3);
  for (int i = 0; i < N; i++) {
    if (vpMapPoints[i]) {
      MapPoint *pMP = vpMapPoints[i];
      cv::Mat x3Dw = pMP->GetWorldPos();
      float z = Rcw2.dot(x3Dw) + zcw;
      vDepths.push_back(z);
    }
  }

  auto sz_depth = vDepths.size();
  if (!sz_depth) {
    return hist_med_depth_;  // like baseline_bf_[0] for mono to be 0.1m
  }
  sort(vDepths.begin(), vDepths.end());
  float med_d = vDepths[(vDepths.size() - 1) / q];
  const float alpha = 0.8;
  hist_med_depth_ = hist_med_depth_ * (1 - alpha) + med_d * alpha;
  return med_d;
}

}  // namespace VIEO_SLAM
