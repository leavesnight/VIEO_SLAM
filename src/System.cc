/**
 * This file is part of VIEO_SLAM
 */

#include <thread>
#include <iomanip>
#include <pangolin/pangolin.h>
#include "Optimizer.h"
#include "System.h"
#include "Converter.h"
#include "common/serialize/serialize.h"
#ifdef USE_PCL
#include "map/pcl/map_sl.h"
#endif

namespace VIEO_SLAM {
bool System::usedistort_ = false;

cv::Mat System::TrackOdom(const double &timestamp, const double *odomdata, const char mode) {
  cv::Mat Tcw = mpTracker->CacheOdom(timestamp, odomdata, mode);

  return Tcw;
}
void System::FinalGBA(int nIterations, bool bRobust) {
  if (!nIterations) return;
  if (mpIMUInitiator->GetVINSInited()) {  // zzh, Full BA, GetVINSInited() instead of GetSensorIMU() for pure-vision+IMU
                                          // Initialization mode
    Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap, mpIMUInitiator->GetGravityVec(), nIterations, NULL, 0, bRobust,
                                                 true);
  } else
    Optimizer::GlobalBundleAdjustment(mpMap, nIterations, NULL, 0, bRobust, mpIMUInitiator->GetSensorEnc());
}

void System::SaveKeyFrameTrajectoryNavState(const string &filename, bool bUseTbc) {
  PRINT_INFO_MUTEX(endl << "Saving keyframe NavState to " << filename << " ..." << endl);
  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  //     sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);//set of KFs in Map is already sorted, so it's useless

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  // cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];
    // pKF->SetPose(pKF->GetPose()*Two);

    if (pKF->isBad()) continue;

    // For VIO, we should compare the Pose of B/IMU Frame!!! not the Twc but the Twb! with EuRoC's Twb_truth(using
    // Tb_prism/Tbs from vicon0/data.csv) (notice vicon0 means the prism's Pose), and I found
    // state_groundtruth_estimate0 is near Twb_truth but I don't think it's truth!
    if (bUseTbc) pKF->UpdateNavStatePVRFromTcw();  // for Monocular
    NavState ns = pKF->GetNavState();
    Eigen::Quaterniond q = ns.mRwb.unit_quaternion();  // qwb from Rwb
    Eigen::Vector3d t = ns.mpwb;                       // twb
    Eigen::Vector3d v = ns.mvwb, bg = ns.mbg + ns.mdbg, ba = ns.mba + ns.mdba;
    f << setprecision(9) << pKF->ftimestamp_ << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " "
      << q.y() << " " << q.z() << " " << q.w() << " " << v(0) << " " << v(1) << " " << v(2) << " " << bg(0) << " "
      << bg(1) << " " << bg(2) << " " << ba(0) << " " << ba(1) << " " << ba(2) << endl;
  }

  f.close();
  PRINT_INFO_MUTEX(endl << "NavState trajectory saved!" << endl);
}
bool System::LoadMap(const string &filename, bool bPCL, bool bReadBadKF) {
  if (bPCL) return false;
  PRINT_INFO_MUTEX(endl
                   << "Loading Map: 1st step...SensorType(static ones),Keyframe (F,PrevKF,BoW),NavState(Pose) from "
                   << filename << " ..." << endl);
  ifstream f;
  f.open(filename.c_str(), ios_base::in | ios_base::binary);
  if (!f.is_open()) {
    cout << redSTR << "Opening Map Failed!" << whiteSTR << endl;
    return false;
  }
  char sensorType = 0;  // 0 for nothing/pure visual SLAM, 1 for encoder/VEO, 2 for IMU/VIO, 3 for encoder+IMU/VIEO
  f.read(&sensorType, sizeof(sensorType));
  if (f.bad()) {
    f.close();
    PRINT_INFO_MUTEX(redSTR << "Reading Map Failed!" << whiteSTR << endl);
    return false;
  }

  cout << (int)sensorType << "!Mode" << endl;
  if (sensorType == 1 || sensorType == 3) {
    EncData::readParam(f);
  }
  if (sensorType == 2 || sensorType == 3) {
    IMUData::readParam(f);
    cv::Mat gravityVec(3, 1, CV_32F);
    Serialize::readMat(f, gravityVec);
    mpIMUInitiator->SetGravityVec(gravityVec);
  }

  size_t NKFs;
  f.read((char *)&NKFs, sizeof(NKFs));
  list<unsigned long> lRefKFParentId;  // old parent id of mpTracker->mlpReferences
  if ((!mpViewer || !mpViewer->isFinished()) && !mpLocalMapper->Getfinish() && !mpLoopCloser->Getfinish()/* &&
      !mpIMUInitiator->Getfinish()*/) {
    mpTracker->Reset();
  } else {
    // now only suitable for loading current map
    if (bReadBadKF) {  // before clear KFs, we save the old id of mpTracker->mlpReferences
      list<KeyFrame *> &lRefKF = mpTracker->mlpReferences;
      for (list<KeyFrame *>::iterator iter = lRefKF.begin(), iterEnd = lRefKF.end(); iter != iterEnd; ++iter) {
        lRefKFParentId.push_back((*iter)->nid_);
      }
    }

    mpKeyFrameDatabase->clear();
    mpMap->clear();  // clear MPs,KFs & KFOrigins
    MapPoint::nNextId = 0;
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;  // new id of good MPs,KFs & Fs starts from 0
  }

  map<size_t, KeyFrame *> mapIdpKF;                         // make a map from nid_ (old) to KeyFrame*
  vector<vector<long unsigned int>> vpKFMPIdMatches(NKFs);  //<vpKFs.size()<cache matched MPs' id (old)>>
  size_t iFirstBad = NKFs;
  for (size_t i = 0; i < NKFs; ++i) {
    cv::Mat Tcp(4, 4, CV_32F);
    char bBad;
    if (bReadBadKF) {
      f.read(&bBad, sizeof(char));
      if (bBad) {
        Serialize::readMat(f, Tcp);
        if (iFirstBad == NKFs) iFirstBad = i;
      }
    }

    long unsigned int oldId;
    f.read((char *)&oldId, sizeof(oldId));  // old Id of KF
    long unsigned int prevId;
    f.read((char *)&prevId, sizeof(prevId));                       // old prevKF's Id
    if (prevId != ULONG_MAX) assert(mapIdpKF.count(prevId) == 1);  // 0<i<NKFsInit
    // NULL correponds to prevId==ULONG_MAX
    KeyFrame *pPrevKF = nullptr;
    if (prevId != ULONG_MAX) pPrevKF = mapIdpKF[prevId];
    Frame tmpF(f, mpVocabulary);
    // we use Serialize::read() corresponding to Serialize::write()
    KeyFrame *pKF = new KeyFrame(tmpF, mpMap, mpKeyFrameDatabase, pPrevKF, f);
    if (bReadBadKF && bBad) pKF->mTcp = Tcp;

    mapIdpKF[oldId] = pKF;
    size_t NMPMatches;
    f.read((char *)&NMPMatches, sizeof(NMPMatches));  // size of KeyPoints
    vpKFMPIdMatches[i].resize(NMPMatches);
    for (size_t j = 0; j < NMPMatches; ++j) {
      // MP's (old) id(if ULONG_MAX meaning unmatched)
      f.read((char *)&vpKFMPIdMatches[i][j], sizeof(vpKFMPIdMatches[i][j]));
    }

    mpMap->AddKeyFrame(pKF);  // Insert KeyFrame in the map
    if (i == 0) {             // ORB_SLAM2 just uses 0th KF/F as the KFOrigins
      assert(pKF->nid_ == 0 && oldId == 0);
      mpMap->mvpKeyFrameOrigins.push_back(pKF);
    }
  }

  PRINT_INFO_MUTEX("2nd step...MapPoint old Id & Position & refKFId & observations from " << filename << " ..."
                                                                                          << endl);
  map<size_t, MapPoint *> mapIdpMP;  // make a map from nid_ (old) to MapPoint*
  long unsigned int nlData;          // for id
  size_t NMPs;
  f.read((char *)&NMPs, sizeof(NMPs));  // size of observations
  for (size_t i = 0; i < NMPs; ++i) {
    long unsigned int oldId;
    f.read((char *)&oldId, sizeof(oldId));  // old Id
    // refKF's id (old), notice the KeyFrame*/address is different in LoadMap from SaveMap
    f.read((char *)&nlData, sizeof(nlData));
    assert(mapIdpKF.count(nlData) == 1);
    MapPoint *pMP = new MapPoint(mapIdpKF[nlData], mpMap, f);

    size_t Nobs;
    f.read((char *)&Nobs, sizeof(Nobs));  // size of observations/MPs
    for (size_t j = 0; j < Nobs; ++j) {
      f.read((char *)&nlData, sizeof(nlData));  // obs: KFj's id (old)
      assert(mapIdpKF.count(nlData) == 1);
      // obs: KFj's corresponding KeyPoint's ids/order of this MP
      size_t size_idxs;
      f.read((char *)&size_idxs, sizeof(size_idxs));
      for (size_t idxi = 0; idxi < size_idxs; ++idxi) {
        size_t idx;
        f.read((char *)&idx, sizeof(idx));
        pMP->AddObservation(mapIdpKF[nlData], idx);
      }
    }
    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();
    mpMap->AddMapPoint(pMP);

    mapIdpMP[oldId] = pMP;
  }

  cout << "3rd step...Add matched MapPoints to KeyFrames, Update Spanning Tree, AddLoopEdges, Add KeyFrameDatabase..."
       << endl;
  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();  // it's using the nid_ of KF as the order, so we must keep the
                                                        // new id has the same order as the old one
  for (size_t i = 0; i < NKFs; ++i) {                   // or vpKFMPIdMatches.size()
    KeyFrame *pKF = vpKFs[i];
    for (size_t j = 0; j < vpKFMPIdMatches[i].size(); ++j) {
      if (vpKFMPIdMatches[i][j] == ULONG_MAX) {  // unmatched
        pKF->EraseMapPointMatch(j);
      } else {
        assert(mapIdpMP.count(vpKFMPIdMatches[i][j]) == 1);
        pKF->AddMapPoint(mapIdpMP[vpKFMPIdMatches[i][j]], j);
      }
    }
    // Update Spanning Tree, must be after when mapIdpKF is made
    long unsigned int parentId, loopId;
    f.read((char *)&parentId, sizeof(parentId));  // old parent KF's Id
    if (i > 0) assert(parentId != ULONG_MAX);
    if (parentId != ULONG_MAX) {
      assert(mapIdpKF.count(parentId) == 1);
      pKF->ChangeParent(mapIdpKF[parentId]);
    }
    // Add LoopEdges
    size_t nLoops;
    f.read((char *)&nLoops, sizeof(nLoops));  // pKF->mspLoopEdges.size()
    for (size_t j = 0; j < nLoops; ++j) {
      f.read((char *)&loopId, sizeof(loopId));  // old loop KF's Id
      assert(mapIdpKF.count(loopId) == 1);
      pKF->AddLoopEdge(mapIdpKF[loopId]);
    }
    mpKeyFrameDatabase->add(pKF);
  }

  cout << "4th step...Update Covisible Graph(Only/Without Spanning Tree)...";
  for (size_t i = 0; i < NKFs; ++i) {
    KeyFrame *pKF = vpKFs[i];
    // Update Covisible Graph(mbFirstConnection==false!), it needs pKF->mvpMapPoints & pMP->mObservations
    pKF->UpdateConnections();
  }

  if (bReadBadKF) {  // we delete bad KFs and correct mpTracker->mlpReferences
    for (size_t i = iFirstBad; i < NKFs; ++i) {
      // i>= NKFsInit; we need keep bad KFs' parent & Tcp unchanged for SaveTrajectoryTUM()!!!
      vpKFs[i]->SetBadFlag(KeyFrame::KeepTree);
    }
    list<KeyFrame *> &lRefKF = mpTracker->mlpReferences;
    list<unsigned long>::iterator iterID = lRefKFParentId.begin();
    for (list<KeyFrame *>::iterator iter = lRefKF.begin(), iterEnd = lRefKF.end(); iter != iterEnd; ++iter, ++iterID) {
      assert(mapIdpKF.count(*iterID) == 1);
      *iter = mapIdpKF[*iterID];  // old KF's id to its new corresponding KF*
    }
  }

  if (iFirstBad > 0) {
    mpTracker->mState = Tracking::MAP_REUSE;
    mpTracker->SetInitLastKeyFrame(vpKFs[iFirstBad - 1]);
    // for NeedNewKeyFrame() judge
    mpTracker->SetInitReferenceKF(vpKFs[iFirstBad - 1]);
    // to avoid first frame after LoadMap() to be keyframe, causing UpdateConnections() assert bug
    mpLocalMapper->SetInitLastCamKF(vpKFs[iFirstBad - 1]);
    if (sensorType >= 2) {
      // though we don't need to init when loading a V(I)(E)O map, but user may reset when no localization mode!
      // if (!mpIMUInitiator->Getfinish()) mpIMUInitiator->Setfinish_request(true);
      mpIMUInitiator->SetInitGBAOver(true);  // gravity & scale should already be inited
      mpIMUInitiator->SetVINSInited(true);
    }
  }

  cout << "Over" << endl;
  f.close();
  return true;
}
// maybe can be rewritten in Tracking.cc
void System::SaveMap(const string &filename, bool bPCL, bool bUseTbc, bool bSaveBadKF) {
  if (filename.empty()) return;
  if (!bPCL) {
    // save sparse map for Map ReUse
    PRINT_INFO_MUTEX(
        endl
        << "Saving Map: 1st step...SensorType(static ones),Keyframe NavState & matched MapPoints' old Id to "
        << filename << " ..." << endl);
    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    ofstream f;
    f.open(filename.c_str(), ios_base::out | ios_base::binary);

    char sensorType = 0;  // 0 for nothing/pure visual SLAM, 1 for encoder/VEO, 2 for IMU/VIO, 3 for encoder+IMU/VIEO
    if (mpIMUInitiator->GetSensorEnc()) ++sensorType;
    if (mpIMUInitiator->GetVINSInited()) sensorType += 2;
    PRINT_INFO_MUTEX("MapType=" << (int)sensorType << endl);
    f.write(&sensorType, sizeof(sensorType));
    if (sensorType == 1 || sensorType == 3) {  // notice here we should always keep parameters same as before
      EncData::writeParam(f);
    }
    if (sensorType == 2 || sensorType == 3) {
      IMUData::writeParam(f);
      Serialize::writeMat(f, mpIMUInitiator->GetGravityVec());
    }

    long unsigned int nlData;
    const long unsigned int *pnlData;  // for id
    size_t NKFs = vpKFs.size();
    size_t NKFsInit = NKFs;
    if (bSaveBadKF) {
      set<KeyFrame *> spKFs;
      list<KeyFrame *> &lRefKF = mpTracker->mlpReferences;
      for (list<KeyFrame *>::iterator iter = lRefKF.begin(), iterEnd = lRefKF.end(); iter != iterEnd; ++iter) {
        KeyFrame *pKF = *iter;
        while (pKF->isBad()) {  // here we save all bad but useful KFs
          if (spKFs.count(pKF) == 0) spKFs.insert(pKF);
          pKF = pKF->GetParent();
        }
      }
      for (set<KeyFrame *>::iterator iter = spKFs.begin(), iterEnd = spKFs.end(); iter != iterEnd; ++iter) {
        vpKFs.push_back(*iter);
      }
      NKFs = vpKFs.size();
    }
    f.write((char *)&NKFs, sizeof(NKFs));  // write NKFs first
    for (size_t i = 0; i < NKFs; ++i) {
      KeyFrame *pKF = vpKFs[i];
      if (bSaveBadKF) {
        char bBad = 0;
        if (pKF->isBad()) {
          bBad = 1;
          f.write(&bBad, sizeof(bBad));
          Serialize::writeMat(f, pKF->mTcp);
          assert(i >= NKFsInit);
          //        cout<<i<<" "<<pKF->nid_<<" "<<pKF->GetParent()->nid_<<endl;
        } else
          f.write(&bBad, sizeof(bBad));
      } else if (pKF->isBad())
        continue;

      pnlData = &pKF->nid_;
      f.write((char *)pnlData, sizeof(*pnlData));  // save old Id of KF
      KeyFrame *pPrevKF = pKF->GetPrevKeyFrame();
      if (pPrevKF == NULL)
        nlData = ULONG_MAX;
      else
        nlData = pPrevKF->nid_;
      f.write((char *)&nlData, sizeof(nlData));  // save old prevKF's Id, NULL is ULONG_MAX
      // For VIO, we should compare the Pose of B/IMU Frame!!! not the Twc but the Twb! with EuRoC's Twb_truth(using
      // Tb_prism/Tbs from vicon0/data.csv) (notice vicon0 means the prism's Pose), and I found
      // state_groundtruth_estimate0 is near Twb_truth but I don't think it's truth!
      if (bUseTbc) pKF->UpdateNavStatePVRFromTcw();  // for Monocular
      pKF->write(f);

      vector<MapPoint *> vpMPMatches = pKF->GetMapPointMatches();
      size_t NMPMatches = vpMPMatches.size();
      f.write((char *)&NMPMatches, sizeof(NMPMatches));  // size of KeyPoints
      for (size_t j = 0; j < NMPMatches; ++j) {
        if (vpMPMatches[j] == NULL || vpMPMatches[j]->isBad())
          nlData = ULONG_MAX;  // unmatched MPs' id
        else
          nlData = vpMPMatches[j]->mnId;  // matched MPs' id (old)
        f.write((char *)&nlData, sizeof(nlData));
      }
    }

    PRINT_INFO_MUTEX("2nd step...MapPoint's old Id & Position & refKFId & observations to " << filename << " ..."
                                                                                            << endl);
    // all MPs/KFs in mpMap are not bad!
    vector<MapPoint *> vpMPs = mpMap->GetAllMapPoints();
    size_t NMPs = vpMPs.size();
    f.write((char *)&NMPs, sizeof(NMPs));  // size of observations/MPs
    for (size_t i = 0; i < NMPs; ++i) {
      MapPoint *pMP = vpMPs[i];
      assert(pMP && !(pMP->isBad()));

      pnlData = &pMP->mnId;
      f.write((char *)pnlData, sizeof(*pnlData));  // old Id
      pnlData = &pMP->GetReferenceKeyFrame()->nid_;
      f.write((char *)pnlData, sizeof(*pnlData));  // refKF's id, must be before MapPoint::write()
      pMP->write(f);
      assert(!(pMP->GetReferenceKeyFrame()->isBad()));
      auto observations = pMP->GetObservations();  // observations
      size_t Nobs = observations.size();
      f.write((char *)&Nobs, sizeof(Nobs));  // size of observations
      for (auto mit = observations.begin(), mend = observations.end(); mit != mend; ++mit) {
        assert(!(mit->first->isBad()));
        pnlData = &mit->first->nid_;
        f.write((char *)pnlData, sizeof(*pnlData));  // obs: KFj's id (old)
        auto idxs = mit->second;
        size_t size_idxs = idxs.size();
        f.write((char *)&size_idxs, sizeof(size_idxs));
        for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
          auto idx = *iter;
          f.write((char *)&idx, sizeof(idx));  // obs: KFj's corresponding KeyPoint's id of this MP
        }
      }
    }

    cout << "3rd step...Save Parent KFs, LoopEdges...";
    for (size_t i = 0; i < NKFs; ++i) {  // or vpKFMPIdMatches.size()
      KeyFrame *pKF = vpKFs[i];
      // Update Spanning Tree, must be after when mapIdpKF is made
      KeyFrame *pParentKF = pKF->GetParent();
      if (pParentKF == NULL)
        nlData = ULONG_MAX;
      else
        nlData = pParentKF->nid_;
      f.write((char *)&nlData, sizeof(nlData));  // old parent KF's Id, NULL is ULONG_MAX
      // Add LoopEdges
      set<KeyFrame *> spLoopEdges = pKF->GetLoopEdges();
      size_t nLoops = spLoopEdges.size();
      f.write((char *)&nLoops, sizeof(nLoops));  // pKF->mspLoopEdges.size()
      for (set<KeyFrame *>::const_iterator iter = spLoopEdges.begin(); iter != spLoopEdges.end(); ++iter) {
        pnlData = &(*iter)->nid_;
        f.write((const char *)pnlData, sizeof(*pnlData));  // old loop KF's Id
      }
    }

    cout << "Over" << endl;
    f.close();
    return;
  }

#ifdef USE_PCL
  pcl::SaveMapPCL(filename, mSensor, mpMap, fsSettings);
#endif
  return;
}
#include <sys/stat.h>
#include <sys/types.h>
void System::SaveFrame(string foldername, const cv::Mat &im, const cv::Mat &depthmap, double tm_stamp) {
  if (foldername[foldername.length() - 1] != '/') foldername += '/';
  string rgbname = foldername + "rgb/", depthname = foldername + "depth/";
  static bool bInit = false;
  if (!bInit) {
    if (access(depthname.c_str(), 0) == -1) {
      cout << depthname << " not exists!" << endl;
      if (mkdir_p(depthname, 0777) == -1) cout << "depth mkdir error" << endl;
    }
    if (access(rgbname.c_str(), 0) == -1) {
      cout << rgbname << " not exists!" << endl;
      if (mkdir_p(rgbname, 0777) == -1) cout << "rgb mkdir error" << endl;
    } else if (access(depthname.c_str(), 0) == 0) {
      ofstream fout(foldername + "odometrysensor.txt");
      fout << "# odometry data\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp vl vr qx qy qz qw" << endl;
      fout.close();
      fout.open(foldername + "groundtruth.txt");
      fout << "# ground truth trajectory\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp tx ty tz qx qy qz qw" << endl;
      fout.close();
      bInit = true;
    }
  }
  char ch[25];                     // at least 10+1+6+4+1=22
  sprintf(ch, "%.6f.", tm_stamp);  // mpTracker->mCurrentFrame.ftimestamp_);
  rgbname = rgbname + ch + "bmp";
  depthname = depthname + ch + "png";

  cv::imwrite(rgbname, im);
  cv::imwrite(depthname, depthmap);
  /*ofstream fout(foldername+"odometrysensor.txt",ios_base::app);
  fout<<fixed;
  fout<<setprecision(6)<<tm_stamp<<" "<<setprecision(3);
  int num_tmp=6;
  for (int i=0;i<num_tmp-1;++i)
    fout<<data[i]<<" ";
  fout<<data[num_tmp-1]<<endl;
  fout.close();
  fout.open(foldername+"groundtruth.txt",ios_base::app);
  fout<<fixed;
  fout<<setprecision(6)<<tm_stamp<<setprecision(4);
  for (int i=0;i<7;++i){
    fout<<" "<<data[2+i];
  }
  fout<<endl;
  fout.close();*/
}
int System::mkdir_p(string foldername, int mode) {
  if (foldername.empty()) return -1;
  if (mkdir(foldername.c_str(), mode) == -1) {
    string::size_type pos = string::npos;
    if (foldername[foldername.length() - 1] == '/') pos = foldername.length() - 2;
    if (mkdir_p(foldername.substr(0, foldername.rfind('/', pos)), mode) == -1)
      return -1;
    else
      return mkdir(foldername.c_str(), mode);
  } else
    return 0;
}

// created by zzh over.

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer,
               const string &map_sparse_name)
    : mSensor(sensor) {
  // Output welcome message
  PRINT_INFO_MUTEX(endl
                   << "VIEO_SLAM Copyright (C) 2016-2018 Zhanghao Zhu, University of Tokyo." << endl
                   << "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl
                   << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
                   << "This is free software, and you are welcome to redistribute it" << endl
                   << "under certain conditions. See LICENSE.txt." << endl
                   << endl);
  PRINT_INFO_MUTEX("Input sensor was set to: ");
  if (mSensor == MONOCULAR)
    PRINT_INFO_MUTEX("Monocular" << endl);
  else if (mSensor == STEREO)
    PRINT_INFO_MUTEX("Stereo" << endl);
  else if (mSensor == RGBD)
    PRINT_INFO_MUTEX("RGB-D" << endl);
  CLEAR_INFO_FILE("start recording alg_event log:" << endl, mlog::vieo_slam_debug_path, "alg_event.txt");

  // Check settings file
  // cv::FileStorage
  fsSettings.open(strSettingsFile.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    PRINT_ERR_MUTEX("Failed to open settings file at: " << strSettingsFile << endl);
    exit(-1);
  }
  // Load ORB Vocabulary
  PRINT_INFO_MUTEX(endl << "Loading ORB Vocabulary. This could take a while..." << endl);
  mpVocabulary = new ORBVocabulary();
  bool bVocLoad = false;
  if (strVocFile.rfind(".txt") != string::npos) {
    bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    // mpVocabulary->saveToBinaryFile(strVocFile.substr(0,strVocFile.rfind(".txt"))+".bin");
  } else
    bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
  if (!bVocLoad) {
    PRINT_ERR_MUTEX("Wrong path to vocabulary. " << endl);
    PRINT_ERR_MUTEX("Falied to open at: " << strVocFile << endl);
    exit(-1);
  }
  PRINT_INFO_MUTEX("Vocabulary loaded!" << endl << endl);

  // Create KeyFrame Database
  mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

  // Create the Map
  mpMap = new Map();

  // Create Drawers. These are used by the Viewer
  auto node_tmp = fsSettings["Viewer.MaxCamsNum"];
  if (node_tmp.empty() || (int)node_tmp) {
    mpFrameDrawer = new FrameDrawer(mpMap);
  }
  node_tmp = fsSettings["Viewer.MapDrawer"];
  if (node_tmp.empty() || (int)node_tmp) {
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
  }

  // Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this constructor)
  mpTracker =
      new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

  // Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR, strSettingsFile);

  // Initialize the Loop Closing thread and launch
  mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR, strSettingsFile);

  // Initialize the Viewer thread and launch
  if (bUseViewer && (mpFrameDrawer || mpMapDrawer)) {
    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);
  }

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  // created by zzh
  // Initialize the IMU Initialization thread and launch
  mpIMUInitiator = new IMUInitialization(mpMap, mSensor == MONOCULAR, strSettingsFile);
  // Set pointers between threads
  mpTracker->SetIMUInitiator(mpIMUInitiator);
  mpLocalMapper->SetIMUInitiator(mpIMUInitiator);
  mpLoopCloser->SetIMUInitiator(mpIMUInitiator);
  mpIMUInitiator->SetLocalMapper(mpLocalMapper);  // for Stop LocalMapping thread&&NeedNewKeyFrame() in Tracking thread

  if (!map_sparse_name.empty()) {
    map_name_[0] = map_name_[2] = map_sparse_name;
    if (LoadMap(map_name_[2])) {
      if (mpViewer)
        mpViewer->blocalization_mode_.store(true);
      else
        ActivateLocalizationMode();
    }
  }

  // bind to assigned core
#if defined(SET_AFFINITY_LINUX)
  {
    multithread::ThreadPolicyInfo event_info;
    const string thread_type = "FE";
    auto node_tmp = fsSettings[thread_type + ".processor_ids"];
    size_t num_cores = sysconf(_SC_NPROCESSORS_CONF);
    event_info.affinity_mask_ = node_tmp.empty() ? ((size_t)(0x1 << num_cores) - 1) : (size_t)(int)node_tmp;
    node_tmp = fsSettings[thread_type + ".priority"];
    event_info.priority_ = node_tmp.empty() ? 48 : (size_t)(int)node_tmp;
    event_info.thread_type_ = multithread::THREAD_FE;
    int priority_max_rr = sched_get_priority_max(SCHED_RR);
    if (event_info.priority_ > priority_max_rr) {
      PRINT_INFO_FILE_MUTEX("th_name=" << (int)event_info.thread_type_
                                       << ",SCHED_FIFO, priority_min/max_rr=" << sched_get_priority_min(SCHED_RR) << "/"
                                       << priority_max_rr << ",min/max_fifo=" << sched_get_priority_min(SCHED_FIFO)
                                       << "/" << sched_get_priority_max(SCHED_FIFO) << std::endl,
                            VIEO_SLAM::mlog::vieo_slam_debug_path, "alg_event.txt");
      event_info.policy_ = SCHED_FIFO;
      event_info.priority_ -= priority_max_rr;
    } else
      event_info.policy_ = SCHED_RR;
    multithread::SetAffinity(event_info);
  }
#endif
}

cv::Mat System::TrackStereo(const vector<cv::Mat> &ims, const double &timestamp) {
  auto sz_ims = ims.size();
  switch (mSensor) {
    case MONOCULAR:
      if (sz_ims < 1) {
        PRINT_ERR_MUTEX("ERROR: you called TrackStereo(Mono) but input image size is 0." << endl);
        exit(-1);
      }
      break;
    case STEREO:
    case RGBD:
      if (sz_ims < 2) {
        PRINT_ERR_MUTEX("ERROR: you called TrackStereo(Stereo/RGB-D) but input image size < 2." << endl);
        exit(-1);
      }
      break;
    default:
      PRINT_ERR_MUTEX("ERROR: you called TrackStereo but input sensor was not set to STEREO/RGBD/MONOCULAR." << endl);
      exit(-1);
  }

  // Check mode change
  bool bload_map_tmp = false;
  {
    unique_lock<mutex> lock(mutex_mode_);
    if (bactivate_localization_mode_ || bsave_map_) {
      mpLocalMapper->RequestStop();
      mpIMUInitiator->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped() || !mpIMUInitiator->isStopped()) {
        usleep(1000);
      }

      if (bactivate_localization_mode_) mpTracker->InformOnlyTracking(true);
      if (bsave_map_) {
        SaveMap(map_name_[0]);
        SaveMap(map_name_[1], true);
        bsave_map_ = false;
        if (!bactivate_localization_mode_) {
          mpIMUInitiator->Release();
          mpLocalMapper->Release();
        }
      }
      if (bactivate_localization_mode_) bactivate_localization_mode_ = false;
    }
    if (bdeactivate_localization_mode_) {
      mpTracker->InformOnlyTracking(false);
      mpIMUInitiator->Release();
      mpLocalMapper->Release();
      bdeactivate_localization_mode_ = false;
    }
    bload_map_tmp = bload_map_;
  }
  // we cannot stuck viewer for Tracker Reset require it Stop()!
  if (bload_map_tmp) {
    bool blocalmap_release_tmp = false;
    if (mpLocalMapper->isStopped()) {
      mpLocalMapper->Release();
      blocalmap_release_tmp = true;
    }
    LoadMap(map_name_[2]);
    if (mpTracker->mbOnlyTracking) {
      if (blocalmap_release_tmp) {
        mpLocalMapper->RequestStop();
        mpIMUInitiator->RequestStop();
        // IMUInit thread stop for VINSInited state won't be suddenly change by it after InformOnlyTracking
        while (!mpLocalMapper->isStopped() || !mpIMUInitiator->isStopped()) {
          usleep(1000);
        }
      }
      mpTracker->InformOnlyTracking(true);
    }
    unique_lock<mutex> lock(mutex_mode_);
    bload_map_ = false;
  }

  // Check reset
  bool breset_tmp = false;
  {
    unique_lock<mutex> lock(mMutexReset);
    breset_tmp = mbReset;
  }
  // we cannot stuck viewer for Tracker Reset require it Stop()!
  if (breset_tmp) {
    mpTracker->Reset();
    if (breset_tmp && breset_smart_ && !map_name_[2].empty()) {
      if (LoadMap(map_name_[2])) {
        if (mpViewer)
          mpViewer->blocalization_mode_.store(true);
        else
          ActivateLocalizationMode();
      }
    }
    unique_lock<mutex> lock(mMutexReset);
    mbReset = false;
  }

  cv::Mat Tcw = mpTracker->GrabImageStereo(ims, timestamp);

  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.GetMapPointMatches();
  return Tcw;
}

void System::ActivateLocalizationMode() {
  unique_lock<mutex> lock(mutex_mode_);
  bactivate_localization_mode_ = true;
  bdeactivate_localization_mode_ = false;
}
void System::DeactivateLocalizationMode() {
  unique_lock<mutex> lock(mutex_mode_);
  bdeactivate_localization_mode_ = true;
  bactivate_localization_mode_ = false;
}
void System::SaveMap() {
  unique_lock<mutex> lock(mutex_mode_);
  bsave_map_ = true;
}
void System::LoadMap() {
  unique_lock<mutex> lock(mutex_mode_);
  bload_map_ = true;
}

bool System::MapChanged() {
  static int n = 0;
  int curn = mpMap->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else
    return false;
}

void System::Reset(bool bsmart) {
  unique_lock<mutex> lock(mMutexReset);
  mbReset = true;
  breset_smart_ = bsmart;
}

void System::ShutdownViewer() {
  if (mpViewer) {
    mpViewer->RequestFinish();
    while (!mpViewer->isFinished()) usleep(5000);

    cv::destroyAllWindows();
    pangolin::DestroyWindow("VIEO_SLAM: Map Viewer");
  }
}
void System::Shutdown() {
  mpIMUInitiator->Setfinish_request(true);
  mpLocalMapper->Setfinish_request(true);
  mpLoopCloser->Setfinish_request(true);
  ShutdownViewer();

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->Getfinish() || !mpLoopCloser->Getfinish() || mpLoopCloser->isRunningGBA() ||
         !mpIMUInitiator->Getfinish()) {
    usleep(5000);
  }

  if (mpViewer) pangolin::BindToContext("VIEO_SLAM: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename, const bool imu_info, const bool bgravity_as_w) {
  PRINT_INFO_MUTEX(endl << "Saving camera trajectory to " << filename << " ..." << endl);
  /*if(mSensor==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
      return;
  }*/

  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  list<VIEO_SLAM::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                               lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL) continue;

    KeyFrame *pKF = *lRit;
    //        if (pKF->isBad()) continue;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    cv::Mat Tcw = (*lit) * Trw;
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    vector<float> q = Converter::toQuaternion(Rwc);

    if (bgravity_as_w && mpIMUInitiator && mpIMUInitiator->GetVINSInited()) {
      Vector3d gw = Converter::toVector3d(mpIMUInitiator->GetGravityVec());
      Vector3d gwn = gw / gw.norm();
      Vector3d gIn;
      gIn << 0, 0, 1;
      Vector3d a_wI = gIn.cross(gwn);
      Vector3d vhat = a_wI.normalized();  // notice that even norm_gIn=1 and norm_gwn=1, norm_a_wI may not be 1!
      double theta_wI_val = std::acos(gIn.dot(gwn));
      Sophus::SO3exd RIw = Sophus::SO3exd::exp(vhat * theta_wI_val).inverse();  // RwI=Exp(theta_wI)
      Rwc = Converter::toCvMat(Matrix3d(RIw.matrix() * Converter::toMatrix3d(Rwc)));
      q = Converter::toQuaternion(Rwc);
      twc = Converter::toCvMat(Vector3d((RIw.matrix() * Converter::toVector3d(twc))));
    }

    f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " "
      << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3];
    if (imu_info) {
      auto ns_kf_ref = pKF->GetNavState();
      Vector3d bg = ns_kf_ref.mbg + ns_kf_ref.mdbg;
      f << " " << bg(0) << " " << bg(1) << " " << bg(2);
      Vector3d ba = ns_kf_ref.mba + ns_kf_ref.mdba;
      f << " " << ba(0) << " " << ba(1) << " " << ba(2);
    }
    f << endl;
  }
  f.close();
  PRINT_INFO_MUTEX(endl << "trajectory saved!" << endl);
}
void System::SaveTrajectoryNavState(const string &filename, bool bUseTbc) {
  PRINT_INFO_MUTEX(endl << "Saving frame NavState to " << filename << " ..." << endl);
  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  //     sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);//set of KFs in Map is already sorted, so it's useless

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  list<VIEO_SLAM::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  list<Vector3d>::iterator lv = mpTracker->relative_frame_bvwbs_.begin();
  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                               lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++, ++lv) {
    if (*lbL) continue;

    KeyFrame *pKF = *lRit;
    //        if (pKF->isBad()) continue;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    cv::Mat Tcw = (*lit) * Trw;
    Matrix3d Rwc = Converter::toMatrix3d(Tcw.rowRange(0, 3).colRange(0, 3).t());
    Vector3d twc = -Rwc * Converter::toVector3d(Tcw.rowRange(0, 3).col(3));

    // For VIO, we should compare the Pose of B/IMU Frame, VO need bUseTbc = true for easy comparison
    NavState ns;
    if (bUseTbc) {
      cv::Mat Twb;
      Twb = Converter::toCvMatInverse(Frame::mTbc * Tcw);
      Eigen::Matrix3d Rwb = Converter::toMatrix3d(Twb.rowRange(0, 3).colRange(0, 3));
      Eigen::Vector3d twb = Converter::toVector3d(Twb.rowRange(0, 3).col(3));
      /*
      Eigen::Matrix3d Rw1 = Rwc;  // Rwbj_old/Rwb1
      Eigen::Vector3d Vw1 = *lv;  // Vw1/wV1=wvbj-1bj_old now bj_old/b1 is changed to bj_new/b2, wV2=wvbj-1bj_new
      // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1
      Eigen::Vector3d Vw2 = Rwb * Rw1.transpose() * Vw1;*/

      ns.mpwb = twb;
      ns.setRwb(Rwb);
      ns.mvwb = Rwb * (*lv);
    } else {
      ns.setRwb(Rwc);
      ns.mpwb = twc;
    }

    Eigen::Quaterniond q = ns.mRwb.unit_quaternion();  // qwb from Rwb
    Eigen::Vector3d t = ns.mpwb;                       // twb
    Eigen::Vector3d v = ns.mvwb, bg = ns.mbg + ns.mdbg, ba = ns.mba + ns.mdba;
    f << setprecision(9) << *lT << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " "
      << q.z() << " " << q.w() << " " << v(0) << " " << v(1) << " " << v(2) << " " << bg(0) << " " << bg(1) << " "
      << bg(2) << " " << ba(0) << " " << ba(1) << " " << ba(2) << endl;
  }
  f.close();
  PRINT_INFO_MUTEX(endl << "NavState trajectory saved!" << endl);
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename, const bool bgravity_as_w) {
  PRINT_INFO_MUTEX(endl << "Saving keyframe trajectory to " << filename << " ..." << endl);

  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin???here it's at the origin
  // cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (pKF->isBad()) continue;
    // 	cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
    //         while(pKF->isBad())
    //         {
    //             Trw = Trw*pKF->mTcp;
    //             pKF = pKF->GetParent();
    //         }
    //         Trw = Trw*pKF->GetPose();
    // 	pKF->SetPose(Trw);

    cv::Mat R = pKF->GetRotation().t();
    vector<float> q = Converter::toQuaternion(R);
    cv::Mat t = pKF->GetCameraCenter();
    if (bgravity_as_w && mpIMUInitiator && mpIMUInitiator->GetVINSInited()) {
      Vector3d gw = Converter::toVector3d(mpIMUInitiator->GetGravityVec());
      Vector3d gwn = gw / gw.norm();
      Vector3d gIn;
      gIn << 0, 0, 1;
      Vector3d a_wI = gIn.cross(gwn);
      Vector3d vhat = a_wI.normalized();  // notice that even norm_gIn=1 and norm_gwn=1, norm_a_wI may not be 1!
      double theta_wI_val = std::acos(gIn.dot(gwn));
      Sophus::SO3exd RIw = Sophus::SO3exd::exp(vhat * theta_wI_val).inverse();  // RwI=Exp(theta_wI)
      R = Converter::toCvMat(Matrix3d(RIw.matrix() * Converter::toMatrix3d(R)));
      q = Converter::toQuaternion(R);
      t = Converter::toCvMat(Vector3d((RIw.matrix() * Converter::toVector3d(t))));
    }
    f << setprecision(6) << pKF->ftimestamp_ << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " "
      << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
  }

  f.close();
  PRINT_INFO_MUTEX(endl << "trajectory saved!" << endl);
}

void System::SaveTrajectoryKITTI(const string &filename) {
  PRINT_INFO_MUTEX(endl << "Saving camera trajectory to " << filename << " ..." << endl);
  if (mSensor == MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
    return;
  }

  vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  cv::Mat Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
  // We need to get first the keyframe pose and then concatenate the relative transformation.
  // Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  list<VIEO_SLAM::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                               lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++) {
    VIEO_SLAM::KeyFrame *pKF = *lRit;

    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

    while (pKF->isBad()) {
      //  PRINT_INFO_MUTEX( "bad parent" << endl);
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    cv::Mat Tcw = (*lit) * Trw;
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " "
      << twc.at<float>(0) << " " << Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2)
      << " " << twc.at<float>(1) << " " << Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " "
      << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
  }
  f.close();
  PRINT_INFO_MUTEX(endl << "trajectory saved!" << endl);
}

int System::GetTrackingState() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackingState;
}

vector<MapPoint *> System::GetTrackedMapPoints() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

}  // namespace VIEO_SLAM
