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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "IMUInitialization.h"//zzh

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#ifdef USE_G2O_NEWEST
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#else
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#endif

namespace VIEO_SLAM
{

class IMUInitialization;//zzh, for they includes each other
  
class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class LoopClosing
{
public:
  unsigned long mnLastOdomKFId;//if >0 use loose loop detection for encoder error correction
  int mnIterations,mnInitIterations;//number of Iterations in Full BA / GBA
  
  bool mbLoopDetected;//for ros_mono_pub.cc
  
public:
  void CreateGBA();
  
//created by zzh
  
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;//pair<loop candidate KF's group,this group's consistency counter(has consecutive new KFs condition)>
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;//use Eigen::aligned_allocator<> when STL container's element uses Eigen::class for memory alignment
    //Eigen::class should in fact be the fixed-size vectorizable Eigen Objects see http://eigen.tuxfamily.org/dox-devel/group__TopicFixedSizeVectorizable.html
    //here Eigen::Quaterniond(in g2o::Sim3) is the reason why we use Eigen::aligned_allocator

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale, const string &strSettingPath);//zzh

    void SetTracker(Tracking* pTracker);
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetIMUInitiator(IMUInitialization *pIMUInitiator){mpIMUInitiator=pIMUInitiator;}//zzh

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);//mlpLoopKeyFrameQueue.push_back(pKF)(pKF->mnID!=0)

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);//GBA thread, call Optimizer::GBA, propagate the GBA optimized Pose and Pos to update all KFs' Pose and MPs' Pos \
    (including the new ones created in Tracking/LocalMapping which is running during Optimizer::GBA), notice during the propagation process and CorrectLoop(), no new KFs and MPs can be created

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    /*bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }//unused */

    void RequestFinish();

    bool isFinished();//mbFinished

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW//for we use g2o::Sim3(its data member has Eigen::Quaterniond), this macro is used for memory alignment/overloaded operator new

protected:

    bool CheckNewKeyFrames();//if KFs exist in mlpLoopKeyFrameQueue

    bool DetectLoop();//pop front of mlpLoopKeyFrameQueue, if not close in time from last CorrectLoop()-> query KeyFrameDatabase by minScore(mpORBVocabulary->score(mpCurrentKF->mBowVec,1st layer covisibility KFs.mBowVec)),\
    result is loop candidate KFs, make groups of them and their 1st layer covisibility KFs, only consecutive(>=3) new mpCurrentKFs detect consistent loop candidate KF's group(>=1 common/shared KFs) \
    then these mvpEnoughConsistentCandidates(consecutive consistent loop candidate KFs) will be made(used in ComputeSim3()) and mvConsistentGroups will be updated as well(for next DetectLoop()'s previous group consistency updation)

    bool ComputeSim3();//fix loop candidate KF, correct the mpCurrentKF's Pose as mScw/mg2oScw; \
    Flow: use mvpEnoughConsistentCandidates && SBBoW to get nice loopCandidateKF and its base vpMapPointMatches, use Sim3Solver to get a base S12 and add matches to vpMapPointMatches through SearchBySim3(), \
    then use Sim3Motion-only BA(OptimizeSim3) to optimize S12(->S1w/Scw,also delete outliers in vpMapPointMatches) and if nInliers is enough -> ,\
    make mvpLoopMapPoints from validated loop KF and its neighbors' mvpMapPoints, \
    finally SearchByProjection(mpCurrentKF,Scw,mvpLoopMapPoints(like local MPs),mvpCurrentMatchedPoints(recording validated loop KF's vpMapPointMatches)) to add matches(2 (may)different MPs) in mvpCurrentMatchedPoints and validate the number of total matches\

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);//Project mvpLoopMapPoints to mvpCurrentConnectedKFs/CorrectedPosesMap.first (by Scw/CorrectedPosesMap.second) and fuse(replace/add) matched MPs

    void CorrectLoop();//kill previous GlobalBA(GBA) && stop LocalMapping thread, update mvpCurrentConnectedKFs as mpCurrentKF && its all 1st layer covisibility KFs, \
    use corrected mg2oScw to correct mvpCurrentConnectedKFs' Pose and their observed MPs/mvpMapPoints' Pos, then firstly fuse(replace/add) mvpCurrentMatchedPoints[i] with mpCurrentKF->mvpMapPoints[i], \
    secondly call SearchAndFuse(), then make LoopConnections(new links from mvpCurrentConnectedKFs to loop KFs through CurrentConnectedKFs' covisibility graph updation) and \
    call PoseGraphOpt. by these LoopConnections, finally add loop edge to mpCurrentKF && mpMatchedKF, start GBA thread, recover LocalMapping thread and update mLastLoopKFid

    void ResetIfRequested();//blocking(5ms refreshing) mode
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();//mbFinishRequested
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;
    IMUInitialization* mpIMUInitiator;//zzh

    
    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    
    // Loop detector parameters
    float mnCovisibilityConsistencyTh;//here 3

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;//validated loop KF
    std::vector<ConsistentGroup> mvConsistentGroups;//used in DetectLoop()
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;//used in DetectLoop() && ComputeSim3()
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;////all 1st layer covisibility KFs of mpCurrentKF+mpCurrentKF, used in CorrectLoop()
    std::vector<MapPoint*> mvpCurrentMatchedPoints;//used in ComputeSim3(), enough BA inliers/successful RANSAC result's corresponding vpMapPointMatches(size==mpCurrentKF.mvpMapPoints.size(),may has nullptr), \
    after last SBP() in ComputeSim3(), mvpCurrentMatchedPoints[i] may !=nullptr when mpCurrentKF.mvpMapPoints[i]==nullptr
    std::vector<MapPoint*> mvpLoopMapPoints;//used in ComputeSim3() && SearchAndFuse(), validated loop KF and its neighbors' mvpMapPoints, (no nullptr)
    cv::Mat mScw;//opencv: ScurrentKF_world, corrected in ComputeSim3()
    g2o::Sim3 mg2oScw;//g2o: ScurrentKF_world, corrected in ComputeSim3()

    long unsigned int mLastLoopKFid;//updated at the last of CorrectLoop(), checked in DetectLoop()(avoid too frequent loop correction/close)

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    //bool mbFinishedGBA;//unused
    bool mbStopGBA;
    std::mutex mMutexGBA;//used in GBA thread
    std::thread* mpThreadGBA;//used in CorrectLoop()

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;//true in RGBD


    char mnFullBAIdx;//char/int may be better
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
