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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>

namespace VIEO_SLAM
{
void LoopClosing::CreateGBA()
{ 
  mpIMUInitiator->SetInitGBA(false);//avoid enter this func. twice when lock mMutexGBA after entered
  
  // If a Global Bundle Adjustment is running, abort it
  if(isRunningGBA()){
    //do nothing, for it must be the one just after IMU Initialization
  }else{
    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    assert(!mbStopGBA);//it's already false
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);
  }
}
  
//created by zzh

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const string &strSettingPath):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), //mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0),
    mpIMUInitiator(NULL),mnLastOdomKFId(0),mbLoopDetected(false)//zzh
{
    cv::FileStorage fSettings(strSettingPath,cv::FileStorage::READ);
    cv::FileNode fnIter[2]={fSettings["GBA.iterations"],fSettings["GBA.initIterations"]};
    if (fnIter[0].empty()||fnIter[1].empty()){
      mnInitIterations=15;//15 as the VIORBSLAM paper
      mnIterations=10;//default 10 for real-time nice responce
      cout<<redSTR"No iterations,use default 15(normal),15(init)"<<whiteSTR<<endl;
    }else{
      mnIterations=fnIter[0];
      mnInitIterations=fnIter[1];
    }
//created by zzh
  
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop()){//else no gw to calculate GBA //(mpIMUInitiator->GetVINSInited())&&
                // Compute similarity transformation [sR|t] 
                // In the stereo/RGBD case s=1
                unique_lock<mutex> lockScale(mpMap->mMutexScaleUpdateLoopClosing);//notice we cannot update scale during LoopClosing or LocalBA!
                if(ComputeSim3()){
                    // Perform loop fusion and pose graph optimization
                    CorrectLoop();
                }
            }
        }
        //for full BA just after IMU Initialized, zzh
        if (mpIMUInitiator&&mpIMUInitiator->GetInitGBA()){
            CreateGBA();
        }

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);//notice 3ms in LocalMapping thread
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread in this function
        if (mpCurrentKF->getState()==Tracking::ODOMOK){//it's quite rare for ODOMOK to close loop, so we just jump over it
            mnLastOdomKFId=mpCurrentKF->mnId;
            mnCovisibilityConsistencyTh=1;//like Relocalization()
            mpKeyFrameDB->add(mpCurrentKF);
            return false;
        }
        cout<<"SetNotErase"<<mpCurrentKF->mnId<<" "<<mpCurrentKF->mTimeStamp<<endl;
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection(CorrectLoop()), close in time from last loop
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);//add CurrentKF into KFDataBase
        cout<<"Too close, discard loop detection!"<<mpCurrentKF->mnId<<" "<<mpCurrentKF->mTimeStamp<<endl;
        mpCurrentKF->SetErase();//allow CurrentKF to be erased
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this (higher score)
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;//score is [0,1]
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }
    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);//returned KFs cannot be in vpConnectedKeyFrames(not made from score(BowVecs))

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();//for it hasn't loop candidate KFs, it breaks the rule of "consecutive" new KFs condition for loop validation/roubst loop detection->restart mvConsistentGroups' counter
	cout<<"Empty, discard loop detection!"<<mpCurrentKF->mnId<<" "<<mpCurrentKF->mTimeStamp<<endl;
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes(mpCurrentKFs/call DetectLoop() many times) to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;//the new mvConsistentGroups' size is the same as vpCandidateKFs.size()/vCurrentConsistentGroups.size()
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();//all 1st layer covisibility KFs of the pCandidateKF
        spCandidateGroup.insert(pCandidateKF);//Each candidate expands a covisibility group(loop candidate+its connectedKFs in covisibility graph)

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)//previous ConsistentGroups
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))//A candidate group is consistent with a previous group if they share at least a keyframe
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)//A candidate group is consistent with this previous group
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;//consistency counter++
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }//vbConsistentGroup[iG]==true if any LoopCandidateKF before is consistent with the iGth previous group
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)//if enough consecutive consistency counter/loop detections, here at least 3 new KFs detect the consistent loop candidate group
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);//notice mvpEnoughConsistentCandidates is a member data, used in this function and ComputeSim3()
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group(this loop candidate KF's group) is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
// 	    if (mnCovisibilityConsistencyTh==0) mvpEnoughConsistentCandidates.push_back(pCandidateKF);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database, always done before return
    mpKeyFrameDB->add(mpCurrentKF);//addition to KFDB only here(LoopClosing)

    if(mvpEnoughConsistentCandidates.empty())
    {
        cout<<"Final Empty, discard loop detection!"<<mpCurrentKF->mnId<<" "<<mpCurrentKF->mTimeStamp<<endl;
        mpCurrentKF->SetErase();
        return false;
    }
    else//if any candidate group is enough(counter >=3) consistent with any previous group
    {//first some detection()s won't go here
        cout<<"DetectLoop!"<<mpCurrentKF->mnId<<" "<<mpCurrentKF->mTimeStamp<<endl;
        return true;//keep mpCurrentKF->mbNotErase==true until ComputeSim3() or even CorrectLoop()
    }

    //unused
    //mpCurrentKF->SetErase();
    //return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3
    //notice if we call loop candidate KFs we mean mpMatchedKF && its neighbors(including some previous mpMatchedKFs), but in DetectLoop() it also means lots of mpMatchedKF candidates

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);//same threshold in Relocalization() in Tracking

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);//rectify vpMatches12 by using pKF->mFeatVec to quickly match, \
        corresponding to pKF1/mpCurrentKF in LoopClosing

        cout<<redSTR<<i<<": "<<nmatches<<endl;
        int thresholdMatches=mnLastOdomKFId==0?20:15;
        if(nmatches<thresholdMatches)//20)//same threshold in TrackWithMotionModel(), new 10
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);//how?
            int minInliers=mnLastOdomKFId==0?20:10;//20 is stricter than Relocalization()s, old 20 new 10
            pSolver->SetRansacParameters(0.99,minInliers,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;//>=20 matches
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    cout<<redSTR<<nCandidates<<whiteSTR<<endl;
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);//same iterations in Relocalization(), ScurrentKF_maploopcandidateKF(enough consistent)/S12

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                cout<<redSTR<<"NoMore"<<whiteSTR<<endl;
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));//inliers(after Sim3Solver iterate) in vvpMapPointMatches[i]
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                    //notice if outliers vvpMapPointMatches[i][j] may exists by SBBoW
                }

                cv::Mat R = pSolver->GetEstimatedRotation();//R12
                cv::Mat t = pSolver->GetEstimatedTranslation();//t12
                const float s = pSolver->GetEstimatedScale();//RGBD is s12==1
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);//add some matched MP to vpMapPointMatches by SBP, 7.5 is like the middle of 10&&3 in Relocalization()
                //notice vpMapPointMatches[k] means mpCurrentKF->mvpMapPoints[k]'s matched pMP(pKF/mvpEnoughConsistentCandidates[i]->mvpMapPoints[j]), \
                they're matched but temporary (may)different 2 MPs, later will be fused in CorrectLoop()

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);//g2o: S12
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);//Sim3Motion-only BA, gScm/S12 is optimized, \
                BA outliers in vpMapPointMatches are erased

                // If optimization is succesful stop ransacs and continue
                cout<<redSTR<<nInliers<<whiteSTR<<endl;
                if(nInliers>=20)//looser than Relocalization() inliers' demand
                {
                    bMatch = true;
                    mpMatchedKF = pKF;//member data, enough matched loop candidate KF
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);//g2o::Sim3(Tc2w/T2w) for RGBD
                    mg2oScw = gScm*gSmw;//g2o: T1w=T12*T2w for RGBD, this means we fix loop candidate KF, correct the mpCurrentKF's Pose
                    mScw = Converter::toCvMat(mg2oScw);//Scamera1_world/S1w/ScurrentKF_world/Scw

                    mvpCurrentMatchedPoints = vpMapPointMatches;//enough BA inliers' mpCurrentKF->mvpMapPoints' matched MPs
                    break;
                }
            }
        }
    }

    if(!bMatch)//if BA inliers validation is not passed
    {
        cout<<"bMatch==false, discard loop detection!"<<mpCurrentKF->mnId<<" "<<mpCurrentKF->mTimeStamp<<endl;
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();//allow loop candidate KFs && mpCurrentKF to be erased for KF.mspLoopEdges is only added in CorrectLoop()
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();//all 1st layer covisibility KFs
    vpLoopConnectedKFs.push_back(mpMatchedKF);//in Loop KF and its neighbors
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)//avoid bad && duplicated MPs
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    cout<<redSTR<<nTotalMatches<<whiteSTR<<endl;
    if(nTotalMatches>=40)//similar to Relocalization() inliers' threshold
    {
        cout<<"ComputeSim3 clear!"<<endl;
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();//allow all other loop candidate KFs to be erased
        return true;//notice mpCurrentKF && mpMatchedKF is still not allowed to be erased, where are they allowed?
    }
    else//not pass the final validation like SearchLocalPoints() in Tracking
    {//allow loop candidate KFs && mpCurrentKF to be erased for KF.mspLoopEdges is only added in CorrectLoop()
        cout<<"nTotalMatches<40, discard loop detection!"<<mpCurrentKF->mnId<<" "<<mpCurrentKF->mTimeStamp<<endl;
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

void LoopClosing::CorrectLoop()
{
    if (mnLastOdomKFId>0){
      mnLastOdomKFId=0;//added by zzh
      mnCovisibilityConsistencyTh=3;//return back
    }
  
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();//rapidly process mlNewKeyFrames(jump over localBA...) in LocalMapping and suspend it

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        cout<<"Abort last global BA...";//for debug

        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;//like mbAbortBA in LocalMapping?

        mnFullBAIdx++;//safe termination variable

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();//detach()(nonblocking)/join()(blocking) must be called before ~thread()
            delete mpThreadGBA;//notice mpThreadGBA!=NULL here
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);//1ms asking
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();//use mpCurrentKF->mvpMapPoints[i]->mObservations to update covisibility graph(1st layer covisibility KFs of mpCurrentKF with mpCurrentKF)

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();//all 1st layer covisibility KFs of mpCurrentKF
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;//corrected Scw(ScurrentKF_world)
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();//noncorrected Twc


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();//noncorrected

            if(pKFi!=mpCurrentKF)//for CorrectedSim3[mpCurrentKF]=mg2oScw before
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);//s=1.0 for Tic
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;//corrected Siw=Sic*Scw(corrected)
                //Pose corrected with the Sim3(Scw) of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;//corrected Siw
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;//save the noncorrected Siw(including noncorrected Scw/nonScw), used for next step's correction of MPs' Pos
        }

        // Correct all MapPoints observed by current keyframe and neighbors (&&these observers(KFs)), so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];//noncorrected Siw

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)//avoid duplications
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);//noncorrected Pw
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));//corrected Swi*Pi(noncorrected Siw*noncorrected Pw)=corrected Pw

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);//corrected Pw
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;//update pMPi->mnCorrectedByKF
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();//update pMPi's normal for its mWordPos is changed
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1], get t in Tiw(corrected)

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);//get opencv: corrected Tiw

            pKFi->SetPose(correctedTiw);//update pKFi's Pose

            // Make sure connections are updated
            pKFi->UpdateConnections();//I think it's useless for no pMP->nmObservations is changed,need test!
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated, these fuse don't need SBP again
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];//matched MP of pCurMP
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);//use new corrected MPs(pLoopMP) instead old ones(pCurMP) for pLoopMP is corrected by Sim3Motion-only BA optimized S12
                else//may happen for additional matched MPs by last SBP() in ComputeSim3()
                {//add loop matched MPs to mpCurrentKF and update MPs' mObservations and descriptor
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();//update pLoopMP's descriptor for its mObservations is changed
                }
            }
        }
        mpMap->InformNewChange();//improved by zzh
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;//new links from mvpCurrentConnectedKFs to its new neighbors/loop KFs(set)

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();//all 1st layer covisibility KFs of mvpCurrentConnectedKFs before pKFi->UpdateConnections

        // Update connections. Detect new links.
        pKFi->UpdateConnections();//update is needed for lots of new fused MPs in pKFi->mvpMapPoints, their mObservations are changed by fusing
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);//delete previous neighbors of this pKFi, so LoopConnections[pKFi] means new links' KFs/neighbors
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);//delete mvpCurrentConnectedKFs/part of previous neighbors of other mvpCurrentConnectedKFs, so LoopConnections[pKFi] means new links' loop KFs/neighbors
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);//PoseGraph Opt.

    mpMap->InformNewBigChange();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    //mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release/recover Local Mapping.
    mpLocalMapper->Release();

    mLastLoopKFid = mpCurrentKF->mnId;   
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);//same threshold as SearchLocalPoints() in Tracking

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);//Scamera_world

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);//th==4 a little larger than 3

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);//replace vpReplacePoints[i]/pKF->mvpMapPoints[bestIdx] by mvpLoopMapPoints[i]
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
	
	mnLastOdomKFId=0;mnCovisibilityConsistencyTh=3;//added by zzh
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)//nLoopKF here is mpCurrentKF
{
    std::chrono::steady_clock::time_point begin= std::chrono::steady_clock::now();
    cout <<redSTR "Starting Global Bundle Adjustment" << whiteSTR<<endl;

    bool bUseGBAPRV=false;
    int idx =  mnFullBAIdx;
    unique_lock<mutex> lockScale(mpMap->mMutexScaleUpdateGBA);//notice we cannot update scale during LoopClosing or LocalBA! 
    if (mpIMUInitiator->GetVINSInited()) {
        if (!mpIMUInitiator->GetInitGBAOver()) {//if it's 1st Full BA just after IMU Initialized(the before ones may be cancelled)
            cerr << redSTR"Full BA just after IMU Initializated!" << whiteSTR << endl;
            Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap, mpIMUInitiator->GetGravityVec(), mnInitIterations,
                                                         &mbStopGBA, nLoopKF,
                                                         false);//15 written in V-B of VIORBSLAM paper
//        mbFixScale=true;//not good for V203 when used
        } else {
            Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap, mpIMUInitiator->GetGravityVec(), mnIterations,
                                                         &mbStopGBA,
                                                         nLoopKF, false);
        }
        bUseGBAPRV = true;
    }else{
      cerr<<redSTR"pure-vision GBA!"<<whiteSTR<<endl;
      Optimizer::GlobalBundleAdjustment(mpMap,mnIterations,&mbStopGBA,nLoopKF,false,mpIMUInitiator->GetSensorEnc());//GlobalBA(GBA),10 iterations same in localBA/motion-only/Sim3motion-only BA, may be stopped by next CorrectLoop()
    }
    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)//it's for safe terminating this thread when it's so slow that mbStopGBA becomes false again(but mnFullBAIdx++ before), synchrone mechanism
            return;

        if(!mbStopGBA)//I think it's useless for when mbStopGBA==true, idx!=mnFullBAIdx
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();//same as CorrectLoop(), suspend/stop/freeze LocalMapping thread
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())//if LocalMapping is killed by System::Shutdown(), don't wait any more
            {
                usleep(1000);
            }
            
            chrono::steady_clock::time_point t1=chrono::steady_clock::now();//test time used

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe(id 0)
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

	    //propagate the correction through the spanning tree(root is always id0 KF)
            while(!lpKFtoCheck.empty())//if the propagation is not over (notice mpMap cannot be reset for LocalMapping is stopped)
            {
                KeyFrame* pKF = lpKFtoCheck.front();//for RGBD, lpKFtoCheck should only have one KF initially
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
// 		cout<<"Check: "<<pKF->mnId<<endl;
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
// 		    cout<<" "<<pChild->mnId;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)//if child is not GBA optimized by mpCurrentKF/it must be the new KFs created by LocalMapping thread during GBA
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;//Tchildw*Tw0=Tchild0
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//Tchild0*T0w(corrected)=Tchildw(corrected)
                        pChild->mnBAGlobalForKF=nLoopKF;//so now its child KF' Pose can seem to be corrected by GBA

                        if (bUseGBAPRV){
                            // Set NavStateGBA and correct the PR&V
                            pChild->mNavStateGBA = pChild->GetNavState();//Tb_old_w
                            Matrix3d Rw1 = pChild->mNavStateGBA.getRwb();Vector3d Vw1 = pChild->mNavStateGBA.mvwb;//Rwb_old&wVwb_old
                            cv::Mat TwbGBA = Converter::toCvMatInverse(Frame::mTbc*pChild->mTcwGBA);//TbwGBA.t()
                            Matrix3d RwbGBA=Converter::toMatrix3d(TwbGBA.rowRange(0,3).colRange(0,3));
                            pChild->mNavStateGBA.setRwb(RwbGBA);
                            pChild->mNavStateGBA.mpwb=Converter::toVector3d(TwbGBA.rowRange(0,3).col(3));
                            pChild->mNavStateGBA.mvwb=RwbGBA*Rw1.transpose()*Vw1;//Vwb_new=wVwb_new=Rwb_new*bVwb=Rwb_new*Rb_old_w*wVwb_old=Rwb2*Rwb1.t()*wV1
                        }
                    }//now the child is optimized by GBA
                    lpKFtoCheck.push_back(pChild);
                }
//                 cout<<endl;

                pKF->mTcwBefGBA = pKF->GetPose();//record the old Tcw
                if (bUseGBAPRV)
                    pKF->SetNavState(pKF->mNavStateGBA);//update all KFs' Pose to GBA optimized Tbw&Tcw, including UpdatePoseFromNS()&&SetPose(pKF->mTcwGBA), not necessary to update mNavStateBefGBA for unused in MapPoints' correction
                else
                    pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)//all MPs in mpMap
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)//if this MP is GBA optimized
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);//update all (old)MPs' Pos to GBA optimized Pos
                }
                else//new MPs created by Tracking/LocalMapping thread during GBA
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)//I think it should be false for it's propagated through spanning tree,need test
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);//old Rcw
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);//old tcw
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;//Xc=(Tcw(old)*Pw(old))(0:2)

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();//new Twc
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);//new Rwc
                    cv::Mat twc = Twc.rowRange(0,3).col(3);//new twc

                    pMP->SetWorldPos(Rwc*Xc+twc);//update all (new)MPs' Pos to GBA optimized Pos/Pw(new)=Twc(new)*Pc
                }
            }            

            mpMap->InformNewBigChange();//used to check the SLAM's state

            mpLocalMapper->Release();//recover LocalMapping thread, same as CorrectLoop()
            
	    if (bUseGBAPRV) mpIMUInitiator->SetInitGBAOver(true);//should be put after 1st visual-inertial full BA!

            cout << redSTR<<"Map updated!" <<whiteSTR<< endl;//if GBA/loop correction successed, this word should appear!
            
            cout<<azureSTR"Used time in propagation="<<chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now()-t1).count()<<whiteSTR<<endl;//test time used
        }

        //mbFinishedGBA = true;
        mbRunningGBA = false;

        //for ros_mono_pub.cc
        mbLoopDetected=true;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
