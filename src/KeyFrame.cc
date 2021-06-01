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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace VIEO_SLAM
{
  
std::mutex KeyFrame::mstMutexPNChanging;
  
void KeyFrame::UpdatePoseFromNS()//same as Frame::UpdatePoseFromNS()
{
  cv::Mat Rbc = Frame::mTbc.rowRange(0,3).colRange(0,3);//don't need clone();
  cv::Mat Pbc = Frame::mTbc.rowRange(0,3).col(3);//or tbc
  
  cv::Mat Rwb = Converter::toCvMat(mNavState.getRwb());
  cv::Mat Pwb = Converter::toCvMat(mNavState.mpwb);//or twb
  //Tcw=Tcb*Twb, Twc=Twb*Tbc
  cv::Mat Rcw = (Rwb*Rbc).t();
  cv::Mat Pwc = Rwb*Pbc + Pwb;
  cv::Mat Pcw = -Rcw*Pwc;//tcw=-Rwc.t()*twc=-Rcw*twc

  cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
  Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
  Pcw.copyTo(Tcw.rowRange(0,3).col(3));

  SetPose(Tcw);//notice w means B0/0th IMU Frame, c means ci/c(ti)/now camera Frame
}

void KeyFrame::UpdateNavStatePVRFromTcw()
{
  unique_lock<mutex> lock(mMutexNavState);
  cv::Mat Twb;
  {
    unique_lock<mutex> lock(mMutexPose);//important for using Tcw for this func. is multi threads!
    Twb=Converter::toCvMatInverse(Frame::mTbc*Tcw);
  }
  Eigen::Matrix3d Rwb=Converter::toMatrix3d(Twb.rowRange(0,3).colRange(0,3));
  Eigen::Vector3d Pwb=Converter::toVector3d(Twb.rowRange(0,3).col(3));

  Eigen::Matrix3d Rw1=mNavState.getRwb();//Rwbj_old/Rwb1
  Eigen::Vector3d Vw1=mNavState.mvwb;//Vw1/wV1=wvbj-1bj_old now bj_old/b1 is changed to bj_new/b2, wV2=wvbj-1bj_new
  Eigen::Vector3d Vw2=Rwb*Rw1.transpose()*Vw1;//bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

  mNavState.mpwb=Pwb;
  mNavState.setRwb(Rwb);
  mNavState.mvwb=Vw2;
}

template <>//specialized
void KeyFrame::SetPreIntegrationList<IMUData>(const listeig(IMUData)::const_iterator &begin,const listeig(IMUData)::const_iterator &pback){
  unique_lock<mutex> lock(mMutexOdomData);
  mOdomPreIntIMU.SetPreIntegrationList(begin,pback);
}
template <>
void KeyFrame::PreIntegration<IMUData>(KeyFrame* pLastKF){
  Eigen::Vector3d bgi_bar=pLastKF->GetNavState().mbg,bai_bar=pLastKF->GetNavState().mba;
  unique_lock<mutex> lock(mMutexOdomData);
#ifndef TRACK_WITH_IMU
  mOdomPreIntIMU.PreIntegration(pLastKF->mTimeStamp,mTimeStamp);
#else
  mOdomPreIntIMU.PreIntegration(pLastKF->mTimeStamp,mTimeStamp,bgi_bar,bai_bar);
#endif
}

std::set<KeyFrame *> KeyFrame::GetConnectedKeyFramesByWeight(int w){
  vector<KeyFrame*> vConnectedKFw=GetCovisiblesByWeight(w);
  unique_lock<mutex> lock(mMutexConnections);
  set<KeyFrame*> s;
  for(vector<KeyFrame*>::iterator vit=vConnectedKFw.begin();vit!=vConnectedKFw.end();++vit)
    s.insert(*vit);
  return s;
}

//for LoadMap()
KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB,KeyFrame* pPrevKF,istream &is):
  mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
  mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
  mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
  mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
  fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
  mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
  mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
  mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
  mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
  mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
  mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),//for mK won't be changed, it cannot be used as clone()
  mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(false), mpParent(NULL), mbNotErase(false),//important false when LoadMap()!
  mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap),
  mbPrior(false)//,mbPNChanging(false)//zzh
{
  if(pPrevKF)
    pPrevKF->SetNextKeyFrame(this);
  mpPrevKeyFrame=pPrevKF;mpNextKeyFrame=NULL;//zzh, constructor doesn't need to lock mutex
  mNavState=F.mNavState;//we don't update bias for convenience in LoadMap(), though we can do it as mOdomPreIntOdom is updated in read()
  
  mnId=nNextId++;
  mGrid.resize(mnGridCols);
  for(int i=0; i<mnGridCols;i++){
    mGrid[i].resize(mnGridRows);
    for(int j=0; j<mnGridRows; j++)
      mGrid[i][j] = F.mGrid[i][j];
  }
  SetPose(F.mTcw);//we have already used UpdatePoseFromNS() in Frame
  
  read(is);//set odom list & mState
}
bool KeyFrame::read(istream &is){
  //we've done ComputeBoW() in Frame!
  {//load odom lists
    listeig(EncData) lenc;
    size_t NOdom;
    is.read((char*)&NOdom,sizeof(NOdom));
    lenc.resize(NOdom);
    readListOdom<EncData>(is,lenc);
    SetPreIntegrationList<EncData>(lenc.begin(),--lenc.end());
    listeig(IMUData) limu;
    is.read((char*)&NOdom,sizeof(NOdom));
    limu.resize(NOdom);
    readListOdom<IMUData>(is,limu);
    SetPreIntegrationList<IMUData>(limu.begin(),--limu.end());
  }
  if (mpPrevKeyFrame!=NULL){//Compute/Recover mOdomPreIntOdom, mpPrevKeyFrame already exists for KFs of mpMap is sorted through mnId
    PreIntegration<EncData>(mpPrevKeyFrame);
    PreIntegration<IMUData>(mpPrevKeyFrame);
  }
  is.read(&mState,sizeof(mState));
  mHalfBaseline=mb/2;
  //we've loaded mNavState in Frame
  //we have calculated mGrid in Frame and load it in constructor
  return is.good();
}
bool KeyFrame::write(ostream &os){
//   os.write((char*)&mnFrameId,sizeof(mnFrameId));//we don't save Frame ID for it's useless in LoadMap(), we save old KF ID/mnId in SaveMap()
  os.write((char*)&mTimeStamp,sizeof(mTimeStamp));
//   os.write((char*)&mfGridElementWidthInv,sizeof(mfGridElementWidthInv));os.write((char*)&mfGridElementHeightInv,sizeof(mfGridElementHeightInv));//we can get these from mnMaxX...
  os.write((char*)&fx,sizeof(fx));os.write((char*)&fy,sizeof(fy));os.write((char*)&cx,sizeof(cx));os.write((char*)&cy,sizeof(cy));
//   os.write((char*)&invfx,sizeof(invfx));os.write((char*)&invfy,sizeof(invfy));//also from the former ones
  os.write((char*)&mbf,sizeof(mbf));os.write((char*)&mThDepth,sizeof(mThDepth));
//   os.write((char*)&mb,sizeof(mb));//=mbf/fx
  os.write((char*)&N,sizeof(N));
  writeVec(os,mvKeys);writeVec(os,mvKeysUn);writeVec(os,mvuRight);writeVec(os,mvDepth);
  writeMat(os,mDescriptors);
//   mBowVec.write(os);mFeatVec.write(os);//we can directly ComputeBoW() from mDescriptors
  os.write((char*)&mnScaleLevels,sizeof(mnScaleLevels));os.write((char*)&mfScaleFactor,sizeof(mfScaleFactor));
//   os.write((char*)&mfLogScaleFactor,sizeof(mfLogScaleFactor));os.write((char*)&mvScaleFactors,sizeof(mvScaleFactors));//we can get these from former 2 parameters
//   writeVec(os,mvLevelSigma2);writeVec(os,mvInvLevelSigma2);
  float fTmp[4]={mnMinX,mnMinY,mnMaxX,mnMaxY};//compatible with Frame
  os.write((char*)fTmp,sizeof(fTmp));
//   writeMat(os,mK);from fx~cy
  //save mvpMapPoints,mpParent,mbNotErase(mspLoopEdges) in LoadMap for convenience
//   os.write((char*)&mHalfBaseline,sizeof(mHalfBaseline));//=mb/2;
  {//save mNavState
    const double* pdData;
    unique_lock<mutex> lock(mMutexNavState);
    Eigen::Quaterniond q=mNavState.mRwb.unit_quaternion();//qwb from Rwb
    pdData=mNavState.mpwb.data();os.write((const char*)pdData,sizeof(*pdData)*3);//txyz
    pdData=q.coeffs().data();os.write((const char*)pdData,sizeof(*pdData)*4);//qxyzw
    pdData=mNavState.mvwb.data();os.write((const char*)pdData,sizeof(*pdData)*3);//vxyz
    pdData=mNavState.mbg.data();os.write((const char*)pdData,sizeof(*pdData)*3);//bgxyz_bar
    pdData=mNavState.mba.data();os.write((const char*)pdData,sizeof(*pdData)*3);//baxyz_bar
    pdData=mNavState.mdbg.data();os.write((const char*)pdData,sizeof(*pdData)*3);//dbgxyz
    pdData=mNavState.mdba.data();os.write((const char*)pdData,sizeof(*pdData)*3);//dbaxyz
  }
//   for(unsigned int i=0; i<FRAME_GRID_COLS;i++) for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){ size_t nSize;os.write((char*)&nSize,sizeof(nSize));writeVec(os,mGrid[i][j]);}//we can still get it from mvKeysUn
  //we add extra info for KF at the end for KeyFrame::write & Frame::read+KeyFrame::read
  {//save odom lists
    unique_lock<mutex> lock(mMutexOdomData);
    const listeig(EncData) &lenc=mOdomPreIntEnc.getlOdom();
    size_t NOdom=lenc.size();
    os.write((char*)&NOdom,sizeof(NOdom));
    writeListOdom<EncData>(os,lenc);
    const listeig(IMUData) &limu=mOdomPreIntIMU.getlOdom();
    NOdom=limu.size();
    os.write((char*)&NOdom,sizeof(NOdom));
    writeListOdom<IMUData>(os,limu);
    //we don't save mOdomPreIntOdom for it can be computed from the former odom list
  }
  os.write(&mState,sizeof(mState));
  return os.good();
}
  
//created by zzh over.

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB,KeyFrame* pPrevKF,const char state):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),//for mK won't be changed, it cannot be used as clone()
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap),
    mState(state),mbPrior(false)//,mbPNChanging(false)//zzh
{
    if(pPrevKF)
      pPrevKF->SetNextKeyFrame(this);
    mpPrevKeyFrame=pPrevKF;mpNextKeyFrame=NULL;//zzh, constructor doesn't need to lock mutex
    mNavState=F.mNavState;
    // Set bias as bias+delta_bias, and reset the delta_bias term
    mNavState.mbg+=mNavState.mdbg;mNavState.mba+=mNavState.mdba;
    mNavState.mdbg=mNavState.mdba=Eigen::Vector3d::Zero();//update bi (bi=bi+dbi) for a better PreIntegration of nextKF(localBA) & fixedlastKF motion-only BA of next Frame(this won't optimize lastKF.mdbi any more)
//created by zzh over
  
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;//4cm right of the Ow for kinect2
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);//first > w, here is first < w for weightComp is >
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);//n is right for the number of element whose value>=w
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(KeyFrame* pLastKF)
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty()){//ODOMOK;||mState!=2&&pLastKF!=NULL
        cout<<"Failed to update spanning tree! "<<mnId<<" "<<mnFrameId<<endl;
	if (pLastKF==NULL){
	  if (mpParent==NULL)
	    assert(mnId==0);//"Error in parameter in UpdateConnections()"
	  else
	    cout<<"but has 1 parent and "<<mConnectedKeyFrameWeights.size()<<" covisibility KFs"<<endl;
	}else{
// 	  pLastKF->AddConnection(this,0);//add the link from pLastKF to this
	  //add the link from this to pLastKF
// 	  KFcounter[pLastKF]=0;
	  unique_lock<mutex> lockCon(mMutexConnections);
// 	  mConnectedKeyFrameWeights=KFcounter;//?
// 	  mvpOrderedConnectedKeyFrames.clear();
// 	  mvOrderedWeights.clear();
// 	  mvpOrderedConnectedKeyFrames.push_back(pLastKF);
// 	  mvOrderedWeights.push_back(0);//0 means it's an Odom link
	  
	  //if first connected then update spanning tree
	  if (mbFirstConnection&&mnId!=0){//mnId!=0/this!=pLastKF is important for 0th F/KF to ensure its parent is NULL!
	    assert(this!=pLastKF);
	    mbFirstConnection=false;
	    mpParent=pLastKF;//the closer, the first connection is better
	    mpParent->AddChild(this);
	  }
	}
        return;
    }

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;//the least number of covisible MapPoints between two KFs to add connection, old 15

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend;++mit)//finally we keep the unidirectional edge strategy for we don't want to change the pure RGBD part!
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);//notice here when <th but vPairs is not empty, it's only one directed edge in the covisibility graph!!! is this right? I think it's wrong, so I added a revision
// 	    ++mit;
        }/*else if (mit->first!=pKFmax){//we avoid one directional edge!
	  mit=KFcounter.erase(mit);//revised by zzh, one original bug of ORBSLAM2!
        }else ++mit;*/
//         (mit->first)->AddConnection(this,mit->second);
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);//notice here push_front not push_back!!!
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();//the closer, the first connection is better
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())//if the pair of loop edges doesn't include this KF, it can be erased
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();//if it's not the loop edges, then erased here when SetBadFlag() called during mbNotErase==true
    }
}

void KeyFrame::SetBadFlag(bool bKeepTree)//this will be released in UpdateLocalKeyFrames() in Tracking, no memory leak(not be deleted) for bad KFs may be used by some Frames' trajectory retrieve
{   
    assert(!mbBad);//check    
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)//cannot erase the initial/fixed KF
            return;
        else if(mbNotErase)//mbNotErase may be set true by LoopClosing
        {
            mbToBeErased = true;//wait to be erased in SetErase() by LoopClosing
            return;
        }
    }
    assert(mnId!=0);

    //erase the relation with this(&KF)
    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);//erase the directed edge from others to this (1 undirected edge <==> 2 directed edges)

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);//erase this observation in this->mvpMapPoints
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();//erase the directed edge from this to others in covisibility graph, this is also used as the interface
        mvpOrderedConnectedKeyFrames.clear();//erase the directed edge for the interface GetVectorCovisibleKeyFrames() will use mvpOrderedConnectedKeyFrames, but no mvOrderedWeights will be used as public functions

	if (!bKeepTree){//for LoadMap(), we don't change bad KFs' parent or Tcp for recovering in SaveTrajectoryTUM()
        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
	assert(mpParent!=NULL);
// 	if (mpParent!=NULL) 
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())//if empty/all Bad -> no need to adjust the spanning tree more
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe (children of this)
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);//notice vpConnected[i]->GetWeight(pKF) may not exist for not in time vpConnected[i]->UpdateConnections()
                            if(w>max)//the pair(pC,pP) highest covisibility weight
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)//this updation(connecting culled KF's children with the KF's parent/children) is same as mbFirstConnection(the closest covisibility KF)
            {
                pC->ChangeParent(pP);//connect pC to its new parent pP(max covisibility in sParentCandidates)
                sParentCandidates.insert(pC);//put pC(max covisibility child correspoding to sParentCandidates) into sParentCandidates
                mspChildrens.erase(pC);
            }
            else//if left children's 1st layer covisibility KFs have no sParentCandidates(max==-1) -> break
                break;
        }

        // If a child has no covisibility links/edges with any parent candidate, assign it to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
	}
// 	if (mpParent!=NULL){
        mpParent->EraseChild(this);//notice here mspChildrens may not be empty, and it doesn't take part in the propagation in LoopClosing thread
        if (!bKeepTree) mTcp = Tcw*mpParent->GetPoseInverse();//the inter spot/link of Frames with its refKF in spanning tree
// 	}
        mbBad = true;
    }

    // Update Prev/Next KeyFrame in prev/next, mbBad is not absolutely related to its existence
    {
//       while (!mbPNChanging){//no need to use GetPNChanging() for it won't come here twice
// 	{
	  cout<<"LockST..";
	  unique_lock<mutex> lock(mstMutexPNChanging);
	  cout<<"LockPN..";
	  unique_lock<mutex> lock2(mMutexPNConnections);
// 	  if (!mpPrevKeyFrame->GetPNChanging()&&!mpNextKeyFrame->GetPNChanging()){
// 	    unique_lock<mutex> lock(mMutexPNChanging);
// 	    mbPNChanging=true;
// 	  }
// 	}
// 	if (mbPNChanging){
// 	  unique_lock<mutex> lock(mMutexPNConnections);
	  if (!mpPrevKeyFrame||!mpNextKeyFrame){
	    cerr<<"Prev/Next KF is NULL!!!Please be aware of the reason!!!"<<endl;
	    mpMap->EraseKeyFrame(this);mpKeyFrameDB->erase(this);return;
	  }
	  assert(mpPrevKeyFrame->GetNextKeyFrame()==this&&mpNextKeyFrame->GetPrevKeyFrame()==this);//check 2!!!
	  mpPrevKeyFrame->SetNextKeyFrame(mpNextKeyFrame);//mpNextKeyFrame here cannot be NULL for mpCurrentKF cannot be erased in KFCulling()
	  mpNextKeyFrame->SetPrevKeyFrame(mpPrevKeyFrame);//0th KF cannot be erased so mpPrevKeyFrame cannot be NULL
	  //AppendIMUDataToFront, qIMU can speed up!
	  listeig(IMUData) limunew=mpNextKeyFrame->GetListIMUData();//notice GetIMUPreInt() doesn't copy list!
	  {
	    unique_lock<mutex> lock(mMutexOdomData);
	    limunew.insert(limunew.begin(),mOdomPreIntIMU.getlOdom().begin(),mOdomPreIntIMU.getlOdom().end());
	    mpNextKeyFrame->SetPreIntegrationList<IMUData>(limunew.begin(),--limunew.end());
	  }
	  //AppendEncDataToFront
	  listeig(EncData) lencnew=mpNextKeyFrame->GetListEncData();//notice GetEncPreInt() doesn't copy list!
	  {
	    unique_lock<mutex> lock(mMutexOdomData);
	    lencnew.insert(lencnew.begin(),mOdomPreIntEnc.getlOdom().begin(),mOdomPreIntEnc.getlOdom().end());
	    mpNextKeyFrame->SetPreIntegrationList<EncData>(lencnew.begin(),--lencnew.end());
	  }
	  //ComputePreInt
	  mpNextKeyFrame->PreIntegration<IMUData>(mpPrevKeyFrame);
	  mpNextKeyFrame->PreIntegration<EncData>(mpPrevKeyFrame);
	  mpPrevKeyFrame=mpNextKeyFrame=NULL;//clear this KF's pointer, to check if its prev/next is deleted
// 	}
//       }
//       unique_lock<mutex> lock(mMutexPNChanging);
//       mbPNChanging=false;
    }

    //erase this(&KF) in mpMap && mpKeyFrameDB
    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
    cout<<"End "<<mnId<<" "<<mTimeStamp<<endl;
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
