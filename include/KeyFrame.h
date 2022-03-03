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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "FrameBase.h"
#include "MultiThreadBase.h"
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace VIEO_SLAM
{

class Map;
class MapPoint;
class KeyFrameDatabase;
class GeometricCamera;

class KeyFrame : public FrameBase, public MutexUsed
{
  char mState;
//   std::mutex mMutexState;

  std::mutex mMutexNavState;//the mutex of mNavState(state/vertex), similar to mMutexPose

  std::mutex mMutexOdomData;//the mutex of PreIntegrator(measurements), though BA doesn't change bi_bar leading to the unchanged IMU measurement, KFCulling() does change Odom measurement

  // Odom connections for localBA
  KeyFrame *mpPrevKeyFrame,*mpNextKeyFrame;
  std::mutex mMutexPNConnections;//the mutex of Prev/Next KF(connections/sparse states related to this KF), similar to mMutexConnections
//   bool mbPNChanging;std::mutex mMutexPNChanging;
  static std::mutex mstMutexPNChanging;//avoid conescutive 2/3 KFs' SetBadFlag() and SetErase() at the same time! //or check if the former & latter KFs' mbPNChanging are both false

  void UpdatePoseFromNS();

public:
  NavState mNavStateGBA;//like mTcwGBA, for LoopClosing, set and used in the same thread(LoopClosing or IMUInitialization)
  NavState mNavStatePrior;Matrix<double,15,15> mMargCovInv;std::vector<bool> mvbOutlier;//empty, just for template of PoseOptimization()
  const bool mbPrior;//always false
  //PCL used image
  cv::Mat Img[2];//0 is color,1 is depth

  NavState GetNavState(void) override{//cannot use const &(make mutex useless)
    unique_lock<mutex> lock(mMutexNavState);
    return mNavState;//call copy constructor
  }
  void SetNavState(const NavState& ns) override{
    unique_lock<mutex> lock(mMutexNavState);
    mNavState=ns;
    UpdatePoseFromNS();
  }//I think this should always call SetPose()
  void SetNavStateOnly(const NavState& ns){
    unique_lock<mutex> lock(mMutexNavState);
    mNavState=ns;
  }//if u use this func., please SetPose() by yourself
  EncPreIntegrator GetEncPreInt(void) override{
    unique_lock<mutex> lock(mMutexOdomData);
    return mOdomPreIntEnc;//call copy constructor
  }
  IMUPreintegrator GetIMUPreInt(void) override{
    unique_lock<mutex> lock(mMutexOdomData);
    return mOdomPreIntIMU;//call copy constructor
  }
  listeig(IMUData) GetListIMUData(){
    unique_lock<mutex> lock(mMutexOdomData);
    return mOdomPreIntIMU.getlOdom();//call copy constructor
  }//used in LocalMapping && IMUInitialization threads
  listeig(EncData) GetListEncData(){
    unique_lock<mutex> lock(mMutexOdomData);
    return mOdomPreIntEnc.getlOdom();//call copy constructor
  }
  KeyFrame* GetPrevKeyFrame(void){//for localBA fixing N+1th KF
      unique_lock<mutex> lock(mMutexPNConnections);
      return mpPrevKeyFrame;//return copy of int
  }
  KeyFrame* GetNextKeyFrame(void){//for KFCulling() judgement of timespan<=0.5/3s in VIORBSLAM paper
      unique_lock<mutex> lock(mMutexPNConnections);
      return mpNextKeyFrame;//return copy of int
  }
  void SetPrevKeyFrame(KeyFrame* pKF){//for KFCulling()/SetBadFlag()
    unique_lock<mutex> lock(mMutexPNConnections);
    mpPrevKeyFrame = pKF;
  }
  void SetNextKeyFrame(KeyFrame* pKF){//for KF's constructor && KFCulling()
    unique_lock<mutex> lock(mMutexPNConnections);
    mpNextKeyFrame = pKF;
  }
  void UpdateNavStatePVRFromTcw();//mainly for Posegraph optimization, but I think when Tcw is finally optimized, please call this func. to update NavState

  // Odom PreIntegration
  template <class _OdomData>
  void SetPreIntegrationList(const typename listeig(_OdomData)::const_iterator &begin,
			     const typename listeig(_OdomData)::const_iterator &end){//notice template definition should be written in the same file! & typename should be added before nested type!
    unique_lock<mutex> lock(mMutexOdomData);
    mOdomPreIntEnc.SetPreIntegrationList(begin,end);
  }
  template <class OdomData>  // splice operation (like move) for fast append
  void AppendFrontPreIntegrationList(aligned_list<OdomData> &x,
                                     const typename aligned_list<OdomData>::const_iterator &begin,
                                     const typename aligned_list<OdomData>::const_iterator &end) {
    unique_lock<mutex> lock(mMutexOdomData);
    mOdomPreIntEnc.AppendFrontPreIntegrationList(x, begin, end);
  }
  template <class OdomData>
  void PreIntegration(KeyFrame* pLastKF){
    unique_lock<mutex> lock(mMutexOdomData);
    //mOdomPreIntEnc.PreIntegration(pLastKF->mTimeStamp,mTimeStamp);
    FrameBase::PreIntegration<OdomData>(pLastKF, mOdomPreIntEnc.getlOdom().begin(), mOdomPreIntEnc.getlOdom().end());
  }//0th frame don't use this function, pLastKF shouldn't be bad
  //[iteri,iterj) IMU preintegration, breset=false could make KF2KF preintegration time averaged to per frame &&
  // connect 2KFs preintegration by only preintegrating the final KF2KF period
  template <class OdomData>
  void PreIntegrationFromLastKF(FrameBase *plastkf,
                                const typename aligned_list<OdomData>::const_iterator &iteri,
                                const typename aligned_list<OdomData>::const_iterator &iterj,
                                bool breset = false, int8_t verbose = 0) {
    NavState ns = plastkf->GetNavState();
    unique_lock<mutex> lock(mMutexOdomData);
    FrameBase::PreIntegration<OdomData, EncPreIntegrator>((*iteri).mtm, mTimeStamp, ns.mbg, ns.mba, iteri, iterj, breset);
  }

  std::set<KeyFrame *> GetConnectedKeyFramesByWeight(int w);//set made from mConnectedKeyFrameWeights[i].first restricted by weight

  inline char getState(){
//     unique_lock<mutex> lock(mMutexState);
    return mState;
  }
//   inline char setState(char state){
//     unique_lock<mutex> lock(mMutexState);
//     mState=state;
//   }

  //for LoadMap() in System.cc
  KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB,KeyFrame* pPrevKF,istream &is);
  template <class T>
  static inline bool readVec(istream &is,T &vec){//can also be used for set/list
    for (typename T::iterator iter=vec.begin();iter!=vec.end();++iter){
      is.read((char*)&(*iter),sizeof(*iter));
    }
    return is.good();
  }
  template <class T>
  static inline bool writeVec(ostream &os,const T &vec){
    for (typename T::const_iterator iter=vec.begin();iter!=vec.end();++iter){
      os.write((char*)&(*iter),sizeof(*iter));
    }
    return os.good();
  }
  template <class _OdomData>
  inline bool readListOdom(std::istream &is,listeig(_OdomData) &lis){
    for (typename listeig(_OdomData)::iterator iter=lis.begin();iter!=lis.end();++iter) iter->read(is);
    return is.good();
  }
  template <class _OdomData>
  bool writeListOdom(std::ostream &os,const listeig(_OdomData) &lis) const{
    for (typename listeig(_OdomData)::const_iterator iter=lis.begin();iter!=lis.end();++iter) iter->write(os);
    return os.good();
  }
  static inline bool readMat(istream &is,cv::Mat &mat){
    for (int i=0;i<mat.rows;++i){
      is.read((char*)mat.ptr(i),mat.cols*mat.elemSize());
    }
    return is.good();
  }
  static inline bool writeMat(ostream &os,const cv::Mat &mat){
    for (int i=0;i<mat.rows;++i){
      os.write((char*)mat.ptr(i),mat.cols*mat.elemSize());
    }
    return os.good();
  }
  template <class T>
  static inline bool readEigMat(istream &is,T &mat){
    is.read((char*)mat.data(),mat.size()*sizeof(typename T::Scalar));
    return is.good();
  }
  template <class T>
  static inline bool writeEigMat(ostream &os,const T &mat){//for Eigen::Matrix<_Scalar,_Rows,_Cols>
    os.write((char*)mat.data(),mat.size()*sizeof(typename T::Scalar));//mat.size()==mat.rows()*mat.cols(), saved acquiescently as the column-major order
    return os.good();
  }
  bool read(istream &is);
  bool write(ostream &os);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW//for quaterniond in NavState
//created by zzh over.

public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB,KeyFrame* pPrevKF=NULL,const char state=2);//2 is OK

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();//Tcw
    cv::Mat GetPoseInverse();
    const Sophus::SE3d GetTwc() override;
    const Sophus::SE3d GetTcw() override;
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);//mConnectedKeyFrameWeights.erase(pKF) && UpdateBestCovisibles()
    void UpdateConnections(KeyFrame* pLastKF=NULL);//first connect other KFs to this, then connect this to other KFs/update this->mConnectedKeyFrameWeights...; an undirected graph(covisibility graph)
    void UpdateBestCovisibles();//update mvpOrderedConnectedKeyFrames && mvOrderedWeights by sort()
    std::set<KeyFrame *> GetConnectedKeyFrames();//set made from mConnectedKeyFrameWeights[i].first
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();//mvpOrderedConnectedKeyFrames
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);//get N closest KFs in covisibility graph(map<KF*,int>)
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);//get some closest KFs in covisibility graph whose weight>=w
    int GetWeight(KeyFrame* pKF);//mConnectedKeyFrameWeights[pKF](0 no found), now 0 maybe found rectified by zzhs

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);//mspChildrens.insert(pKF)
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);//mpParent = pKF,pKF->AddChild(this);
    std::set<KeyFrame*> GetChilds();//mspChildrens
    KeyFrame* GetParent();//mpParent
    bool hasChild(KeyFrame* pKF);//if pKF in mspChildrens

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);//mspLoopEdges.insert(pKF);mbNotErase=true;
    std::set<KeyFrame*> GetLoopEdges();//mspLoopEdges

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx) override;//mvpMapPoints[idx]=pMP
    void EraseMapPointMatch(const size_t &idx) override;//mvpMapPoints[idx]=nullptr
    void EraseMapPointMatch(MapPoint* pMP);//mvpMapPoints[idx corresp. pMP]=nullptr
    std::set<std::pair<MapPoint*, size_t>> GetMapPointsCami() override;
    std::vector<MapPoint*> GetMapPointMatches() override;//mvpMapPoints
    int TrackedMapPoints(const int &minObs);//return the number of good mvpMapPoints whose nObs>=minObs
    MapPoint* GetMapPoint(const size_t &idx);//mvpMapPoints[idx]
    void FuseMP(size_t idx, MapPoint* pMP);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(size_t cami, const float &x, const float  &y, const float  &r) const;//return vec<featureID>, a 2r*2r window search by Grids/Cells speed-up, here no min/maxlevel check unlike Frame.h
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();//mbNotErase=true means it cannot be directly erased by SetBadFlag(), but can use SetErase()
    void SetErase();//try to erase this(&KF) by SetBadFlag() when mbToBeErased==true(SetBadFlag() before)&&mspLoopEdges.empty()

    // Set/check bad flag
    void SetBadFlag(bool bKeepTree=false);//Erase the relation with this(&KF), Update Spanning Tree&& mbBad+mTcp, erase this(&KF) in mpMap && mpKeyFrameDB; KeepTree=true is used for BadKF's recover in LoadMap()
    bool isBad();//mbBad

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;//for local Map in tracking
    long unsigned int mnFuseTargetForKF;//for LocalMapping

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;//for local BA in LocalMapping
    long unsigned int mnBAFixedForKF;//for local BA in LocalMapping

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing in GBA
    cv::Mat mTcwGBA;//optimized Tcw in the end of GBA
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;//mpCurrentKF used to correct loop and call GBA

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;//used in SaveTrajectoryTUM() in System.cc

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Twc;
    cv::Mat Ow;
    cv::Mat Cw; // Stereo middel point. Only for visualization

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector< std::vector <std::vector<size_t>>>> vgrids_;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;//covisibility graph need KFs (>0 maybe unidirectional edge!maybe u can revise it~) covisible MapPoints
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;//ordered covisibility graph/connected KFs need KFs >=15 covisible MPs or the KF with Max covisible MapPoints
    std::vector<int> mvOrderedWeights;//covisible MPs' number

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;//the mutex of mvpMapPoints(landmarks' states/vertices)

    inline const Sophus::SE3d GetTcwCst() const { CV_Assert(0 && "Need Lock Mutex, cannot use ()const!"); }
};

//created by zzh
template <>//specialized
void KeyFrame::SetPreIntegrationList<IMUData>(const listeig(IMUData)::const_iterator &begin,const listeig(IMUData)::const_iterator &end);
template <>  // splice operation (like move) for fast append
void KeyFrame::AppendFrontPreIntegrationList(aligned_list<IMUData> &x,
                                             const typename aligned_list<IMUData>::const_iterator &begin,
                                             const typename aligned_list<IMUData>::const_iterator &end);
template <>
void KeyFrame::PreIntegration<IMUData>(KeyFrame* pLastKF);
template <>
void KeyFrame::PreIntegrationFromLastKF<IMUData>(
    FrameBase *plastkf, const typename aligned_list<IMUData>::const_iterator &iteri,
    const typename aligned_list<IMUData>::const_iterator &iterj, bool breset, int8_t verbose);

} //namespace ORB_SLAM

#endif // KEYFRAME_H
