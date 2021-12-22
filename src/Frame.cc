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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include "KannalaBrandt8.h"
#include "Pinhole.h"
#include "log.h"

namespace VIEO_SLAM
{

cv::Mat Frame::mTbc,Frame::mTce;
Eigen::Matrix3d Frame::meigRcb;Eigen::Vector3d Frame::meigtcb;

//For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);
bool Frame::usedistort_ = false;

void Frame::UpdatePoseFromNS()
{
  cv::Mat Rbc = mTbc.rowRange(0,3).colRange(0,3);//don't need clone();
  cv::Mat Pbc = mTbc.rowRange(0,3).col(3);//or tbc

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
void Frame::UpdateNavStatePVRFromTcw()
{
  cv::Mat Twb = Converter::toCvMatInverse(mTbc*mTcw);
  Eigen::Matrix3d Rwb=Converter::toMatrix3d(Twb.rowRange(0,3).colRange(0,3));
  Eigen::Vector3d Pwb=Converter::toVector3d(Twb.rowRange(0,3).col(3));

  Eigen::Matrix3d Rw1=mNavState.getRwb();//Rwbj_old/Rwb1
  Eigen::Vector3d Vw1=mNavState.mvwb;//Vw1/wV1=wvbj-1bj_old now bj_old/b1 is changed to bj_new/b2, wV2=wvbj-1bj_new
  Eigen::Vector3d Vw2=Rwb*Rw1.transpose()*Vw1;//bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

  mNavState.mpwb=Pwb;
  mNavState.setRwb(Rwb);
  mNavState.mvwb=Vw2;
}

//created by zzh
// template <>//specialized
// void Frame::SetPreIntegrationList<IMUData>(const typename std::list<IMUData>::const_iterator &begin,const typename std::list<IMUData>::const_iterator &pback){
//   mOdomPreIntIMU.SetPreIntegrationList(begin,pback);
// }
template <>
void Frame::PreIntegration<IMUData>(Frame* pLastF,const listeig(IMUData)::const_iterator &iteri,listeig(IMUData)::const_iterator iterjBack){
  Eigen::Vector3d bgi_bar=pLastF->mNavState.mbg,bai_bar=pLastF->mNavState.mba;//we can directly use mNavState here
#ifndef TRACK_WITH_IMU
  mOdomPreIntIMU.PreIntegration(pLastF->mTimeStamp,mTimeStamp,iteri,++iterjBack);
#else
  mOdomPreIntIMU.PreIntegration(pLastF->mTimeStamp,mTimeStamp,bgi_bar,bai_bar,iteri,++iterjBack);
#endif
}
template <>
void Frame::PreIntegration<EncData>(KeyFrame* pLastKF,const listeig(EncData)::const_iterator &iteri,listeig(EncData)::const_iterator iterjBack){
  mOdomPreIntEnc.PreIntegration(pLastKF->mTimeStamp,mTimeStamp,iteri,++iterjBack);
}
template <>
void Frame::PreIntegration<IMUData>(KeyFrame* pLastKF,const listeig(IMUData)::const_iterator &iteri,listeig(IMUData)::const_iterator iterjBack){
  Eigen::Vector3d bgi_bar=pLastKF->GetNavState().mbg,bai_bar=pLastKF->GetNavState().mba;
#ifndef TRACK_WITH_IMU
  mOdomPreIntIMU.PreIntegration(pLastKF->mTimeStamp,mTimeStamp,iteri,++iterjBack);
#else
  mOdomPreIntIMU.PreIntegration(pLastKF->mTimeStamp,mTimeStamp,bgi_bar,bai_bar,iteri,++iterjBack);
#endif
}

Frame::Frame(istream &is,ORBVocabulary* voc):mpORBvocabulary(voc){//please don't forget voc!! Or ComputeBoW() will have a segement fault problem
  mnId=nNextId++;//new Frame ID
  read(is);
  mvpMapPoints.resize(N,static_cast<MapPoint*>(NULL));//N is got in read(), very important allocation! for LoadMap()
}
bool Frame::read(istream &is,bool bOdomList){
  //we don't save old ID for it's useless in LoadMap()
  is.read((char*)&this->mTimeStamp,sizeof(this->mTimeStamp));
  is.read((char*)&fx,sizeof(fx));is.read((char*)&fy,sizeof(fy));is.read((char*)&cx,sizeof(cx));is.read((char*)&cy,sizeof(cy));
  invfx=1.0f/fx;invfy=1.0f/fy;
  is.read((char*)&mbf,sizeof(mbf));is.read((char*)&mThDepth,sizeof(mThDepth));
  mb=mbf/fx;
  is.read((char*)&N,sizeof(N));
  mvuRight.resize(N);mvDepth.resize(N);
  mvKeys.resize(N);mvKeysUn.resize(N);
  KeyFrame::readVec(is, mvKeys);
  KeyFrame::readVec(is, mvKeysUn);
  KeyFrame::readVec(is,mvuRight);KeyFrame::readVec(is,mvDepth);
  vdescriptors_.resize(1);
  mDescriptors=cv::Mat::zeros(N,32,CV_8UC1);//256bit binary descriptors
  KeyFrame::readMat(is,mDescriptors);
  ComputeBoW();//calculate mBowVec & mFeatVec, or we can do it by pKF
  is.read((char*)&mnScaleLevels,sizeof(mnScaleLevels));is.read((char*)&mfScaleFactor,sizeof(mfScaleFactor));
  mfLogScaleFactor=log(mfScaleFactor);
  mvScaleFactors.resize(mnScaleLevels);mvLevelSigma2.resize(mnScaleLevels);mvInvLevelSigma2.resize(mnScaleLevels);
  mvScaleFactors[0]=mvLevelSigma2[0]=1.0f;
  for(int i=1; i<mnScaleLevels; i++){
    mvScaleFactors[i]=mvScaleFactors[i-1]*mfScaleFactor;
    mvLevelSigma2[i]=mvScaleFactors[i]*mvScaleFactors[i];//at 0 level sigma=1 pixel
  }
  for(int i=0; i<mnScaleLevels; i++) mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
  is.read((char*)&mnMinX,sizeof(mnMinX));is.read((char*)&mnMinY,sizeof(mnMinY));is.read((char*)&mnMaxX,sizeof(mnMaxX));is.read((char*)&mnMaxY,sizeof(mnMaxY));
  mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
  mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
  cv::Mat K=cv::Mat::eye(3,3,CV_32F);
  K.at<float>(0,0)=fx;
  K.at<float>(1,1)=fy;
  K.at<float>(0,2)=cx;
  K.at<float>(1,2)=cy;
  K.copyTo(mK);//here mK will be allocated for it does not have a proper size or type before the operation
  //load mNavState
  //For VIO, we should compare the Pose of B/IMU Frame!!! not the Twc but the Twb! with EuRoC's Twb_truth(using Tb_prism/Tbs from vicon0/data.csv) (notice vicon0 means the prism's Pose), and I found state_groundtruth_estimate0 is near Twb_truth but I don't think it's truth!
  double pdData[3];
  double pdData4[4];
  NavState ns;
  is.read((char*)pdData,sizeof(pdData));ns.mpwb<<pdData[0],pdData[1],pdData[2];//txyz
  is.read((char*)pdData4,sizeof(pdData4));ns.mRwb.setQuaternion(Eigen::Quaterniond(pdData4));//qxyzw
  is.read((char*)pdData,sizeof(pdData));ns.mvwb<<pdData[0],pdData[1],pdData[2];//vxyz
  is.read((char*)pdData,sizeof(pdData));ns.mbg<<pdData[0],pdData[1],pdData[2];//bgxyz
  is.read((char*)pdData,sizeof(pdData));ns.mba<<pdData[0],pdData[1],pdData[2];//baxyz
  is.read((char*)pdData,sizeof(pdData));ns.mdbg<<pdData[0],pdData[1],pdData[2];//dbgxyz
  is.read((char*)pdData,sizeof(pdData));ns.mdba<<pdData[0],pdData[1],pdData[2];//dbaxyz
  mNavState=ns;UpdatePoseFromNS();
  //load mGrid[i][j]
  AssignFeaturesToGrid();
  if (bOdomList){
    double &tmEnc=mOdomPreIntEnc.mdeltatij;
    is.read((char*)&tmEnc,sizeof(tmEnc));
    if (tmEnc>0){
      KeyFrame::readEigMat(is,mOdomPreIntEnc.mdelxEij);
      KeyFrame::readEigMat(is,mOdomPreIntEnc.mSigmaEij);
    }
    double &tmIMU=mOdomPreIntIMU.mdeltatij;
    is.read((char*)&tmIMU,sizeof(tmIMU));
    if (tmIMU>0){//for IMUPreIntegratorBase<IMUDataBase>
      KeyFrame::readEigMat(is,mOdomPreIntIMU.mpij);KeyFrame::readEigMat(is,mOdomPreIntIMU.mRij);KeyFrame::readEigMat(is,mOdomPreIntIMU.mvij);//PRV
      KeyFrame::readEigMat(is,mOdomPreIntIMU.mSigmaijPRV);KeyFrame::readEigMat(is,mOdomPreIntIMU.mSigmaij);
      KeyFrame::readEigMat(is,mOdomPreIntIMU.mJgpij);KeyFrame::readEigMat(is,mOdomPreIntIMU.mJapij);
      KeyFrame::readEigMat(is,mOdomPreIntIMU.mJgvij);KeyFrame::readEigMat(is,mOdomPreIntIMU.mJavij);
      KeyFrame::readEigMat(is,mOdomPreIntIMU.mJgRij);
    }
  }
  return is.good();
}
bool Frame::write(ostream &os) const{
  //we don't save old ID for it's useless in LoadMap()
  os.write((char*)&mTimeStamp,sizeof(mTimeStamp));
//   os.write((char*)&mfGridElementWidthInv,sizeof(mfGridElementWidthInv));os.write((char*)&mfGridElementHeightInv,sizeof(mfGridElementHeightInv));//we can get these from mnMaxX...
  os.write((char*)&fx,sizeof(fx));os.write((char*)&fy,sizeof(fy));os.write((char*)&cx,sizeof(cx));os.write((char*)&cy,sizeof(cy));
//   os.write((char*)&invfx,sizeof(invfx));os.write((char*)&invfy,sizeof(invfy));//also from the former ones
  os.write((char*)&mbf,sizeof(mbf));os.write((char*)&mThDepth,sizeof(mThDepth));
//   os.write((char*)&mb,sizeof(mb));//=mbf/fx
  os.write((char*)&N,sizeof(N));
  KeyFrame::writeVec(os, mvKeys);
  KeyFrame::writeVec(os, mvKeysUn);
  KeyFrame::writeVec(os,mvuRight);KeyFrame::writeVec(os,mvDepth);
  KeyFrame::writeMat(os,mDescriptors);
//   mBowVec.write(os);mFeatVec.write(os);//we can directly ComputeBoW() from mDescriptors
  os.write((char*)&mnScaleLevels,sizeof(mnScaleLevels));os.write((char*)&mfScaleFactor,sizeof(mfScaleFactor));
//   os.write((char*)&mfLogScaleFactor,sizeof(mfLogScaleFactor));os.write((char*)&mvScaleFactors,sizeof(mvScaleFactors));//we can get these from former 2 parameters
//   writeVec(os,mvLevelSigma2);writeVec(os,mvInvLevelSigma2);
  os.write((char*)&mnMinX,sizeof(mnMinX));os.write((char*)&mnMinY,sizeof(mnMinY));os.write((char*)&mnMaxX,sizeof(mnMaxX));os.write((char*)&mnMaxY,sizeof(mnMaxY));
//   writeMat(os,mK);from fx~cy
  //save mvpMapPoints in LoadMap for convenience
//   os.write((char*)&mHalfBaseline,sizeof(mHalfBaseline));//=mb/2;
  //save mNavState
  const double* pdData;
  Eigen::Quaterniond q=mNavState.mRwb.unit_quaternion();//qwb from Rwb
  pdData=mNavState.mpwb.data();os.write((const char*)pdData,sizeof(*pdData)*3);//txyz
  pdData=q.coeffs().data();os.write((const char*)pdData,sizeof(*pdData)*4);//qxyzw
  pdData=mNavState.mvwb.data();os.write((const char*)pdData,sizeof(*pdData)*3);//vxyz
  pdData=mNavState.mbg.data();os.write((const char*)pdData,sizeof(*pdData)*3);//bgxyz_bar
  pdData=mNavState.mba.data();os.write((const char*)pdData,sizeof(*pdData)*3);//baxyz_bar
  pdData=mNavState.mdbg.data();os.write((const char*)pdData,sizeof(*pdData)*3);//dbgxyz
  pdData=mNavState.mdba.data();os.write((const char*)pdData,sizeof(*pdData)*3);//dbaxyz
//   for(unsigned int i=0; i<FRAME_GRID_COLS;i++) for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){ size_t nSize;os.write((char*)&nSize,sizeof(nSize));writeVec(os,mGrid[i][j]);}//we can still get it from mvKeysUn
  //save mOdomPreIntOdom, code starting from here is diffrent from KeyFrame::write()
  double tm=mOdomPreIntEnc.mdeltatij;
  os.write((char*)&tm,sizeof(tm));
  if (tm>0){
    KeyFrame::writeEigMat(os,mOdomPreIntEnc.mdelxEij);
    KeyFrame::writeEigMat(os,mOdomPreIntEnc.mSigmaEij);
  }
  tm=mOdomPreIntIMU.mdeltatij;
  os.write((char*)&tm,sizeof(tm));
  if (tm>0){//for IMUPreIntegratorBase<IMUDataBase>
    KeyFrame::writeEigMat(os,mOdomPreIntIMU.mpij);KeyFrame::writeEigMat(os,mOdomPreIntIMU.mRij);KeyFrame::writeEigMat(os,mOdomPreIntIMU.mvij);//PRV
    KeyFrame::writeEigMat(os,mOdomPreIntIMU.mSigmaijPRV);KeyFrame::writeEigMat(os,mOdomPreIntIMU.mSigmaij);
    KeyFrame::writeEigMat(os,mOdomPreIntIMU.mJgpij);KeyFrame::writeEigMat(os,mOdomPreIntIMU.mJapij);
    KeyFrame::writeEigMat(os,mOdomPreIntIMU.mJgvij);KeyFrame::writeEigMat(os,mOdomPreIntIMU.mJavij);
    KeyFrame::writeEigMat(os,mOdomPreIntIMU.mJgRij);
  }
  return os.good();
}

//created by zzh over.

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame(){}

//Copy Constructor
Frame::Frame(const Frame &frame)
    : FrameBase(frame), mpORBvocabulary(frame.mpORBvocabulary), mpORBextractors(frame.mpORBextractors),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N),
      num_mono(frame.num_mono), mvidxsMatches(frame.mvidxsMatches), goodmatches_(frame.goodmatches_),
      mapcamidx2idxs_(frame.mapcamidx2idxs_), mapidxs2n_(frame.mapidxs2n_),
      mv3Dpoints(frame.mv3Dpoints),
      mvKeys(frame.mvKeys), mvKeysUn(frame.mvKeysUn), vvkeys_(frame.vvkeys_), vvkeys_un_(frame.vvkeys_un_),
      mapin2n_(frame.mapin2n_),
      mvuRight(frame.mvuRight), mvDepth(frame.mvDepth),
     mDescriptors(frame.mDescriptors.clone()),
      mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2) {
  vdescriptors_.resize(frame.vdescriptors_.size());
  for (int i = 0; i < vdescriptors_.size(); ++i) vdescriptors_[i] = frame.vdescriptors_[i].clone();
  vgrids_ = frame.vgrids_;

  if (!frame.mTcw.empty()) SetPose(frame.mTcw);

  // created by zzh
  mOdomPreIntIMU = frame.mOdomPreIntIMU;
  mOdomPreIntEnc = frame.mOdomPreIntEnc;  // list uncopied
  mNavState = frame.mNavState;
  mMargCovInv = frame.mMargCovInv;
  mNavStatePrior = frame.mNavStatePrior;
  mbPrior = frame.mbPrior;
}


Frame::Frame(const vector<cv::Mat> &ims, const double &timeStamp, vector<ORBextractor*> extractors, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
             const vector<GeometricCamera *> *pCamInsts, bool usedistort)
    :mpORBvocabulary(voc), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL)),
     mbPrior(false) //zzh
{
  // Frame ID
  mnId=nNextId++;
  usedistort_ = usedistort;

  CV_Assert(ims.size() == extractors.size());
  mpORBextractors.resize(extractors.size());
  for (int i = 0 ; i < extractors.size(); ++i) {
    mpORBextractors[i] = extractors[i];
  }
  // Scale Level Info
  mnScaleLevels = mpORBextractors[0]->GetLevels();
  mfScaleFactor = mpORBextractors[0]->GetScaleFactor();
  mfLogScaleFactor = log(mfScaleFactor);
  mvScaleFactors = mpORBextractors[0]->GetScaleFactors();
  mvInvScaleFactors = mpORBextractors[0]->GetInverseScaleFactors();
  mvLevelSigma2 = mpORBextractors[0]->GetScaleSigmaSquares();
  mvInvLevelSigma2 = mpORBextractors[0]->GetInverseScaleSigmaSquares();

    // ORB extraction
    vector<thread> threads(ims.size());
    CV_Assert(vvkeys_.size() == vdescriptors_.size() && num_mono.size() == vvkeys_.size());
    size_t n_size = mpORBextractors.size();
    vdescriptors_.resize(n_size);
    vvkeys_.resize(n_size);
    num_mono.resize(n_size);
    for (int i = 0; i <ims.size(); ++i) {
      if (pCamInsts) {
        CV_Assert(ims.size() == pCamInsts->size());
        if ((*pCamInsts)[i]->CAM_FISHEYE == (*pCamInsts)[i]->GetType())
          threads[i] = thread(&Frame::ExtractORB, this, i, ims[i],
                              &static_cast<KannalaBrandt8 *>((*pCamInsts)[i])->mvLappingArea);
        else if ((*pCamInsts)[i]->CAM_PINHOLE == (*pCamInsts)[i]->GetType())
          threads[i] = thread(&Frame::ExtractORB, this, i, ims[i], nullptr);
        else
          CV_Assert(0 && "No such cam model");
      } else
        threads[i] = thread(&Frame::ExtractORB, this, i, ims[i], nullptr);
    }
    for (int i = 0; i <ims.size(); ++i) {
      threads[i].join();
    }

    if (!pCamInsts) {
      mvKeys = vvkeys_[0];
      mDescriptors = vdescriptors_[0].clone();
      N = mvKeys.size();
      if(!N) return;

      UndistortKeyPoints();

      CV_Assert(2 == ims.size());
      ComputeStereoMatches();
    }
    else {
      mpCameras.resize(pCamInsts->size());
      for (int i = 0; i < pCamInsts->size(); ++i) {
        mpCameras[i] = (*pCamInsts)[i];
      }

      ComputeStereoFishEyeMatches();
      if (!usedistort_) UndistortKeyPoints();
    }

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(ims[0]);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mbPrior(false)//zzh
{
    // Frame ID
    mnId=nNextId++;

    mpORBextractors.resize(1);
    mpORBextractors[0] = extractor;
    // Scale Level Info
    mnScaleLevels = mpORBextractors[0]->GetLevels();
    mfScaleFactor = mpORBextractors[0]->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractors[0]->GetScaleFactors();
    mvInvScaleFactors = mpORBextractors[0]->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractors[0]->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractors[0]->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    mvKeys = vvkeys_[0];
    mDescriptors = vdescriptors_[0].clone();
    N = mvKeys.size();

    if(!N) return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));//for directly associated vector type mvpMapPoints,used in KF::AddMapPiont()
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

	//divide the img into 64*48(rows) grids for features matching!
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;//metre

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mbPrior(false)//zzh
{
    // Frame ID
    mnId=nNextId++;

    mpORBextractors.resize(1);
    mpORBextractors[0] = extractor;
    // Scale Level Info
    mnScaleLevels = mpORBextractors[0]->GetLevels();
    mfScaleFactor = mpORBextractors[0]->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractors[0]->GetScaleFactors();
    mvInvScaleFactors = mpORBextractors[0]->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractors[0]->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractors[0]->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    mvKeys = vvkeys_[0];
    mDescriptors = vdescriptors_[0].clone();
    N = mvKeys.size();

    if(!N) return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid() {
  size_t n_cams = !mpCameras.size() ? 1 : vvkeys_.size();
  vgrids_.resize(n_cams, vector<vector<vector<size_t>>>(FRAME_GRID_COLS, vector<vector<size_t>>(FRAME_GRID_ROWS)));
  int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS * n_cams);
  for (size_t cami = 0; cami < n_cams; ++cami) {
    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
      for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) vgrids_[cami][i][j].reserve(nReserve);

    if (!mpCameras.size()) CV_Assert(vvkeys_[cami].size() == mvKeysUn.size());
    for (int i = 0; i < vvkeys_[cami].size(); i++) {
      const cv::KeyPoint &kp = (!mpCameras.size() || !usedistort_) ? mvKeysUn[i] : vvkeys_[cami][i];

      int nGridPosX, nGridPosY;
      size_t mapi = !mapin2n_.size() ? i : mapin2n_[cami][i];
      if (PosInGrid(kp, nGridPosX, nGridPosY)) {
        vgrids_[cami][nGridPosX][nGridPosY].push_back(mapi);
      }
    }
  }
}

void Frame::ExtractORB(int flag, const cv::Mat &im, std::vector<int> *pvLappingArea) {
  num_mono[flag] = (*mpORBextractors[flag])(im, cv::Mat(), vvkeys_[flag], vdescriptors_[flag], pvLappingArea);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

//TODO:extend this to 4 cams
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
  bool bret = false;
  size_t n_cams = !mpCameras.size() ? 1 : mpCameras.size();
  pMP->mbTrackInView = bret;
  pMP->vbtrack_inview.clear();
  pMP->vbtrack_inview.resize(n_cams, false);
  for (int i = 0; i < 3; ++i) {
    pMP->vtrack_proj[i].resize(n_cams, -1);
  }
  pMP->vtrack_scalelevel.resize(n_cams);
  pMP->vtrack_viewcos.resize(n_cams);

  // 3D in absolute coordinates
  cv::Mat wP = pMP->GetWorldPos();
  cv::Mat Pn = pMP->GetNormal();
  const float maxDistance = pMP->GetMaxDistanceInvariance();
  const float minDistance = pMP->GetMinDistanceInvariance();
  cv::Mat Rcrw = mTcw.rowRange(0,3).colRange(0,3);
  // 3D in camera coordinates
  const cv::Mat Pcr = mRcw * wP + mtcw;
  pMP->vtrack_cami.clear();
  for (size_t cami = 0; cami < n_cams; ++cami) {
    // TODO: unify this cycle with SBP to one func
    Vector3d Pc = Converter::toVector3d(Pcr);
    GeometricCamera *pcam1 = nullptr;
    cv::Mat twc = mOw.clone(); // wO
    if (mpCameras.size() > cami) {
      pcam1 = mpCameras[cami];
      Pc = pcam1->Rcr_ * Pc + pcam1->tcr_;
      twc += Rcrw.t() * pcam1->Trc_.col(3);
    }
    const float &PcZ = Pc(2);

    // Check positive depth
    if (PcZ < 0.0f) continue;

    // Project in image and check it is not outside
    const float invz = 1.0f / PcZ;
    float u, v;
    if (!mpCameras.size() || !usedistort_) {
      const float &PcX = Pc(0);
      const float &PcY = Pc(1);
      u = fx * PcX * invz + cx;  // K*Xc
      v = fy * PcY * invz + cy;
    } else {
      CV_Assert(pcam1);
      auto pt = pcam1->project(Pc);
      u = pt[0];
      v = pt[1];
    }

    if (u < mnMinX || u > mnMaxX) continue;
    if (v < mnMinY || v > mnMaxY) continue;

    // Check viewing angle
    const cv::Mat PO = wP - twc;
    const float dist3D = cv::norm(PO);
    // Check distance is in the scale invariance region of the MapPoint
    // if it's out of the frustum, image pyramid is not effective
    if (dist3D < minDistance || dist3D > maxDistance) continue;
    // use nbar(P) instead of n(Frame) because the SBP() use the best descriptor of MapPoint
    const float viewCos = PO.dot(Pn) / dist3D;
    // if viewing angle(vec OP && vec nbar(P)) > arccos(viewingCosLimit)(here is 60 degrees), it's not in Frustum
    if (viewCos < viewingCosLimit) continue;
    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist3D, this);

    // Data used by the tracking
    pMP->vbtrack_inview[cami] = true;
    pMP->vtrack_proj[0][cami] = u;
    pMP->vtrack_proj[1][cami] = v;
    pMP->vtrack_proj[2][cami] = u - mbf * invz;  // ur=ul-b*fx/dl
    pMP->vtrack_scalelevel[cami] = nPredictedLevel;
    pMP->vtrack_viewcos[cami] = viewCos;
    pMP->vtrack_cami.push_back(cami);
    bret = true;
  }
  pMP->mbTrackInView = bret;

  return bret;
}

vector<size_t> Frame::GetFeaturesInArea(size_t cami, const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);//minLevel==0&&maxLevel<0 don't need to judge

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = vgrids_[cami][ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (!mpCameras.size() || !usedistort_) ? mvKeysUn[vCell[j]] : mvKeys[vCell[j]];
                if(bCheckLevels)//if the octave is out of level range
                {
                    if(kpUn.octave<minLevel)//-1 is also ok,0 cannot be true
                        continue;
                    if(maxLevel>=0)//avoid for -1
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)//find the features in a rectangle window whose centre is (x,y)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);//transform Mat(N*32*8U) to vec<Mat>(N*1*32*8U)
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);//compute mBowVec && mFeatVec(at level (d-levelsup)6-4) of this Frame
    }
}

void Frame::UndistortKeyPoints() {
  if (mDistCoef.at<float>(0) == 0.0 && !mpCameras.size()) {
    mvKeysUn = mvKeys;
    vvkeys_un_ = vvkeys_;
    return;
  }

  // Fill matrix with points
  cv::Mat mat(N, 2, CV_32F);
  if (!mpCameras.size()) {
    for (int i = 0; i < N; i++) {
      mat.at<float>(i, 0) = mvKeys[i].pt.x;
      mat.at<float>(i, 1) = mvKeys[i].pt.y;
    }
    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);
  } else {
    for (int i = 0; i < N; ++i) {
      size_t cami = get<0>(mapn2in_[i]);
      cv::Mat mattmp = mpCameras[cami]->toK() * mpCameras[cami]->unprojectMat(mvKeys[i].pt);
      mat.at<float>(i, 0) = mattmp.at<float>(0);
      mat.at<float>(i, 1) = mattmp.at<float>(1);
    }
  }

  // Fill undistorted keypoint vector
  mvKeysUn.resize(N);
  for (int i = 0; i < N; i++) {
    cv::KeyPoint kp = mvKeys[i];
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    mvKeysUn[i] = kp;
  }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
  if (mDistCoef.at<float>(0) != 0.0 || (mpCameras.size() && !usedistort_)) {
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.0;
    mat.at<float>(0, 1) = 0.0;
    mat.at<float>(1, 0) = imLeft.cols;
    mat.at<float>(1, 1) = 0.0;
    mat.at<float>(2, 0) = 0.0;
    mat.at<float>(2, 1) = imLeft.rows;
    mat.at<float>(3, 0) = imLeft.cols;
    mat.at<float>(3, 1) = imLeft.rows;

    // Undistort corners
    if (!mpCameras.size()) {
      mat = mat.reshape(2);
      cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
      mat = mat.reshape(1);
    } else {
      // TODO: limit different cam model's border
      size_t cami = 0;
      for (int i = 0; i < 4; ++i) {
        cv::Mat mattmp = mpCameras[cami]->toK() *
                         mpCameras[cami]->unprojectMat(cv::Point2f(mat.at<float>(i, 0), mat.at<float>(i, 1)));
        mat.at<float>(i, 0) = mattmp.at<float>(0);
        mat.at<float>(i, 1) = mattmp.at<float>(1);
      }
    }

    mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
  } else {
    mnMinX = 0.0f;
    mnMaxX = imLeft.cols;
    mnMinY = 0.0f;
    mnMaxY = imLeft.rows;
  }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractors[0]->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = vvkeys_[1].size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = vvkeys_[1][iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[vvkeys_[1][iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = vvkeys_[0][iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = vvkeys_[1][iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = vdescriptors_[1].row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = vvkeys_[1][bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractors[0]->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractors[1]->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractors[1]->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

//size_t Frame::GetIndexIlessJ(int i, int j) {
//  if (i > j) swap(i, j);
//  size_t n = vvkeys_.size();
//  return (2 * n - 1 - i) * i / 2 + j - i - 1;
//}

void Frame::ComputeStereoFishEyeMatches() {
  // TODO: test 4 cams
  CV_Assert(mpCameras.size() == 2);
  // Speed it up by matching keypoints in the lapping area
  size_t n_cams = vvkeys_.size();

  // Perform a brute force between Keypoint in the all images
  vector<vector<vector<cv::DMatch>>> allmatches;
  for (int i = 0; i < n_cams; ++i) CV_Assert(-1 != num_mono[i]);
  for (int i = 0; i < n_cams - 1; ++i) {
    for (int j = i + 1; j < n_cams; ++j) {
      allmatches.push_back(vector<vector<cv::DMatch>>());
      BFmatcher.knnMatch(vdescriptors_[i].rowRange(num_mono[i], vdescriptors_[i].rows),
                         vdescriptors_[j].rowRange(num_mono[j], vdescriptors_[j].rows), allmatches.back(), 2);
    }
  }

  int nMatches = 0;
  int descMatches = 0;

  // Check matches using Lowe's ratio
  CV_Assert(!goodmatches_.size() && !mapcamidx2idxs_.size() && !mvidxsMatches.size());
  vector<vector<double>> lastdists;
  aligned_vector<Vector3d> pts;
  for (size_t i = 0, idmatches = 0; i < n_cams - 1; ++i) {
    for (size_t j = i + 1; j < n_cams; ++j, ++idmatches) {
      auto &matches = allmatches[idmatches];
      for (vector<vector<cv::DMatch>>::iterator it = matches.begin(); it != matches.end(); ++it) {
        if ((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7) {
          size_t idxi = (*it)[0].queryIdx + num_mono[i], idxj = (*it)[0].trainIdx + num_mono[j];
          auto camidxi = make_pair(i, idxi), camidxj = make_pair(j, idxj);
          auto iteri = mapcamidx2idxs_.find(camidxi), iterj = mapcamidx2idxs_.find(camidxj);
          if (iteri == mapcamidx2idxs_.end() && iterj != mapcamidx2idxs_.end()) {
            iteri = iterj;
          }
          uint8_t checkdepth[2] = {0};  // 1 means create, 2 means replace
          size_t ididxs;
          if (iteri != mapcamidx2idxs_.end()) {
            ididxs = iteri->second;
            auto &idxs = mvidxsMatches[ididxs];
            if (-1 == idxs[i]){// || lastdists[ididxs][i] > (*it)[0].distance) {
              checkdepth[0] = 2;
            }
            //            else if (idxi != idxs[i])
            //              goodmatches_[ididxs] = false;
            if (-1 == idxs[j]){// || lastdists[ididxs][j] > (*it)[0].distance) {
              checkdepth[1] = 2;
            }
            //            else if (idxj != idxs[j])
            //              goodmatches_[ididxs] = false;
          } else {
            checkdepth[0] = 1;
            checkdepth[1] = 1;
          }
          if (checkdepth[0] || checkdepth[1]) {
            cv::Mat p3D;
            vector<float> sigmas = {mvLevelSigma2[vvkeys_[i][idxi].octave], mvLevelSigma2[vvkeys_[j][idxj].octave]};
            float depth2;
            float depth1;
            if (mpCameras[i]->CAM_FISHEYE == mpCameras[i]->GetType())
            depth1 = static_cast<KannalaBrandt8 *>(mpCameras[i])
                               ->TriangulateMatches(mpCameras[j], vvkeys_[i][idxi], vvkeys_[j][idxj], sigmas[0],
                                                    sigmas[1], p3D, &depth2);
            else
              depth1 = static_cast<Pinhole *>(mpCameras[i])
                  ->TriangulateMatches(mpCameras[j], vvkeys_[i][idxi], vvkeys_[j][idxj], sigmas[0],
                                       sigmas[1], p3D, &depth2);
            //cout << "dp21="<<depth2 <<" "<<depth1<<endl;
            if (depth1 > 0.0001f && depth2 > 0.0001f) {
              vector<size_t> idxs(n_cams, -1);
              if (checkdepth[0]) {
                idxs[i] = idxi;
              }
              if (checkdepth[1]) {
                idxs[j] = idxj;
              }
              if (1 == checkdepth[0]) {
                CV_Assert(1 == checkdepth[1]);
                ididxs = mvidxsMatches.size();
                mapcamidx2idxs_.emplace(camidxi, ididxs);
                mapcamidx2idxs_.emplace(camidxj, ididxs);
                mvidxsMatches.push_back(idxs);
                mv3Dpoints.resize(mvidxsMatches.size());
                goodmatches_.push_back(true);
                vector<double> dists(n_cams, INFINITY);
                dists[i] = (*it)[0].distance;
                dists[j] = (*it)[0].distance;
                lastdists.push_back(dists);
              }
              mv3Dpoints[ididxs] =
                  Converter::toVector3d(p3D.clone());  // here should return pt in cami's ref frame, usually 0
              nMatches++;
            }
          }
          descMatches++;
        }
      }
    }
  }
  if (n_cams > 2) {
    nMatches = 0;
    for (size_t i = 0; i < mvidxsMatches.size(); ++i) {
      if (!goodmatches_[i]) continue;
      // For every good match, check parallax and reprojection error to discard spurious matches
      cv::Mat p3D;
      auto &idx = mvidxsMatches[i];
      vector<float> sigmas;
      for (int k = 0; k < idx.size(); ++k) sigmas.push_back(mvLevelSigma2[vvkeys_[k][idx[k]].octave]);
      float depth2;
      float depth = static_cast<KannalaBrandt8 *>(mpCameras[0])
                        ->TriangulateMatches(mpCameras[1], vvkeys_[0][idx[0]], vvkeys_[1][idx[1]], sigmas[0], sigmas[1],
                                             p3D, &depth2);
      // cout << "dp21="<<depth2 <<" "<<depth<<endl;
      if (depth > 0.0001f && depth2 > 0.0001f) {
        // mccMatches[0][ij][k] = k2;
        // mccMatches[1][ij][k2] = k;
        // mvStereo3Dpoints[ij][k] = p3D.clone();
        mv3Dpoints[i] = Converter::toVector3d(p3D.clone());  // here should return pt in cami's ref frame, usually 0
        // vv_depth_[0][ij][k] = depth;
        // vv_depth_[1][ij][k2] = depth2;
        nMatches++;
      } else
        goodmatches_[i] = false;
    }
  }
  PRINT_DEBUG_INFO_MUTEX("match num=" << nMatches << endl, imu_tightly_debug_path, "debug.txt");

  CV_Assert(!mvDepth.size() && !mvuRight.size());
  size_t num_pt_added = 0;
  for (size_t i = 0; i < n_cams; ++i) {
    if (mDescriptors.empty() && vdescriptors_.size()) {
      mDescriptors = vdescriptors_[i].clone();
    } else
      cv::vconcat(mDescriptors, vdescriptors_[i], mDescriptors);
    for (size_t k = 0; k < vvkeys_[i].size(); ++k) {
      auto camidx = make_pair(i, k);
      auto iteridxs = mapcamidx2idxs_.find(camidx);
      if (iteridxs != mapcamidx2idxs_.end()) {
        auto &ididxs = iteridxs->second;
        if (-1 != ididxs && goodmatches_[ididxs]) {
          auto &idxs = mvidxsMatches[ididxs];
          Matrix3d Rcr = ((KannalaBrandt8 *)mpCameras[i])->Rcr_;
          Vector3d tcr = ((KannalaBrandt8 *)mpCameras[i])->tcr_;
          Vector3d x3Dc = Rcr * mv3Dpoints[ididxs] + tcr;
          mvDepth.push_back(x3Dc[2]);
        } else
          mvDepth.push_back(-1);
      } else {
        mvDepth.push_back(-1);
      }
      mvuRight.push_back(-1);
      mvKeys.push_back(vvkeys_[i][k]);
      mapn2in_.push_back(make_pair(i, k));
      ++num_pt_added;
    }
  }

  if (mapin2n_.size() < vvkeys_.size()) mapin2n_.resize(vvkeys_.size());
  mapidxs2n_.resize(mv3Dpoints.size(), -1);
  for (size_t k = 0; k < mvKeys.size(); ++k) {
    size_t cami = get<0>(mapn2in_[k]);
    CV_Assert(mapin2n_.size() > cami);
    if (mapin2n_[cami].size() < vvkeys_[cami].size()) mapin2n_[cami].resize(vvkeys_[cami].size());
    CV_Assert(mapin2n_[cami].size() > get<1>(mapn2in_[k]));
    mapin2n_[cami][get<1>(mapn2in_[k])] = k;
    auto iteridxs = mapcamidx2idxs_.find(mapn2in_[k]);
    if (iteridxs != mapcamidx2idxs_.end()) mapidxs2n_[iteridxs->second] = k;
  }
  N = num_pt_added;
  CV_Assert(N == mvKeys.size());
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;//here maybe <0 and >=mnMaxX, suppose [mnMinX,mnMaxX), is there some problem?
        }
    }
}

size_t Frame::GetMapn2idxs(size_t i) {
  auto iteridx = mapcamidx2idxs_.find(mapn2in_[i]);
  if (iteridx == mapcamidx2idxs_.end())
    return -1;
  else
    return iteridx->second;
}

cv::Mat Frame::UnprojectStereo(const int &i) {
  //if (mapn2ijn_.size() > i) {
  if (mapn2in_.size() > i) {
    const float z = mvDepth[i];
    if (z > 0) {
//      cv::Mat Rrc = ((KannalaBrandt8 *)mpCameras[get<0>(mapn2in_[i])])->Trc_.rowRange(0, 3).colRange(0, 3);
//      cv::Mat trc = ((KannalaBrandt8 *)mpCameras[get<0>(mapn2in_[i])])->Trc_.col(3);
//      cv::Mat x3Dc = ((KannalaBrandt8 *)mpCameras[get<0>(mapn2in_[i])])->unprojectMat(mvKeys[i].pt) * z;  // x_Cr
//      x3Dc = Rrc * x3Dc + trc;

      auto ididxs = GetMapn2idxs(i);
      CV_Assert(-1 != ididxs && goodmatches_[ididxs]);
      Vector3d x3Dw = Converter::toMatrix3d(mRwc) * mv3Dpoints[ididxs] + Converter::toVector3d(mOw);
      // cout << "check w3Dw=" << x3Dw.transpose() << ", z=" << z << ", xy=" << mvKeys[i].pt << " " << endl;
//           << (mRwc * x3Dc + mOw).t() << endl;

      return Converter::toCvMat(x3Dw);
    } else {
      return cv::Mat();
    }
  }

  const float z = mvDepth[i];
  if (z > 0) {
    const float u = mvKeysUn[i].pt.x;
    const float v = mvKeysUn[i].pt.y;
    const float x = (u - cx) * z * invfx;
    const float y = (v - cy) * z * invfy;
    cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
    return mRwc * x3Dc + mOw;
  } else
    return cv::Mat();
}

} //namespace ORB_SLAM
