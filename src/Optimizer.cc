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

#include "Optimizer.h"
#include "common/log.h"

#ifdef USE_G2O_NEWEST
#include "g2o/solvers/dense/linear_solver_dense.h"
#else
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#endif

#include <Eigen/StdVector>

#include "Converter.h"

#include <mutex>

namespace VIEO_SLAM {  // changed a lot refering to the JingWang's code

using namespace Eigen;

void Optimizer::LocalBAPRVIDP(KeyFrame* pCurKF, int Nlocal, bool* pbStopFlag, Map* pMap, cv::Mat& gw) {}
void Optimizer::LocalBundleAdjustmentNavStatePRV(KeyFrame* pKF, int Nlocal, bool* pbStopFlag, Map* pMap, cv::Mat gw) {
  // Gravity vector in world frame
  Vector3d GravityVec = Converter::toVector3d(gw);

  // strategy refering the VIORBSLAM paper Fig.3.
  list<KeyFrame*> lLocalKeyFrames;
  // All KeyFrames in Local window are optimized, get N last KFs as Local Window
  const int NlocalIni = Nlocal;  // for assert
  KeyFrame* pKFlocal = pKF;
  do {
    assert(pKFlocal && !pKFlocal->isBad() && "!pKFi. why??????");
    pKFlocal->mnBALocalForKF = pKF->mnId;  // avoid adding it into lFixedCameras
    lLocalKeyFrames.push_front(pKFlocal);  // notice the order is opposite
    pKFlocal = pKFlocal->GetPrevKeyFrame();
  } while (--Nlocal > 0 && pKFlocal != NULL);  // maybe less than N KFs in pMap
  //   assert(pKFlocal!=NULL||pKFlocal==NULL&&pMap->KeyFramesInMap()<=NlocalIni);

  // Local MapPoints seen in Local KeyFrames
  list<MapPoint*> lLocalMapPoints;
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
    vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pKF->mnId)  // avoid duplications
          {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
    }
  }
  PRINT_INFO_MUTEX(blueSTR "Enter local BA..." << pKF->mnId << ", size of localKFs=" << lLocalKeyFrames.size()
                                               << ", mps=" << lLocalMapPoints.size() << whiteSTR << endl);

  // Fixed Keyframes. Keyframes that see Local MapPoints and last (N+1)th KF but that are not Local Keyframes: \
  2nd layer fixed neighbors(don't optimize them but contribute to the target min funcation)
  list<KeyFrame*> lFixedCameras;
  // the last N+1th KF / Add the KeyFrame before local window.
  KeyFrame* pKFPrevLocal = pKFlocal;  // lLocalKeyFrames.front()->GetPrevKeyFrame();
  if (pKFPrevLocal) {
    assert(!pKFPrevLocal->isBad() && pKFPrevLocal->mnBALocalForKF != pKF->mnId && pKF->mnBAFixedForKF != pKF->mnId);
    pKFPrevLocal->mnBAFixedForKF = pKF->mnId;
    if (!pKFPrevLocal->isBad()) lFixedCameras.push_back(pKFPrevLocal);
  }  // else means mpMap->KeyFramesInMap()<N+1 / pKFPrevLocal point to nullptr
  // Covisibility neighbors
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    auto observations = (*lit)->GetObservations();
    for (auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId)  // avoid duplications in lLocalKeyFrames && lFixedCameras
      {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad()) lFixedCameras.push_back(pKFi);
      }
    }
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolverX>(new g2o::BlockSolverX(unique_ptr<g2o::BlockSolverX::LinearSolverType>(
          new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()))));  // LM descending method
#else
  g2o::BlockSolverX::LinearSolverType* linearSolver;  // 6*1 is PR:t,Log(R), 3*1 is V:v, 6*1 is Bias/B:bgi,bai, 3*1 is
                                                      // location of landmark, 4 types of vertices so using BlockSolverX

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();  // sparse Cholesky solver, similar to CSparse

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // LM descending method
#endif
  optimizer.setAlgorithm(solver);

  if (pbStopFlag)  // if &mbAbortBA !=nullptr, true in LocalMapping
    optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // Set Local KeyFrame vertices
  for (list<KeyFrame*>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend;
       ++lit) {
    KeyFrame* pKFi = *lit;
    int idKF = pKFi->mnId * 3;  // PRi,Vi,Biasi
    bool bFixed = pKFi->mnId == 0;
    NavState ns(pKFi->GetNavState());
    // Vertex of PR/V
    g2o::VertexNavStatePR* vNSPR = new g2o::VertexNavStatePR();
    vNSPR->setEstimate(ns);
    vNSPR->setId(idKF);
    vNSPR->setFixed(bFixed);
    optimizer.addVertex(vNSPR);
    g2o::VertexNavStateV* vNSV = new g2o::VertexNavStateV();
    vNSV->setEstimate(ns);
    vNSV->setId(idKF + 1);
    vNSV->setFixed(bFixed);
    optimizer.addVertex(vNSV);
    // Vertex of Bias
    g2o::VertexNavStateBias* vNSBias = new g2o::VertexNavStateBias();
    vNSBias->setEstimate(ns);
    vNSBias->setId(idKF + 2);
    vNSBias->setFixed(bFixed);
    optimizer.addVertex(vNSBias);
    if (idKF + 2 > maxKFid) maxKFid = idKF + 2;  // update maxKFid
  }

  // Set Fixed KeyFrame vertices. Including the pKFPrevLocal. see VIORBSLAM paper Fig.3.
  for (list<KeyFrame*>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    int idKF = pKFi->mnId * 3;
    NavState ns(pKFi->GetNavState());
    // For common fixed KeyFrames, only add PR vertex
    g2o::VertexNavStatePR* vNSPR = new g2o::VertexNavStatePR();
    vNSPR->setEstimate(ns);
    vNSPR->setId(idKF);
    vNSPR->setFixed(true);
    optimizer.addVertex(vNSPR);
    // For Local-Window-Previous KeyFrame, add V and Bias vertex
    if (pKFi == pKFPrevLocal) {
      g2o::VertexNavStateV* vNSV = new g2o::VertexNavStateV();
      vNSV->setEstimate(ns);
      vNSV->setId(idKF + 1);
      vNSV->setFixed(true);
      optimizer.addVertex(vNSV);
      g2o::VertexNavStateBias* vNSBias = new g2o::VertexNavStateBias();
      vNSBias->setEstimate(ns);
      vNSBias->setId(idKF + 2);
      vNSBias->setFixed(true);
      optimizer.addVertex(vNSBias);
    }
    if (idKF + 2 > maxKFid) maxKFid = idKF + 2;
  }

  // Set IMU/KF-KF/PRV(B)+B edges, here (B) means it's not included in error but used to calculate the error
  //  PRVB/IMU & B/RandomWalk edge
  vector<g2o::EdgeNavStatePRV*> vpEdgesNavStatePRV;
  vector<g2o::EdgeNavStateBias*> vpEdgesNavStateBias;
  // what does this 10(sigma) mean??? better result?
  const float thHuberNavStatePRV =
      sqrt(16.919);  // chi2(0.05/0.01,9), 16.919/21.666 for 0.95/0.99 9DoF, but JingWang uses 100*21.666
  const float thHuberNavStateBias =
      sqrt(12.592);  // chi2(0.05/0.01,6), 12.592/16.812 for 0.95/0.99 6DoF, but JW uses 100*16.812
  Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
  InvCovBgaRW.topLeftCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmabg2;  // Gyroscope bias random walk, covariance INVERSE
  InvCovBgaRW.bottomRightCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmaba2;  // Accelerometer bias random walk, covariance INVERSE

  cv::Mat Tbe = Frame::mTbc * Frame::mTce;  // for Enc
  Quaterniond qRbe = Quaterniond(Converter::toMatrix3d(Tbe.rowRange(0, 3).colRange(0, 3)));
  Vector3d tbe = Converter::toVector3d(Tbe.rowRange(0, 3).col(3));  // for Enc
  for (list<KeyFrame*>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend;
       lit++) {
    KeyFrame* pKF1 = *lit;                     // Current KF, store the IMU pre-integration between previous-current
    KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();  // Previous KF
    if (!pKF0) continue;
    IMUPreintegrator imupreint = pKF1->GetIMUPreInt();
    CV_Assert(!pKF0->isBad());
    CV_Assert(!pKF1->isBad());
    if (imupreint.mdeltatij == 0) continue;
    // IMU_I/PRV(B) edges
    int idKF0 = 3 * pKF0->mnId, idKF1 = 3 * pKF1->mnId;
    g2o::EdgeNavStatePRV* eprv = new g2o::EdgeNavStatePRV();
    eprv->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));      // PRi 0
    eprv->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));      // PRj 1
    eprv->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 1)));  // Vi 2
    eprv->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 1)));  // Vj 3
    eprv->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 4
    eprv->setMeasurement(imupreint);
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
      eprv->setInformation(imupreint.mSigmaijPRV.inverse() * 1e-2);
    else
      eprv->setInformation(imupreint.mSigmaijPRV.inverse());
    eprv->SetParams(GravityVec);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    eprv->setRobustKernel(rk);
    rk->setDelta(thHuberNavStatePRV);
    optimizer.addEdge(eprv);
    vpEdgesNavStatePRV.push_back(eprv);  // for robust processing/ erroneous edges' culling
    // IMU_RW/Bias edge
    g2o::EdgeNavStateBias* ebias = new g2o::EdgeNavStateBias();
    ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 0
    ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 2)));  // Bj 1
    ebias->setMeasurement(imupreint);
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
      ebias->setInformation(InvCovBgaRW / imupreint.mdeltatij * 1e-2);
    else
      // see Manifold paper (47), notice here is Omega_d/Sigma_d.inverse()
      ebias->setInformation(InvCovBgaRW / imupreint.mdeltatij);
    rk = new g2o::RobustKernelHuber;
    ebias->setRobustKernel(rk);
    rk->setDelta(thHuberNavStateBias);
    optimizer.addEdge(ebias);
    vpEdgesNavStateBias.push_back(ebias);

    // Set Enc edge(binary edge) between LastF-Frame
    const EncPreIntegrator encpreint = pKF1->GetEncPreInt();
    CV_Assert(!pKF0->isBad());
    CV_Assert(!pKF1->isBad());
    if (encpreint.mdeltatij == 0) continue;
    g2o::EdgeEncNavStatePR* eEnc = new g2o::EdgeEncNavStatePR();
    eEnc->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));  // lastF,i
    eEnc->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));  // curF,j
    eEnc->setMeasurement(encpreint.mdelxEij);
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
      eEnc->setInformation(encpreint.mSigmaEij.inverse() * 1e-2);
    else
      // no vbgba problem(camera could not give enough restriction on vbgba) but
      eEnc->setInformation(encpreint.mSigmaEij.inverse());
    // calibration for enc is worse so we add robust kernel here
    rk = new g2o::RobustKernelHuber;
    eEnc->setRobustKernel(rk);
    rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
    eEnc->qRbe = qRbe;
    eEnc->pbe = tbe;  // SetParams
    optimizer.addEdge(eEnc);
  }

  // Set MapPoint vertices && MPs-KFs' edges
  const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();  // max edges'
                                                                                                       // size

  vector<g2o::EdgeReprojectPR*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);
  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);
  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<g2o::EdgeReprojectPRStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);
  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);
  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991);    // sqrt(e_block)<=sqrt(chi2(0.05,2)) allow power 2 increasing(1/2*e_block),
  const float thHuberStereo = sqrt(7.815);  // chi2(0.05,3)
  Pinhole CamInst;
  bool binitcaminst = false;
  // Extrinsics
  Matrix3d Rcb = Frame::meigRcb;
  Vector3d tcb = Frame::meigtcb;

  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; ++lit) {
    // Set MP vertices
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();  //<3,Eigen::Vector3d>, for MPs' Xw
    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
    int id = pMP->mnId + maxKFid + 1;  //>=maxKFid+1
    vPoint->setId(id);
    vPoint->setMarginalized(true);  // P(xc,xp)=P(xc)*P(xp|xc), P(xc) is called marginalized/Schur elimination,
                                    // [B-E*C^(-1)*E.t()]*deltaXc=v-E*C^(-1)*w, H*deltaX=g=[v;w]; used in Sparse solver
    optimizer.addVertex(vPoint);

    const map<KeyFrame*, set<size_t>> observations = pMP->GetObservations();

    // Set edges
    for (map<KeyFrame*, set<size_t>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend;
         ++mit) {
      KeyFrame* pKFi = mit->first;

      if (!pKFi->isBad())  // good pKFobserv then connect it with pMP by an edge
      {
        bool usedistort = Frame::usedistort_ && pKFi->mpCameras.size();
        auto idxs = mit->second;
        for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
          auto idx = *iter;
          const cv::KeyPoint& kpUn = !usedistort ? pKFi->mvKeysUn[idx] : pKFi->mvKeys[idx];

          // Monocular observation
          if (pKFi->mvuRight[idx] < 0) {
            g2o::EdgeReprojectPR* e = new g2o::EdgeReprojectPR();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId * 3)));

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);  // similar to ||e||

            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb);
            }

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);       //_vertices[1]
            vpMapPointEdgeMono.push_back(pMP);  //_vertices[0]
          } else                                // Stereo observation
          {
            g2o::EdgeReprojectPRStereo* e = new g2o::EdgeReprojectPRStereo();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId * 3)));

            Eigen::Matrix<double, 3, 1> obs;
            const float kp_ur = pKFi->mvuRight[idx];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberStereo);

            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb, &pKFi->mbf);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb, &pKFi->mbf);
            }

            optimizer.addEdge(e);
            vpEdgesStereo.push_back(e);
            vpEdgeKFStereo.push_back(pKFi);
            vpMapPointEdgeStereo.push_back(pMP);
          }
        }
      }
    }
  }
  PRINT_INFO_MUTEX("factor_visual num="<<vpEdgesMono.size()<<endl);

  if (pbStopFlag)     // true in LocalMapping
    if (*pbStopFlag)  // if mbAbortBA
      return;

  optimizer.initializeOptimization();
  optimizer.optimize(5);  // maybe stopped by *_forceStopFlag(mbAbortBA) in some step/iteration

  bool bDoMore = true;

  if (pbStopFlag)
    if (*pbStopFlag)  // judge mbAbortBA again
      bDoMore = false;

  if (bDoMore) {
    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeReprojectPR* e = vpEdgesMono[i];
      MapPoint* pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad())  // why this can be true?
        continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive())  // if chi2 error too big(5% wrong) or Zc<=0 then outlier
      {
        e->setLevel(1);
      }

      e->setRobustKernel(0);  // cancel RobustKernel
    }
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeReprojectPRStereo* e = vpEdgesStereo[i];
      MapPoint* pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive())  // chi2(0.05,3)
      {
        e->setLevel(1);
      }

      e->setRobustKernel(0);
    }

    // Optimize again without the outliers
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);  // 10 steps same as motion-only BA
  }

  vector<pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    g2o::EdgeReprojectPR* e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));  // ready to erase outliers of pKFi && pMP in monocular edges
    }
  }
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeReprojectPRStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));  // ready to erase outliers of pKFi && pMP in stereo edges
    }
  }

#ifndef NDEBUG
  {
    double th_chi2 = thHuberNavStatePRV * thHuberNavStatePRV;
    for (size_t i = 0, iend = vpEdgesNavStatePRV.size(); i < iend; i++) {
      g2o::EdgeNavStatePRV* e = vpEdgesNavStatePRV[i];

      // if chi2 error too big(5% wrong) then outlier
      if (e->chi2() > th_chi2) {
        PRINT_DEBUG_INFO_MUTEX("2 PRVedge " << redSTR << i << whiteSTR << ", chi2 " << e->chi2() << ". ",
                               imu_tightly_debug_path, "debug.txt");
      }
    }
    th_chi2 = thHuberNavStateBias * thHuberNavStateBias;
    for (size_t i = 0, iend = vpEdgesNavStateBias.size(); i < iend; i++) {
      g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];

      if (e->chi2() > th_chi2) {
        PRINT_DEBUG_INFO_MUTEX("2 Biasedge " << redSTR << i << whiteSTR << ", chi2 " << e->chi2() << ". ",
                               imu_tightly_debug_path, "debug.txt");
      }
    }
  }
#endif

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {  // erase the relation between outliers of pKFi(matched mvpMapPoints) && pMP(mObservations)
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);  // here may erase pMP in mpMap
    }
  }

  // Recover optimized data

  // Keyframes update(Pose Tbw&Tcw...)
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; ++lit) {
    KeyFrame* pKFi = *lit;
    int idKF = 3 * pKFi->mnId;
    g2o::VertexNavStatePR* vNSPR = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(idKF));
    g2o::VertexNavStateV* vNSV = static_cast<g2o::VertexNavStateV*>(optimizer.vertex(idKF + 1));
    g2o::VertexNavStateBias* vNSBias = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(idKF + 2));
    NavState ns_recov = vNSPR->estimate();
    ns_recov.mvwb = vNSV->estimate().mvwb;
    const NavState& optBiasns = vNSBias->estimate();
    ns_recov.mdbg = optBiasns.mdbg;
    ns_recov.mdba = optBiasns.mdba;  // don't need to update const bi_bar
    pKFi->SetNavState(ns_recov);     // it has already called the UpdatePoseFromNS();
  }

  // Points update(Position, normal), no need to update descriptor for unchanging pMP->mObservations except for the ones
  // having outliers' edges?
  for (list<MapPoint*>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend;
       ++lit)  // we don't change the pointer data in list
  {
    MapPoint* pMP = *lit;  // but we can change pMP(copy) and *pMP
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
    pMP->UpdateNormalAndDepth();
  }

  pMap->InformNewChange();  // zzh
}

void Optimizer::GlobalBundleAdjustmentNavStatePRV(Map* pMap, const cv::Mat& gw, int nIterations, bool* pbStopFlag,
                                                  const unsigned long nLoopKF, const bool bRobust, bool bScaleOpt) {
  vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  vector<MapPoint*> vpMP = pMap->GetAllMapPoints();

  // Gravity vector in world frame
  Vector3d GravityVec = Converter::toVector3d(gw);

  vector<bool> vbNotIncludedMP;  // true means this MP can not be used to optimize some KFs' Pose/is not added into
                                 // optimizer/is not optimized
  vbNotIncludedMP.resize(vpMP.size());

  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolverX>(new g2o::BlockSolverX(unique_ptr<g2o::BlockSolverX::LinearSolverType>(
          new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()))));  // LM method
#else
  g2o::BlockSolverX::LinearSolverType* linearSolver;  // 6,3,6 KFs' Pose(PR),Velocity(V),Bias(B) && 3 MPs' pos

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();  // sparse Cholesky solver

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // LM method
#endif
  optimizer.setAlgorithm(solver);

  if (pbStopFlag)                            // if mbStopGBA exists
    optimizer.setForceStopFlag(pbStopFlag);  //_forceStopFlag=&mbStopGBA

  long unsigned int maxKFid = 0;

  // Set KeyFrame vertices
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->isBad())  // don't add the bad KFs to optimizer
      continue;
    int idKF = pKFi->mnId * 3;  // PRi,Vi,Biasi
    bool bFixed = pKFi->mnId == 0;
    NavState ns(pKFi->GetNavState());
    // Vertex of PR/V
    g2o::VertexNavStatePR* vNSPR = new g2o::VertexNavStatePR();
    vNSPR->setEstimate(ns);
    vNSPR->setId(idKF);
    vNSPR->setFixed(bFixed);
    optimizer.addVertex(vNSPR);
    g2o::VertexNavStateV* vNSV = new g2o::VertexNavStateV();
    vNSV->setEstimate(ns);
    vNSV->setId(idKF + 1);
    vNSV->setFixed(bFixed);
    optimizer.addVertex(vNSV);
    // Vertex of Bias
    g2o::VertexNavStateBias* vNSBias = new g2o::VertexNavStateBias();
    vNSBias->setEstimate(ns);
    vNSBias->setId(idKF + 2);
    vNSBias->setFixed(bFixed);
    optimizer.addVertex(vNSBias);
    if (idKF + 2 > maxKFid) maxKFid = idKF + 2;  // update maxKFid
  }
  // Set Scale vertex
  if (bScaleOpt) {  // if dInitialScale>0 the Full BA includes scale's optimization
    g2o::VertexScale* pvScale = new g2o::VertexScale();
    pvScale->setEstimate(1.);
    pvScale->setId(++maxKFid);
    pvScale->setFixed(false);
    optimizer.addVertex(pvScale);
  }

  // Set IMU/KF-KF/PRV(B)+B edges
  // what does this 10(sigma) mean??? better result?
  const float thHuberNavStatePRV = sqrt(16.919);   // chi2(0.05/0.01,9), 16.919/21.666 for 0.95/0.99 9DoF; 100*
  const float thHuberNavStateBias = sqrt(12.592);  // chi2(0.05/0.01,6), 12.592/16.812 for 0.95/0.99 6DoF; 100*
  Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
  InvCovBgaRW.topLeftCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmabg2;  // Gyroscope bias random walk, covariance INVERSE
  InvCovBgaRW.bottomRightCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmaba2;  // Accelerometer bias random walk, covariance INVERSE

  cv::Mat Tbe = Frame::mTbc * Frame::mTce;  // for Enc
  Quaterniond qRbe = Quaterniond(Converter::toMatrix3d(Tbe.rowRange(0, 3).colRange(0, 3)));
  Vector3d tbe = Converter::toVector3d(Tbe.rowRange(0, 3).col(3));  // for Enc
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF1 = vpKFs[i];  // KFj, store the IMU pre-integration between previous-current
    // way0 to avoid mutex problem is to judge if bad after accessing all pKF1 data which may be changed after set bad
    // way1 to update the may-be-changed data together and judge them to avoid bugs (both used)
    KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();  // Previous KF
    //if no KFi/IMUPreInt's info, this IMUPreInt edge cannot be added for lack of vertices i / edge ij, \
    notice we don't exclude the situation that KFi has no imupreint but KFj has for KFi's NavState is updated in TrackLocalMapWithIMU()
    if (!pKF0) continue;
    IMUPreintegrator imupreint = pKF1->GetIMUPreInt();
    while (!pKF1->isBad() && pKF0->isBad()) { // to ensure imupreint is matched with pKF0
      pKF0 = pKF1->GetPrevKeyFrame();
      imupreint = pKF1->GetIMUPreInt();
    }
    // don't add the bad KFs to optimizer
    if (pKF1->isBad()) continue; // way0
    if (imupreint.mdeltatij == 0) continue; // way1
    // IMU_I/PRV(B) edges
    int idKF0 = 3 * pKF0->mnId, idKF1 = 3 * pKF1->mnId;
    g2o::EdgeNavStatePRV* eprv = new g2o::EdgeNavStatePRV();
    eprv->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));      // PRi 0
    eprv->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));      // PRj 1
    eprv->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 1)));  // Vi 2
    eprv->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 1)));  // Vj 3
    eprv->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 4
    eprv->setMeasurement(imupreint);
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
      eprv->setInformation(imupreint.mSigmaijPRV.inverse() * 1e-2);
    else
      eprv->setInformation(imupreint.mSigmaijPRV.inverse());
    eprv->SetParams(GravityVec);
    if (bRobust) {
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      eprv->setRobustKernel(rk);
      rk->setDelta(thHuberNavStatePRV);
    }  // here false
    optimizer.addEdge(eprv);
    // IMU_RW/Bias edge
    g2o::EdgeNavStateBias* ebias = new g2o::EdgeNavStateBias();
    ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 0
    ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 2)));  // Bj 1
    ebias->setMeasurement(imupreint);
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
      ebias->setInformation(InvCovBgaRW / imupreint.mdeltatij * 1e-2);
    else
      // see Manifold paper (47), notice here is Omega_d/Sigma_d.inverse()
      ebias->setInformation(InvCovBgaRW / imupreint.mdeltatij);
    if (bRobust) {
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      ebias->setRobustKernel(rk);
      rk->setDelta(thHuberNavStateBias);
    }  // here false
    optimizer.addEdge(ebias);

    // Set Enc edge(binary edge) between LastF-Frame
    EncPreIntegrator encpreint = pKF1->GetEncPreInt();
    while (!pKF1->isBad() && pKF0->isBad()) { // to ensure encpreint is matched with pKF0
      pKF0 = pKF1->GetPrevKeyFrame();
      idKF0 = 3 * pKF0->mnId;
      encpreint = pKF1->GetEncPreInt();
    }
    // don't add the bad KFs to optimizer
    if (pKF1->isBad()) continue;
    if (encpreint.mdeltatij == 0) continue;
    g2o::EdgeEncNavStatePR* eEnc = new g2o::EdgeEncNavStatePR();
    eEnc->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));  // lastF,i
    eEnc->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));  // curF,j
    eEnc->setMeasurement(encpreint.mdelxEij);
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
      eEnc->setInformation(encpreint.mSigmaEij.inverse() * 1e-2);
    else
      eEnc->setInformation(encpreint.mSigmaEij.inverse());
    eEnc->qRbe = qRbe;
    eEnc->pbe = tbe;  // SetParams
    if (bRobust) {
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      eEnc->setRobustKernel(rk);
      rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
    }
    optimizer.addEdge(eEnc);
  }

  const float thHuber2D = sqrt(5.99);   // chi2(0.05,2), sqrt(e'*Omega*e)<=delta, here unused
  const float thHuber3D = sqrt(7.815);  // chi2(0.05,3)
  Pinhole CamInst;
  bool binitcaminst = false;
  // Extrinsics
  Matrix3d Rcb = Frame::meigRcb;
  Vector3d tcb = Frame::meigtcb;

  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    MapPoint* pMP = vpMP[i];
    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
    const int id = pMP->mnId + maxKFid + 1;  // same as localBA
    vPoint->setId(id);
    // P(xc,xp)=P(xc)*P(xp|xc), P(xc) is called marginalized/Schur elimination,
    // [B-E*C^(-1)*E.t()]*deltaXc=v-E*C^(-1)*w, H*deltaX=g=[v;w]; used in Sparse solver
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const auto observations = pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (map<KeyFrame*, set<size_t>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {
      KeyFrame* pKFi = mit->first;
      // pKF->mnId*3 may > maxKFid for LocalMapping is recovered, here -1 for VertexScale
      // only connect observation edges to optimized KFs/vertices
      if (pKFi->isBad() || 3 * pKFi->mnId > maxKFid - 2) continue;
      nEdges++;
      bool usedistort = Frame::usedistort_ && pKFi->mpCameras.size();
      auto idxs = mit->second;
      for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
        auto idx = *iter;
        const cv::KeyPoint& kpUn = !usedistort ? pKFi->mvKeysUn[idx] : pKFi->mvKeys[idx];

        if (pKFi->mvuRight[idx] < 0)  // monocular MPs use 2*1 error
        {
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          if (!bScaleOpt) {  // scaled Xw
            g2o::EdgeReprojectPR* e = new g2o::EdgeReprojectPR();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId * 3)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            if (bRobust) {   // here false
              g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuber2D);  // Mono in localBA
            }
            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb);
            }

            optimizer.addEdge(e);
          } else {  // unscaled Xw but scaled pwb
            g2o::EdgeReprojectPRS* e = new g2o::EdgeReprojectPRS();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId * 3)));
            e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(maxKFid)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            if (bRobust) {
              g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuber2D);
            }
            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb);
            }

            optimizer.addEdge(e);
          }
        } else  // stereo MPs uses 3*1 error
        {
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKFi->mvuRight[idx];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          if (!bScaleOpt) {  // scaled Xw
            g2o::EdgeReprojectPRStereo* e = new g2o::EdgeReprojectPRStereo();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId * 3)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

            if (bRobust) {  // here false
              g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuber3D);  // called Stereo in localBA
            }
            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb, &pKFi->mbf);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb, &pKFi->mbf);
            }

            optimizer.addEdge(e);
          } else {  // unscaled Xw but scaled pwb
            g2o::EdgeReprojectPRSStereo * e = new g2o::EdgeReprojectPRSStereo();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId * 3)));
            e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(maxKFid)));
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);
            if (bRobust) {
              g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(thHuber3D);
            }
            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb, &pKFi->mbf);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb, &pKFi->mbf);
            }

            optimizer.addEdge(e);
          }
        }
      }
    }

    if (nEdges == 0)  // if no edges/observations can be made, delete this Vertex from optimizer
    {
      optimizer.removeVertex(vPoint);  // here only do _vertices.erase(it(it->second==vPoint))
      vbNotIncludedMP[i] = true;
    } else {
      vbNotIncludedMP[i] = false;
    }
  }

  // Optimize!
  optimizer.initializeOptimization();
  // 10 same as pure inliers iterations in localBA/motion-only BA/Sim3Motion-only BA,
  // maybe stopped by next CorrectLoop() in LoopClosing
  optimizer.optimize(nIterations);

  // Recover optimized data in an intermediate way
  // Scale for Xw
  double scale = 1.;
  if (bScaleOpt) {
    scale = static_cast<g2o::VertexScale*>(optimizer.vertex(maxKFid))->estimate();
    PRINT_INFO_MUTEX( azureSTR "Recovered scale is " << scale << whiteSTR << endl);
  }

  // Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++)  // all KFs in mpMap
  {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->isBad())  // don't add the bad KFs to optimizer
      continue;
    // get ns_recov(PRVB) from vertices(PR,V,B)
    int idKF = 3 * pKFi->mnId;
    g2o::VertexNavStatePR* vNSPR = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(idKF));
    g2o::VertexNavStateV* vNSV = static_cast<g2o::VertexNavStateV*>(optimizer.vertex(idKF + 1));
    g2o::VertexNavStateBias* vNSBias = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(idKF + 2));
    NavState ns_recov = vNSPR->estimate();
    ns_recov.mvwb = vNSV->estimate().mvwb;
    const NavState& optBiasns = vNSBias->estimate();
    ns_recov.mdbg = optBiasns.mdbg;
    ns_recov.mdba = optBiasns.mdba;  // don't need to update const bi_bar

    // 	ns_recov.mpwb=scale*(ns_recov.mpwb-RwcPcb)+RwcPcb;
    // 	if (nScaleOpt==3)
    // ns_recov.mvwb=scale*ns_recov.mvwb;//+ns_recov.getRwb()*skew(pKFi->GetListIMUData().back().mw)*Rcb.transpose()*tcb*(1-scale);//if
    // we want to use complete unscaled vwb, we need to add rotation speed to the state!
    if (nLoopKF == 0) {
      pKFi->SetNavState(ns_recov);  // it has already called the UpdatePoseFromNS()(SetPose())
    } else {
      pKFi->mNavStateGBA = ns_recov;  // store ns_recov in nsGBA, for direct recover of optimized/old MPs

      pKFi->mTcwGBA.create(4, 4, CV_32F);  // for indirect recover of new MPs
      // get Twc=Twb*Tbc, cv::Mat Twb is from NavState ns_recov(Rwb,pwb); then set TcwGBA
      cv::Mat Twb_ = cv::Mat::eye(4, 4, CV_32F);
      Converter::toCvMat(ns_recov.getRwb()).copyTo(Twb_.rowRange(0, 3).colRange(0, 3));
      Converter::toCvMat(ns_recov.mpwb).copyTo(Twb_.rowRange(0, 3).col(3));
      cv::Mat Twc_ = Twb_ * Frame::mTbc;
      pKFi->mTcwGBA = Converter::toCvMatInverse(Twc_);

      pKFi->mnBAGlobalForKF = nLoopKF;
    }
  }

  // Points
  for (size_t i = 0; i < vpMP.size(); i++)  // all MPs in mpMap
  {
    if (vbNotIncludedMP[i]) continue;
    // if this MP is optimized by g2o

    MapPoint* pMP = vpMP[i];

    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));

    if (!bScaleOpt) {
      if (nLoopKF == 0)  // it's for final Full BA
      {
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
      } else {
        pMP->mPosGBA.create(3, 1, CV_32F);
        Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
        pMP->mnBAGlobalForKF = nLoopKF;
      }
    } else {  // unscaled Xw
      if (nLoopKF == 0) {
        pMP->SetWorldPos(scale * Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
      } else {
        pMP->mPosGBA.create(3, 1, CV_32F);
        Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
        pMP->mPosGBA *= scale;
        pMP->mnBAGlobalForKF = nLoopKF;
      }
    }
  }
}

// created by zzh over.

void Optimizer::GlobalBundleAdjustment(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF,
                                       const bool bRobust, const bool bEnc) {
  vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
  BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust, bEnc);
}

void Optimizer::BundleAdjustment(const vector<KeyFrame*>& vpKFs, const vector<MapPoint*>& vpMP, int nIterations,
                                 bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust, const bool bEnc) {
  PRINT_INFO_MUTEX( redSTR << "Enter GBA" << whiteSTR << endl);
  vector<bool> vbNotIncludedMP;  // true means this MP can not be used to optimize some KFs' Pose/is not added into
                                 // optimizer/is not optimized
  vbNotIncludedMP.resize(vpMP.size());

  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolver_6_3>(new g2o::BlockSolver_6_3(unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(
          new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()))));  // LM method
#else
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;  // 6 KFs' se3 Pose && 3 MPs' pos

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();  // sparse Cholesky solver

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // LM method
#endif
  optimizer.setAlgorithm(solver);

  if (pbStopFlag)                            // if mbStopGBA exists
    optimizer.setForceStopFlag(pbStopFlag);  //_forceStopFlag=&mbStopGBA

  long unsigned int maxKFid = 0;

  // Set KeyFrame vertices
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad()) continue;

    g2o::VertexNavStatePR* vns = new g2o::VertexNavStatePR();  // 6*1 vertex
    // here g2o vertex uses Twb(different in default VertexSE3 using EdgeSE3); edge/measurement formula input R&p(<-p+dp)
    pKF->UpdateNavStatePVRFromTcw();
    vns->setEstimate(pKF->GetNavState());
    vns->setId(pKF->mnId);
    vns->setFixed(pKF->mnId == 0);  // GBA fix id0 KF, same in localBA
    optimizer.addVertex(vns);

    if (pKF->mnId > maxKFid) maxKFid = pKF->mnId;
  }

  if (bEnc) {
    // Set Enc edges
    cv::Mat Tbe = Frame::mTbc * Frame::mTce;  // for Enc
    Quaterniond qRbe = Quaterniond(Converter::toMatrix3d(Tbe.rowRange(0, 3).colRange(0, 3)));
    Vector3d tbe = Converter::toVector3d(Tbe.rowRange(0, 3).col(3));  // for Enc

    for (size_t i = 0; i < vpKFs.size(); i++) {
      KeyFrame* pKF1 = vpKFs[i];  // KFj, store the Enc pre-integration between previous-current
      KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();  // Previous KF
      // if no KFi/EncPreInt's info, this EncPreInt edge cannot be added for lack of vertices i / edge ij
      if (!pKF0) continue;
      EncPreIntegrator encpreint = pKF1->GetEncPreInt();
      while (!pKF1->isBad() && pKF0->isBad()) { // to ensure encpreint is matched with pKF0
        pKF0 = pKF1->GetPrevKeyFrame();
        encpreint = pKF1->GetEncPreInt();
      }
      if (pKF1->isBad()) continue;
      if (encpreint.mdeltatij == 0) continue;
      // Enc edges
      int idKF0 = pKF0->mnId, idKF1 = pKF1->mnId;
      g2o::EdgeEncNavStatePR* eEnc = new g2o::EdgeEncNavStatePR();
      eEnc->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));  // Ti 0
      eEnc->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));  // Tj 1
      eEnc->setMeasurement(encpreint.mdelxEij);
      if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
        eEnc->setInformation(encpreint.mSigmaEij.inverse() * 1e-2);
      else
        eEnc->setInformation(encpreint.mSigmaEij.inverse());
      eEnc->qRbe = qRbe;
      eEnc->pbe = tbe;  // SetParams
      if (bRobust) {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eEnc->setRobustKernel(rk);
        rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
      }
      optimizer.addEdge(eEnc);
    }
  }

  const float thHuber2D = sqrt(5.99);   // chi2(0.05,2), sqrt(e'*Omega*e)<=delta, here unused
  const float thHuber3D = sqrt(7.815);  // chi2(0.05,3)
  Pinhole CamInst;
  bool binitcaminst = false;
  // Extrinsics
  Matrix3d Rcb = Frame::meigRcb;
  Vector3d tcb = Frame::meigtcb;

  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    MapPoint* pMP = vpMP[i];
    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
    const int id = pMP->mnId + maxKFid + 1;  // same as localBA
    vPoint->setId(id);
    vPoint->setMarginalized(true);  // P(xc,xp)=P(xc)*P(xp|xc), P(xc) is called marginalized/Schur elimination,
                                    // [B-E*C^(-1)*E.t()]*deltaXc=v-E*C^(-1)*w, H*deltaX=g=[v;w]; used in Sparse solver
    optimizer.addVertex(vPoint);

    const map<KeyFrame*, set<size_t>> observations = pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (map<KeyFrame*, set<size_t>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {
      KeyFrame* pKFi = mit->first;
      // pKF->mnId may > maxKFid for LocalMapping is recovered
      // only connect observation edges to optimized KFs/vertices
      if (pKFi->isBad() || pKFi->mnId > maxKFid) continue;

      bool usedistort = Frame::usedistort_ && pKFi->mpCameras.size();

      nEdges++;

      auto idxs = mit->second;
      for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
        auto idx = *iter;
        const cv::KeyPoint& kpUn = !usedistort ? pKFi->mvKeysUn[idx] : pKFi->mvKeys[idx];

        if (pKFi->mvuRight[idx] < 0)  // monocular MPs use 2*1 error
        {
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          g2o::EdgeReprojectPR * e = new g2o::EdgeReprojectPR();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));         // 0 is point
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));  // 1 is pose
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];  // 1/sigma^2
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);   // Omega=Sigma^(-1)=(here)=I/sigma^2

          if (bRobust)  // here false
          {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
          }
          if (!usedistort) {
            if (!binitcaminst) {
              CamInst.setParameter(pKFi->fx, 0);
              CamInst.setParameter(pKFi->fy, 1);
              CamInst.setParameter(pKFi->cx, 2);
              CamInst.setParameter(pKFi->cy, 3);
              binitcaminst = true;
            }
            e->SetParams(&CamInst, Rcb, tcb);
          } else {
            CV_Assert(pKFi->mapn2in_.size() > idx);
            e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb);
          }

          optimizer.addEdge(e);
        } else  // stereo MPs uses 3*1 error
        {
          Eigen::Matrix<double, 3, 1> obs;
          const float kp_ur = pKFi->mvuRight[idx];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeReprojectPRStereo * e = new g2o::EdgeReprojectPRStereo();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          if (bRobust) {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber3D);
          }
          if (!usedistort) {
            if (!binitcaminst) {
              CamInst.setParameter(pKFi->fx, 0);
              CamInst.setParameter(pKFi->fy, 1);
              CamInst.setParameter(pKFi->cx, 2);
              CamInst.setParameter(pKFi->cy, 3);
              binitcaminst = true;
            }
            e->SetParams(&CamInst, Rcb, tcb, &pKFi->mbf);
          } else {
            CV_Assert(pKFi->mapn2in_.size() > idx);
            e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb, &pKFi->mbf);
          }

          optimizer.addEdge(e);
        }
      }
    }

    if (nEdges == 0)  // if no edges/observations can be made, delete this Vertex from optimizer
    {
      optimizer.removeVertex(vPoint);  // here only do _vertices.erase(it(it->second==vPoint))
      vbNotIncludedMP[i] = true;
    } else {
      vbNotIncludedMP[i] = false;
    }
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(nIterations);  // 10 same as pure inliers iterations in localBA/motion-only BA/Sim3Motion-only BA,
                                    // maybe stopped by next CorrectLoop() in LoopClosing

  // Recover optimized data in a intermediate way

  // Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++)  // all KFs in mpMap
  {
    KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexNavStatePR* vns_recov = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(pKF->mnId));
    NavStated ns = vns_recov->estimate();
    // I think this impossible for mpCurrentKF in LoopClosing cannot be id0 KF but used for final
    // Full BA and initial GBA(Mono)
    if (nLoopKF == 0)
    {
      pKF->SetNavState(ns); // update posematrices of pFrame, mainly Twb/Tcw
    } else {
      pKF->mTcwGBA.create(4, 4, CV_32F);

      Sophus::SE3d Twb(ns.mRwb, ns.mpwb);
      Sophus::SE3d Tcb(Rcb, tcb);
      Sophus::SE3d Tcw = Tcb * Twb.inverse();
      g2o::SE3Quat SE3quat(Tcw.unit_quaternion(), Tcw.translation());

      Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
      pKF->mnBAGlobalForKF = nLoopKF;
    }
  }

  // Points
  for (size_t i = 0; i < vpMP.size(); i++)  // all MPs in mpMap
  {
    if (vbNotIncludedMP[i]) continue;
    // if this MP is optimized by g2o

    MapPoint* pMP = vpMP[i];

    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));

    if (nLoopKF == 0)  // I think it's impossible for normal flow but used for final Full BA and initial GBA(Mono)
    {
      pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA.create(3, 1, CV_32F);
      Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
}

int Optimizer::PoseOptimization(Frame* pFrame, Frame* pLastF) {
  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolver_6_3>(new g2o::BlockSolver_6_3(unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(
          new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>()))));
#else
  g2o::BlockSolver_6_3::LinearSolverType*
      linearSolver;  // 6*1 is Li algebra/se3, 3*1 is location of landmark, here only use unary edge of 6, 3 won't be
                     // optimized=>motion(6*1)-only BA

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();  // linearSolver<MatrixType> use dense solver

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);  // BlockSolver covers linearSolver

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // descending/optimization strategy is LM; this
                                                            // (BlockSolver*)_solver will be deleted in ~BaseClass()
#endif
  optimizer.setAlgorithm(solver);  // delete solver/_algorithm in ~SparseOptimizer()

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  g2o::VertexNavStatePR* vns = new g2o::VertexNavStatePR();  // 6*1 vertex
  // here g2o vertex uses Twb(different in default VertexSE3 using EdgeSE3); edge/measurement formula input R&p(<-p+dp)
  pFrame->UpdateNavStatePVRFromTcw();
  vns->setEstimate(pFrame->GetNavStateRef());
  vns->setId(0);
  vns->setFixed(false);
  optimizer.addVertex(vns);
  // Extrinsics
  Matrix3d Rcb = pFrame->meigRcb;
  Vector3d tcb = pFrame->meigtcb;

  if (pLastF != NULL && pFrame->GetEncPreInt().mdeltatij > 0 && !pLastF->GetTcwRef().empty()) {
    // Set LastFrame vertex
    g2o::VertexNavStatePR* vnslast = new g2o::VertexNavStatePR();
    pLastF->UpdateNavStatePVRFromTcw();
    vnslast->setEstimate(pLastF->GetNavStateRef());
    vnslast->setId(1);
    vnslast->setFixed(true);
    optimizer.addVertex(vnslast);
    // Set Enc edge(binary edge) between LastF-Frame
    const EncPreIntegrator& encpreint = pFrame->GetEncPreInt();
    g2o::EdgeEncNavStatePR* eEnc = new g2o::EdgeEncNavStatePR();
    eEnc->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));  // lastF,i
    eEnc->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));  // curF,j
    eEnc->setMeasurement(encpreint.mdelxEij);
    eEnc->setInformation(encpreint.mSigmaEij.inverse());
    cv::Mat Tbe = Frame::mTbc * Frame::mTce;  // for Enc
    Quaterniond qRbe = Quaterniond(Converter::toMatrix3d(Tbe.rowRange(0, 3).colRange(0, 3)));
    Vector3d tbe = Converter::toVector3d(Tbe.rowRange(0, 3).col(3));  // for Enc
    eEnc->qRbe = qRbe;
    eEnc->pbe = tbe;  // SetParams
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    eEnc->setRobustKernel(rk);
    rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
    optimizer.addEdge(eEnc);
  }

  // Set MapPoint vertices
  const int N = pFrame->N;

  vector<g2o::EdgeReprojectPR*> vpEdgesMono;  // 2*1(_measurement) binary edge<VertexSBAPointXYZ,VertexNavStatePR>
  vector<size_t> vnIndexEdgeMono;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);  // this can be optimized in RGBD mode

  vector<g2o::EdgeReprojectPRStereo*> vpEdgesStereo;  // 3*1(ul vl ur) binary edge
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);    // chi2(0.05,2)
  const float deltaStereo = sqrt(7.815);  // chi2 distribution chi2(0.05,3), the huber kernel delta

  Pinhole CamInst;
  bool usedistort = Frame::usedistort_ && pFrame->mpCameras.size();
  {
    if (!usedistort) {
      CamInst.setParameter(pFrame->fx, 0);
      CamInst.setParameter(pFrame->fy, 1);
      CamInst.setParameter(pFrame->cx, 2);
      CamInst.setParameter(pFrame->cy, 3);
    }

    unique_lock<mutex> lock(MapPoint::mGlobalMutex);  // forbid other threads to rectify pFrame->mvpMapPoints
    int id_mp_beg = 2;

    const auto &frame_mps = pFrame->GetMapPointMatches();
    for (int i = 0; i < N; i++) {
      MapPoint* pMP = frame_mps[i];
      if (pMP) {
        // add fixed mp vertices for motion_only BA
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();  //<3,Eigen::Vector3d>, for MPs' Xw
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = i + id_mp_beg;  //>=maxKFid+1
        vPoint->setId(id);
        vPoint->setFixed(true);
        optimizer.addVertex(vPoint);

        nInitialCorrespondences++;
        pFrame->mvbOutlier[i] = false;

        // Monocular observation
        if (pFrame->mvuRight[i] < 0)  // this may happen in RGBD case!
        {
          g2o::EdgeReprojectPR* e = new g2o::EdgeReprojectPR();
          if (!usedistort)
            e->SetParams(&CamInst, Rcb, tcb);
          else {
            CV_Assert(pFrame->mapn2in_.size() > i);
            e->SetParams(pFrame->mpCameras[get<0>(pFrame->mapn2in_[i])], Rcb, tcb);
          }

          // 0 Xw, VertexSBAPointXYZ* corresponding to pMP->mWorldPos
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          // 1 Tbw, VertexNavStatePR* corresponding to pFB->mNavState
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));

          // optimization target block=|e'*Omiga(or Sigma^(-1))*e|
          Eigen::Matrix<double, 2, 1> obs;
          const cv::KeyPoint& kpUn = !usedistort ? pFrame->mvKeysUn[i] : pFrame->mvKeys[i];
          obs << kpUn.pt.x, kpUn.pt.y;
          e->setMeasurement(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          // diagonal matrix means independece between x and y pixel noise 2*2 matrix
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          // optimization target=KernelHuber(block)=H(e)={1/2*e sqrt(e)<=delta;delta(sqrt(e)-1/2*delta) others}
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaMono);

          optimizer.addEdge(e);

          // record the edge recording feature index
          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        } else  // Stereo observation
        {
          g2o::EdgeReprojectPRStereo* e = new g2o::EdgeReprojectPRStereo();
          if (!usedistort)
            e->SetParams(&CamInst, Rcb, tcb, &pFrame->mbf);
          else {
            CV_Assert(pFrame->mapn2in_.size() > i);
            e->SetParams(pFrame->mpCameras[get<0>(pFrame->mapn2in_[i])], Rcb, tcb, &pFrame->mbf);
          }

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));

          // SET EDGE
          Eigen::Matrix<double, 3, 1> obs;
          const cv::KeyPoint& kpUn = !usedistort ? pFrame->mvKeysUn[i] : pFrame->mvKeys[i];
          const float& kp_ur = pFrame->mvuRight[i];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
          e->setMeasurement(obs);  // edge parameter/measurement formula output z
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }
      }
    }
  }

  if (nInitialCorrespondences < 3)  // at least P3P（well posed equation） EPnP(n>3) (overdetermined equation)
    return 0;

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815,
                               7.815};  // chi2(0.05,3), error_block limit(over will be outliers,here also lead to
                                        // turning point in RobustKernelHuber)
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  // 4 optimizations, each 10 steps, initial value is the same, but inliers are different
  for (size_t it = 0; it < 4; it++) {
    vns->setEstimate(pFrame->GetNavStateRef());
    // default edges' level is 0, so initially use all edges to optimize, after
    // it=0, just use inlier edges(_activeEdges) to optimize only call _activeEdges[k].computeError()
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)  // for 3D-monocular 2D matches, may entered in RGBD!
    {
      g2o::EdgeReprojectPR* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)  // for 3D-stereo 2D matches
    {
      g2o::EdgeReprojectPRStereo* e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx])  // at 1st time, all false for all edges is at level 0 or inliers(supposed),so
                                    // e._error is computed by g2o
      {
        e->computeError();
      }

      const float chi2 = e->chi2();  // chi2=e'*Omiga*e

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);  // adjust the outlier edges' level to 1
        nBad++;
      } else {
        e->setLevel(0);  // maybe adjust the outliers' level to inliers' level
        pFrame->mvbOutlier[idx] = false;
      }

      if (it == 2)
        e->setRobustKernel(
            0);  // let the final(it==3) optimization use no RobustKernel; this function will delete _robustkernel first
    }

    if (optimizer.edges().size() <
        10)  // it outliers+inliers(/_edges) number<10 only optimize once with RobustKernelHuber
      break;
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexNavStatePR* vns_recov = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(0));
  pFrame->GetNavStateRef() = vns_recov->estimate();  // update posematrices of pFrame, mainly Twb/Tcw
  pFrame->UpdatePoseFromNS();

  return nInitialCorrespondences - nBad;  // number of inliers
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int Nlocal) {
  KeyFrame* pKFlocal = NULL;  // for Nlocal
  // Local KeyFrames: First Breath Search from Current Keyframe
  list<KeyFrame*> lLocalKeyFrames;

  if (Nlocal == 0) {
    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    // insert all 1st layer Covisibility KFs into lLocalKeyFrames(not best n)
    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++) {
      KeyFrame* pKFi = vNeighKFs[i];
      pKFi->mnBALocalForKF = pKF->mnId;
      if (!pKFi->isBad())
        lLocalKeyFrames.push_back(pKFi);  // no covisible KFs(mvpOrderedConnectedKeyFrames) are duplicated or pKF itself
    }
  } else {
    // All KeyFrames in Local window are optimized, get N last KFs as Local Window
    int NlocalCnt = Nlocal;  // for assert
    pKFlocal = pKF;
    do {
      assert(pKFlocal && !pKFlocal->isBad() && "!pKFi. why??????");
      pKFlocal->mnBALocalForKF = pKF->mnId;  // avoid adding it into lFixedCameras
      lLocalKeyFrames.push_front(pKFlocal);  // notice the order is opposite
      pKFlocal = pKFlocal->GetPrevKeyFrame();
    } while (--NlocalCnt > 0 && pKFlocal != NULL);  // maybe less than N KFs in pMap
  }

  // Local MapPoints seen in Local KeyFrames
  list<MapPoint*> lLocalMapPoints;
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
    vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
    for (vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
      MapPoint* pMP = *vit;
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pKF->mnId)  // avoid duplications
          {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->mnId;
          }
    }
  }
  PRINT_INFO_MUTEX(blueSTR "Enter local BA..." << pKF->mnId << ", size of localKFs=" << lLocalKeyFrames.size()
                                               << ", mps=" << lLocalMapPoints.size() << whiteSTR << endl);

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes, 2nd layer fixed
  // neighbors(don't optimize them but contribute to the target min funcation)
  list<KeyFrame*> lFixedCameras;
  if (Nlocal > 0) {
    // Fixed Keyframes. Keyframes that see Local MapPoints and last (N+1)th KF but that are not Local Keyframes: \
      2nd layer fixed neighbors(don't optimize them but contribute to the target min funcation)
    // the last N+1th KF / Add the KeyFrame before local window.
    KeyFrame* pKFPrevLocal = pKFlocal;  // lLocalKeyFrames.front()->GetPrevKeyFrame();
    if (pKFPrevLocal) {
      assert(!pKFPrevLocal->isBad() && pKFPrevLocal->mnBALocalForKF != pKF->mnId && pKF->mnBAFixedForKF != pKF->mnId);
      pKFPrevLocal->mnBAFixedForKF = pKF->mnId;
      if (!pKFPrevLocal->isBad()) lFixedCameras.push_back(pKFPrevLocal);
    }  // else means mpMap->KeyFramesInMap()<N+1 / pKFPrevLocal point to nullptr
  }
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    auto observations = (*lit)->GetObservations();
    for (auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId)  // avoid duplications in lLocalKeyFrames && lFixedCameras
      {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (!pKFi->isBad()) lFixedCameras.push_back(pKFi);
      }
    }
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolver_6_3>(new g2o::BlockSolver_6_3(unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(
          new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()))));  // LM descending method
#else
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;  //<6,3> at least one type of BaseVertex<6/3,>

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();  // sparse Cholesky solver, similar to CSparse

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // LM descending method
#endif
  optimizer.setAlgorithm(solver);

  if (pbStopFlag)  // if &mbAbortBA !=nullptr, true in LocalMapping
    optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  size_t num_fixed_kf = 0;
  // Set Local KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexNavStatePR *vns = new g2o::VertexNavStatePR();
    pKFi->UpdateNavStatePVRFromTcw(); //TODO: delete this
    vns->setEstimate(pKFi->GetNavState());
    vns->setId(pKFi->mnId);
    if (pKFi->mnId == 0) ++num_fixed_kf;
    vns->setFixed(pKFi->mnId == 0); // only fix the vertex of initial KF(KF.mnId==0)
    optimizer.addVertex(vns);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
  }
  // Set Fixed KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexNavStatePR *vns = new g2o::VertexNavStatePR();
    pKFi->UpdateNavStatePVRFromTcw();
    vns->setEstimate(pKFi->GetNavState());
    vns->setId(pKFi->mnId);
    vns->setFixed(true);
    optimizer.addVertex(vns);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
    ++num_fixed_kf;
  }

  vector<g2o::EdgeEncNavStatePR*> vpEdgesEnc;  // Enc edges
  if (Nlocal > 0) {
    // Set Enc edges
    cv::Mat Tbe = Frame::mTbc * Frame::mTce;
    Quaterniond qRbe = Quaterniond(Converter::toMatrix3d(Tbe.rowRange(0, 3).colRange(0, 3)));
    Vector3d tbe = Converter::toVector3d(Tbe.rowRange(0, 3).col(3));

    for (list<KeyFrame*>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend;
         lit++) {
      KeyFrame* pKF1 = *lit;                     // Current KF, store the Enc pre-integration between previous-current
      KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();  // Previous KF
      EncPreIntegrator encpreint = pKF1->GetEncPreInt();
      // if no KFi/EncPreInt's info, this EncPreInt edge cannot be added for lack of vertices i / edge ij
      if (!pKF0 || encpreint.mdeltatij == 0) continue;
      CV_Assert(!pKF0->isBad());
      CV_Assert(!pKF1->isBad());
      // Enc edges
      int idKF0 = pKF0->mnId, idKF1 = pKF1->mnId;
      g2o::EdgeEncNavStatePR *eEnc = new g2o::EdgeEncNavStatePR();
      eEnc->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));  // Ti 0
      eEnc->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));  // Tj 1
      eEnc->setMeasurement(encpreint.mdelxEij);
      if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
        eEnc->setInformation(encpreint.mSigmaEij.inverse() * 1e-2);
      else
        eEnc->setInformation(encpreint.mSigmaEij.inverse());
      eEnc->qRbe = qRbe;
      eEnc->pbe = tbe;  // SetParams
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      eEnc->setRobustKernel(rk);
      rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
      optimizer.addEdge(eEnc);
      vpEdgesEnc.push_back(eEnc);  // for robust processing/ erroneous edges' culling
    }
  }

  // Set MapPoint vertices && MPs-KFs' edges
  const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();  // max edges'
                                                                                                       // size

  vector<g2o::EdgeReprojectPR*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  vector<g2o::EdgeReprojectPRStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono = sqrt(5.991); // sqrt(e_block)<=sqrt(chi2(0.05,2)) allow power 2 increasing(1/2*e_block)
  const float thHuberStereo = sqrt(7.815); // chi2(0.05,3)

  Pinhole CamInst;
  bool binitcaminst = false;
  Matrix3d Rcb = Frame::meigRcb;
  Vector3d tcb = Frame::meigtcb;
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    // Set MP vertices
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();  //<3,Eigen::Vector3d>, for MPs' Xw
    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
    int id = pMP->mnId + maxKFid + 1;  //>=maxKFid+1
    vPoint->setId(id);
    // P(xc,xp)=P(xc)*P(xp|xc), P(xc) is called marginalized/Schur elimination,
    // [B-E*C^(-1)*E.t()]*deltaXc=v-E*C^(-1)*w, H*deltaX=g=[v;w]; used in Sparse solver
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const auto observations = pMP->GetObservations();

    // Set edges
    for (map<KeyFrame*, set<size_t>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend;
         mit++) {
      KeyFrame* pKFi = mit->first;

      if (!pKFi->isBad())  // good pKFobserv then connect it with pMP by an edge
      {
        bool usedistort = Frame::usedistort_ && pKFi->mpCameras.size();
        auto idxs = mit->second;
        for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
          auto idx = *iter;
          const cv::KeyPoint& kpUn = !usedistort ? pKFi->mvKeysUn[idx] : pKFi->mvKeys[idx];

          // Monocular observation
          if (pKFi->mvuRight[idx] < 0) {
            g2o::EdgeReprojectPR* e = new g2o::EdgeReprojectPR();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);  // similar to ||e||

            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb);
            }

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(pKFi);       //_vertices[1]
            vpMapPointEdgeMono.push_back(pMP);  //_vertices[0]
          } else                                // Stereo observation
          {
            g2o::EdgeReprojectPRStereo* e = new g2o::EdgeReprojectPRStereo();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));

            Eigen::Matrix<double, 3, 1> obs;
            const float kp_ur = pKFi->mvuRight[idx];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
            e->setMeasurement(obs);
            const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberStereo);

            if (!usedistort) {
              if (!binitcaminst) {
                CamInst.setParameter(pKFi->fx, 0);
                CamInst.setParameter(pKFi->fy, 1);
                CamInst.setParameter(pKFi->cx, 2);
                CamInst.setParameter(pKFi->cy, 3);
                binitcaminst = true;
              }
              e->SetParams(&CamInst, Rcb, tcb, &pKFi->mbf);
            } else {
              CV_Assert(pKFi->mapn2in_.size() > idx);
              e->SetParams(pKFi->mpCameras[get<0>(pKFi->mapn2in_[idx])], Rcb, tcb, &pKFi->mbf);
            }

            optimizer.addEdge(e);
            vpEdgesStereo.push_back(e);
            vpEdgeKFStereo.push_back(pKFi);
            vpMapPointEdgeStereo.push_back(pMP);
          }
        }
      }
    }
  }

  if (pbStopFlag)     // true in LocalMapping
    if (*pbStopFlag)  // if mbAbortBA
      return;

  optimizer.initializeOptimization();
  optimizer.optimize(5);  // maybe stopped by *_forceStopFlag(mbAbortBA) in some step/iteration

  bool bDoMore = true;

  if (pbStopFlag)
    if (*pbStopFlag)  // judge mbAbortBA again
      bDoMore = false;

  if (bDoMore) {
    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeReprojectPR *e = vpEdgesMono[i];
      MapPoint* pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad())  // why this can be true?
        continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive())  // if chi2 error too big(5% wrong) or Zc<=0 then outlier
      {
        e->setLevel(1);
      }

      e->setRobustKernel(0);  // cancel RobustKernel
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
      g2o::EdgeReprojectPRStereo *e = vpEdgesStereo[i];
      MapPoint* pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 7.815 || !e->isDepthPositive())  // chi2(0.05,3)
      {
        e->setLevel(1);
      }

      e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);  // 10 steps same as motion-only BA
  }

  vector<pair<KeyFrame*, MapPoint*> > vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    g2o::EdgeReprojectPR *e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(make_pair(pKFi, pMP));  // ready to erase outliers of pKFi && pMP in monocular edges
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeReprojectPRStereo *e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(make_pair(pKFi, pMP));  // ready to erase outliers of pKFi && pMP in stereo edges
    }
  }

#ifndef NDEBUG
  {
    for (size_t i = 0, iend = vpEdgesEnc.size(); i < iend; i++) {
      g2o::EdgeEncNavStatePR* e = vpEdgesEnc[i];
      // if chi2 error too big(5% wrong)
      if (e->chi2() > 12.592) {
        PRINT_INFO_MUTEX( "Enc edge " << redSTR << i << whiteSTR << ", chi2 " << e->chi2() << ". ");
      }
    }
  }
#endif

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty())  // erase the relation between outliers of pKFi(matched mvpMapPoints) && pMP(mObservations)
  {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = vToErase[i].first;
      MapPoint* pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);  // here may erase pMP in mpMap
    }
  }

  // Recover optimized data

  // Keyframes update(Pose Tcw...)
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
    KeyFrame* pKF = *lit;
    g2o::VertexNavStatePR *vns = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(pKF->mnId));
    pKF->SetNavState(vns->estimate());
  }

  // Points update(Position, normal), no need to update descriptor for unchanging pMP->mObservations except for the ones
  // having outliers' edges?
  for (list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
    MapPoint* pMP = *lit;
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
    pMP->UpdateNormalAndDepth();
  }

  pMap->InformNewChange();  // zzh
}

void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose& CorrectedSim3,
                                       const map<KeyFrame*, set<KeyFrame*> >& LoopConnections,
                                       const bool& bFixScale)  // can be improved by adding IMU edges
{
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  // optimizer.setVerbose(false);//useless for initially _verbose(false)
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolver_7_3>(new g2o::BlockSolver_7_3(unique_ptr<g2o::BlockSolver_7_3::LinearSolverType>(
          new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>()))));  // LM descending method
#else
  g2o::BlockSolver_7_3::LinearSolverType* linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();  // sparse Cholesky solver
  g2o::BlockSolver_7_3* solver_ptr =
      new g2o::BlockSolver_7_3(linearSolver);  // 7 is sim3's dimension, 3 is position's dimension(unused here)
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // LM descending method
#endif

  // solver->_userLambdaInit->_value=0 at initial, which means 1e-5*max(H(j,j)) used in g2o
  // here use 1e-16 for initial faster Gauss-Newton (instead of Steepest Descent)
  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vScw(
      nMaxKFid + 1);  //+1 for id is from 0, Eigen::aligned_allocator is for Eigen::Quaterniond in g2o::Sim3, vScw used
                      // mainly for all MPs' correction
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid + 1);
  // vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);//unused here

  const int minFeat = 100;  // min loop edges' covisible MPs' number threshold

  // Set KeyFrame vertices
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)  // all KFs in pMap
  {
    KeyFrame* pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = pKF->mnId;

    LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

    if (it != CorrectedSim3.end())  // if pKF is found in CorrectedSim3
    {
      vScw[nIDi] = it->second;  // actually we can also use the same method as the following, but this way is faster
      VSim3->setEstimate(it->second);  // corrected g2oScw, here c means camera
    } else {
      Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
      Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
      g2o::Sim3 Siw(Rcw, tcw, 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);  // noncorrected g2oScw
    }

    if (pKF == pLoopKF)
      VSim3->setFixed(true);  // fix mpMatchedKF(in LoopClosing), this means id0 KF's Pose will be optimized here, but
                              // it will be only changed a little for mpMatchedKF's Pose is relatively accurate!
    // maybe fix mpMatchedKF instead of id0 KF for g2o optimization is start from fixed Pose, so if time is limited,
    // optimizing the close in time KFs first is better

    VSim3->setId(nIDi);
    // VSim3->setMarginalized(false);//useless for initially _marginalized(false)
    VSim3->_fix_scale = bFixScale;  // true/s=1 in VertexSE3Expmap for RGBD

    optimizer.addVertex(VSim3);

    // vpVertices[nIDi]=VSim3;
  }

  set<pair<long unsigned int, long unsigned int> > sInsertedEdges;

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7,
                    7>::Identity();  //information matrix/Omega/Sigma^(-1) uses default or use Euclidean distance \
    instead of Mahalonobis in error_block=e'e instead of e'Omega(!=I)e, so we also don't use RobustKernel/chi2/outliers/scale concept here, \
    so maybe we can use erroneous edges' concept in RGBDSLAM2 here!

  // Set Loop edges, these cannot be pure odom edges
  for (map<KeyFrame*, set<KeyFrame*> >::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end();
       mit != mend; mit++) {
    KeyFrame* pKF = mit->first;
    const long unsigned int nIDi = pKF->mnId;
    const set<KeyFrame*>& spConnections = mit->second;
    const g2o::Sim3 Siw = vScw[nIDi];     // not only faster than optimizer.vertex(nIDi)->estimate() but also used to
                                          // correct MPs in the end of this function, corrected Siw
    const g2o::Sim3 Swi = Siw.inverse();  // corrected Swi

    for (set<KeyFrame*>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++) {
      const long unsigned int nIDj = (*sit)->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat) continue;
      //if i is pCurKF && j is pLoopKF or pKF,*sit's covisible MPs' number>=100 here(notice 15 can have ordered covisible connections, \
            meaning Essential graph is a smaller(closer) part of Covisibility graph, here use stricter condition for it's new loop edges(validation in PoseGraphOpt.))

      const g2o::Sim3 Sjw = vScw[nIDj];  // noncorrected but relatively accurate Sjw
      const g2o::Sim3 Sji = Sjw * Swi;   // relatively accurate Sji

      g2o::EdgeSim3* e = new g2o::EdgeSim3();  // vertex 0/1 VertexSE3Expmap
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));  // 1 j
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));  // 0 i
      e->setMeasurement(
          Sji);  // S10/Sji~=Sjw*Siw^(-1), notice in default g2o lib g2o::EdgeSE3&&g2o::VertexSE3 use Tij&&Twi

      e->information() = matLambda;  // error_block=e'e

      optimizer.addEdge(e);

      sInsertedEdges.insert(
          make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));  // use min&&max to avoid duplications like (i,j)&&(j,i) for
                                                         // loop edges(in Essential/Pose Graph) are undirected edges
    }
  }

  Eigen::Matrix<double, 7, 7> matLambdaEnc = Eigen::Matrix<double, 7, 7>::Identity();  // added by zzh
  // 0 for phi, 1 for p, which is mainly caused by encoder measurement model instead of plane assumption
  // model(+kinematic model error+Tce calibration error)
  double dEncBase[2] = {1, 1};
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
    KeyFrame* pKF = vpKFs[i];
    KeyFrame* pParentKF = pKF->GetParent();
    if (pParentKF && pKF->GetWeight(pParentKF) < minFeat) {  // pure odom edge
      if (pKF->getState() != 2 && pKF->GetPrevKeyFrame() == pParentKF ||
          pParentKF->getState() != 2 &&
              pParentKF->GetPrevKeyFrame() == pKF) {  // pure Odom Edge, need to decrease information matrix!
        EncPreIntegrator encpreint;
        if (pKF->getState() != 2) {
          encpreint = pKF->GetEncPreInt();
        } else {
          encpreint = pParentKF->GetEncPreInt();
        }
        // notice matLambda((2,2)&(3,3)) used in other edges means the camera(+encoder) accuracy of (phi,p) which should
        // be close to encoder(here use 1), and different unit has different base!
        double dTmp=encpreint.mSigmaEij(2,2);
        if (dEncBase[0] > dTmp) {  // phi
          dEncBase[0] = dTmp;
        }
        // intuitively we choose sqrt(sigma2x^2+sigma2y^2) or p, makes (Sigma_p,0) to be (1,0) but (Sigma_p/2,
        // Sigma_p/2) to be near (1, 1)
        dTmp = encpreint.mSigmaEij.block<2,2>(3,3).norm();
        if (dEncBase[1] > dTmp) {
          dEncBase[1] = dTmp;
        }
      }
    }
  }

  // Set normal edges
  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)  // all KFs in pMap
  {
    KeyFrame* pKF = vpKFs[i];

    const int nIDi = pKF->mnId;

    g2o::Sim3 Swi;

    LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

    if (iti != NonCorrectedSim3.end())  // if pKF in found in NonCorrectedSim3/mvpCurrentConnectedKFs in LoopClosing
      Swi = (iti->second).inverse();    // noncorrected Swi
    else
      Swi = vScw[nIDi]
                .inverse();  // noncorrected Swi, vScw already records the pKF->Tcw of the keyframes not to be corrected

    KeyFrame* pParentKF = pKF->GetParent();

    // Spanning tree edge, like VO/visual odometry edges, these may be pure odom edges!
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Sjw;

      LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

      // noncorrected Swj
      if (itj != NonCorrectedSim3.end())
        Sjw = itj->second;
      else
        Sjw = vScw[nIDj];  // it may be pure odom edge!

      g2o::Sim3 Sji = Sjw * Swi;  // get Sji before CorrectLoop() in LoopClosing; noncorrected: Sji=Sjw*Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);

      e->information() = matLambda;               // I
      if (pKF->GetWeight(pParentKF) < minFeat) {  // pure odom edge
        if (pKF->getState() != 2 && pKF->GetPrevKeyFrame() == pParentKF ||
            pParentKF->getState() != 2 &&
                pParentKF->GetPrevKeyFrame() == pKF) {  // pure Odom Edge, need to decrease information matrix!
          EncPreIntegrator encpreint;
          if (pKF->GetPrevKeyFrame() == pParentKF) {
            encpreint = pKF->GetEncPreInt();
          } else {
            encpreint = pParentKF->GetEncPreInt();
          }
          for (int i = 0; i < 3; ++i) {
            if (dEncBase[0] == 0 && encpreint.mSigmaEij(i, i) == 0)
              matLambdaEnc(i, i) = 1;  // though this should never happen
            else
              matLambdaEnc(i, i) =
                  dEncBase[0] /
                  encpreint.mSigmaEij(i,
                                      i);  // this Information Matrix can help solve the dropping problem of our dataset
          }
          for (int i = 3; i < 6; ++i) {
            if (dEncBase[1] == 0 && encpreint.mSigmaEij(i, i) == 0)
              matLambdaEnc(i, i) = 1;  // though this should never happen
            else
              matLambdaEnc(i, i) =
                  dEncBase[1] /
                  encpreint.mSigmaEij(i,
                                      i);  // this Information Matrix can help solve the dropping problem of our dataset
          }
          e->information() = matLambdaEnc;
          PRINT_INFO_MUTEX( matLambdaEnc << endl);
        }
        PRINT_INFO_MUTEX( "Weight0/Odom link: " << pParentKF->mnId << " " << pKF->mnId << " " << pKF->GetWeight(pParentKF) << endl);
      }
      optimizer.addEdge(e);
    }

    // Loop edges, previous ones added by the order of el->vertex(0)->id() < el->vertex(1)->id()
    const set<KeyFrame*> sLoopEdges =
        pKF->GetLoopEdges();  //loop edges before this CorrectLoop()/previous mspLoopEdges of pKF, \
        notice only one(pCurKF-pLoopKF) of new loop edges will be mspLoopEdges, the reason why PoseGraphOpt. need lots of new loop edges for a better believe on this loop close and \
        optimize all KFs' Poses basing on this loop close
    for (set<KeyFrame*>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++) {
      KeyFrame* pLKF = *sit;
      if (pLKF->mnId < pKF->mnId)  // avoid repetitively adding the same loop edge/ add loop edges in an ordered way
      {
        g2o::Sim3 Slw;

        LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

        // noncorrected(in this loop correction) Slw
        if (itl != NonCorrectedSim3.end())
          Slw = itl->second;
        else
          Slw = vScw[pLKF->mnId];

        g2o::Sim3 Sli = Slw * Swi;  // previous/noncorrected Sli
        g2o::EdgeSim3* el = new g2o::EdgeSim3();
        el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        el->setMeasurement(Sli);
        el->information() = matLambda;  // I
        optimizer.addEdge(el);
      }
    }

    // Covisibility graph edges, a smaller(closer) part whose weight>=100, these cannot be pure odom edges
    const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
    for (vector<KeyFrame*>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++) {
      KeyFrame* pKFn = *vit;
      if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) &&
          !sLoopEdges.count(
              pKFn))  //don't need to judge !pKFn->mspLoopEdges.count(pKF) for previous loop edges are symmetric, \
            avoid duplications in optimizer._edges...;pKFn cannot be the parent/child of pKF(spanning tree edge) && previous loop edge of pKF
      {
        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId)  // good && avoid duplications by the order 1id<0id
        {
          if (sInsertedEdges.count(
                  make_pair(min(pKF->mnId, pKFn->mnId),
                            max(pKF->mnId, pKFn->mnId))))  // check if inserted as new loop edges before!
            continue;

          g2o::Sim3 Snw;  // Snormal_world, normal means a smaller part of Covisibility graph but not the loop/spanning
                          // tree edges

          LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

          // noncorrected Snw
          if (itn != NonCorrectedSim3.end())
            Snw = itn->second;
          else
            Snw = vScw[pKFn->mnId];

          g2o::Sim3 Sni = Snw * Swi;  // previous/noncorrected Sni

          g2o::EdgeSim3* en = new g2o::EdgeSim3();
          en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
          en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
          en->setMeasurement(Sni);
          en->information() = matLambda;  // I
          optimizer.addEdge(en);
        }
      }
    }
  }
  PRINT_INFO_MUTEX( "Spanning tree edge end!" << endl);

  // Optimize!
  optimizer.initializeOptimization();  // optimize all KFs' Pose Siw by new loop edges and normal edges
  optimizer.optimize(20);  // 2*10 steps, 10 is the pure inliers' iterations in localBA/motion-only BA/Sim3Motion-only
                           // BA

  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (size_t i = 0; i < vpKFs.size(); i++)  // all KFs in pMap
  {
    KeyFrame* pKFi = vpKFs[i];
    if (pKFi->isBad()) continue;

    const int nIDi = pKFi->mnId;

    g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3 CorrectedSiw = VSim3->estimate();    // optimized Siw
    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();  // used for latter points' correction
    Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = CorrectedSiw.translation();
    double s = CorrectedSiw.scale();

    eigt *= (1. / s);  //[R t/s;0 1]

    cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

    pKFi->SetPose(Tiw);  // update KF's Pose to PoseGraphOpt. optimized Tiw
    // Update P/V/R in NavState
    pKFi->UpdateNavStatePVRFromTcw();
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)  // all MPs in pMap
  {
    MapPoint* pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    int nIDr;
    if (pMP->mnCorrectedByKF == pCurKF->mnId)  // if this MP has already been correted by pCurKF/mpCurrentKF
    {
      nIDr = pMP->mnCorrectedReference;  // vScw[nIDr(here)] is corrected Scw for mvpCurrentConnectedKFs, corresponding
                                         // pMP's Pos is also corrected in CorrectLoop()
    } else                               // if this MP's Pos is noncorrected
    {
      KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
      nIDr = pRefKF->mnId;  // vScw[nIDr] should be noncorrected one!
    }

    g2o::Sim3 Srw = vScw[nIDr];  // corrected/noncorrected Scw but always can be used to calculate Pr=Scw*Pw
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];  // BA optimized result

    cv::Mat P3Dw = pMP->GetWorldPos();
    Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(
        eigP3Dw));  // optimized Pw=optimized Swr*Pr(noncorrected Srw*noncorrected Pw/corrected Srw*corrected Pw)

    cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
    pMP->SetWorldPos(cvCorrectedP3Dw);  // update MP's Pos to correted one through BA optimized Siw

    pMP->UpdateNormalAndDepth();  // update MP's normal for its position changed
  }
  // don't call pMap->InformNewChange(); for it's done outside
  PRINT_INFO_MUTEX( "PoseGraph end!" << endl);
}

int Optimizer::OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint*>& vpMatches1, g2o::Sim3& g2oS12,
                            const float th2, const bool bFixScale) {
  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolverX>(new g2o::BlockSolverX(unique_ptr<g2o::BlockSolverX::LinearSolverType>(
          new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>()))));  // LM descending method
#else
  g2o::BlockSolverX::LinearSolverType* linearSolver;  //<Eigen::Dynamic,Dynamic>

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();  // LinearSolver<MatrixType> uses a dense solver,
                                                                        // the same as PoseOptimization()

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // LM descending method
#endif
  optimizer.setAlgorithm(solver);

  // Camera poses
  const cv::Mat R1w = pKF1->GetRotation();
  const cv::Mat t1w = pKF1->GetTranslation();
  const cv::Mat R2w = pKF2->GetRotation();
  const cv::Mat t2w = pKF2->GetTranslation();

  // Set Sim3 vertex
  g2o::VertexNavStatePR* pvSE3;
  NavStated ns;
  ns.mRwb = Sophus::SO3exd(g2oS12.rotation()).inverse();
  ns.mpwb = -(ns.mRwb * g2oS12.translation());
  pvSE3 = new g2o::VertexNavStatePR();
  pvSE3->setEstimate(ns);
  pvSE3->setId(0);
  pvSE3->setFixed(false);
  optimizer.addVertex(pvSE3);
  g2o::VertexScale* pvScale = new g2o::VertexScale();
  pvScale->setEstimate(g2oS12.scale());
  pvScale->setId(1);
  if (bFixScale)
    pvScale->setFixed(true);
  else
    pvScale->setFixed(false);
  optimizer.addVertex(pvScale);

  // Set MapPoint vertices
  const int N = vpMatches1.size();
  const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
  vector<g2o::EdgeReprojectPRS*> vpEdges12;
  vector<g2o::EdgeReprojectPRSInv*> vpEdges21;
  vector<size_t> vnIndexEdge;

  vnIndexEdge.reserve(2 * N);
  vpEdges12.reserve(2 * N);
  vpEdges21.reserve(2 * N);

  const float deltaHuber = sqrt(th2);  // here sqrt(10), upper bound/supremum of ||e||

  int nCorrespondences = 0;

  bool usedistort = Frame::usedistort_ && pKF1->mpCameras.size() && pKF2->mpCameras.size();
  Pinhole CamInst;
  if (!usedistort) {
    CamInst.setParameter(pKF1->fx, 0);
    CamInst.setParameter(pKF1->fy, 1);
    CamInst.setParameter(pKF1->cx, 2);
    CamInst.setParameter(pKF1->cy, 3);
    CV_Assert(pKF2->fx == pKF1->fx);
    CV_Assert(pKF2->fy == pKF1->fy);
    CV_Assert(pKF2->cx == pKF1->cx);
    CV_Assert(pKF2->cy == pKF1->cy);
  }
  for (int i = 0; i < N; i++) {
    if (!vpMatches1[i]) continue;

    MapPoint* pMP1 = vpMapPoints1[i];
    MapPoint* pMP2 = vpMatches1[i];  // pMP1's matched MP

    // old +1;+2
    const int id1 = 2 * i + 2;  // different from PoseOptimization()
    const int id2 = 2 * i + 3;

    // TODO: check if needed to extend for 4 cams
    const auto idxs2 = pMP2->GetIndexInKeyFrame(pKF2);
    const int i2 = !idxs2.size() ? -1 : *idxs2.begin();

    if (pMP1 && pMP2)  // I think it's always true for the SBP/SBB method in LoopClosing, need test!
    {
      // I this it's always true for SBP/SBB method in LoopClosing, need test! if wrong, it may be due to
      // MapPointCulling() in LocalMapping
      if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0) {
        g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
        cv::Mat P3D1w = pMP1->GetWorldPos();
        cv::Mat P3D1c = R1w * P3D1w + t1w;
        vPoint1->setEstimate(Converter::toVector3d(P3D1c));
        vPoint1->setId(id1);
        // don't optimize the position of matched MapPoints(pKF1,pKF2), just contribute to the target function
        vPoint1->setFixed(true);
        optimizer.addVertex(vPoint1);

        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        cv::Mat P3D2w = pMP2->GetWorldPos();
        cv::Mat P3D2c = R2w * P3D2w + t2w;
        vPoint2->setEstimate(Converter::toVector3d(P3D2c));
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);
      } else
        continue;
    } else
      continue;

    nCorrespondences++;  // the number of matches

    // Set edge x1 = S12*X2
    Eigen::Matrix<double, 2, 1> obs1;
    const cv::KeyPoint& kpUn1 = !usedistort ? pKF1->mvKeysUn[i] : pKF1->mvKeys[i];
    obs1 << kpUn1.pt.x, kpUn1.pt.y;

    g2o::EdgeReprojectPRS* e12 = new g2o::EdgeReprojectPRS();
    e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));  // 0 c2X2
    e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));    // 1 S12
    e12->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
    e12->setMeasurement(obs1);
    const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
    e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);  // Omega/Sigma^(-1)
    if (!usedistort) {
      e12->SetParams(&CamInst);
    } else {
      CV_Assert(pKF1->mapn2in_.size() > i);
      e12->SetParams(pKF1->mpCameras[get<0>(pKF1->mapn2in_[i])]);
    }

    g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
    e12->setRobustKernel(rk1);
    rk1->setDelta(deltaHuber);  // here ||e||<=sqrt(10) use 1/2*e'Omega*e
    optimizer.addEdge(e12);     // edge_S12_Xc2

    // Set edge x2 = S21*X1
    Eigen::Matrix<double, 2, 1> obs2;
    const cv::KeyPoint& kpUn2 = !usedistort ? pKF2->mvKeysUn[i2] : pKF2->mvKeys[i2];
    obs2 << kpUn2.pt.x, kpUn2.pt.y;

    g2o::EdgeReprojectPRSInv* e21 = new g2o::EdgeReprojectPRSInv();
    e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));  // 0 c1X1
    e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));    // 1 S12&Inv
    e21->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
    e21->setMeasurement(obs2);
    float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
    e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);  // Omega
    if (!usedistort) {
      e21->SetParams(&CamInst);
    } else {
      CV_Assert(pKF2->mapn2in_.size() > i2);
      e21->SetParams(pKF2->mpCameras[get<0>(pKF2->mapn2in_[i2])]);
    }

    g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
    e21->setRobustKernel(rk2);
    rk2->setDelta(deltaHuber);
    optimizer.addEdge(e21);  // edge_S12/S21_Xc1

    vpEdges12.push_back(e12);
    vpEdges21.push_back(e21);
    vnIndexEdge.push_back(i);  // i in pKF1, matched one is vpMatches1[i]->GetIndexInKeyFrame(pKF2) in pKF2
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(5);  // same as LocalBundleAdjustment()

  // Check inliers
  int nBad = 0;
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    g2o::EdgeReprojectPRS* e12 = vpEdges12[i];
    g2o::EdgeReprojectPRSInv* e21 = vpEdges21[i];
    if (!e12 || !e21)  // I think this cannot be true, need test!
      continue;

    // about 99% inliers are right, chi2(0.01,2)=9.21, looser then localBA
    if (e12->chi2() > th2 || e21->chi2() > th2) {
      size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint*>(NULL);  // erase outlier matches immediately
      optimizer.removeEdge(e12);  // erase e12 from the HyperGraph::EdgeSet _edges && HyperGraph::Vertex::EdgeSet _edges
      optimizer.removeEdge(e21);
      vpEdges12[i] = nullptr;
      vpEdges21[i] = nullptr;
      nBad++;  // the number of outliers
    }
  }

  int nMoreIterations;
  if (nBad > 0)            // if any outlier is found
    nMoreIterations = 10;  // same as localBA/motion-only BA
  else
    nMoreIterations = 5;  // 5+5=10, always 10 iterations including only inliers

  // if the number of inliers <10 directly regard it's an unbelievable loop candidate match, half of the used threshold
  // in ComputeSim3() in LoopClosing, same as nGood<10 then continue threshold in Relocalization()
  if (nCorrespondences - nBad < 10) return 0;

  // Optimize again only with inliers
  optimizer.initializeOptimization();
  optimizer.optimize(nMoreIterations);

  int nIn = 0;  // the number of inliers
  for (size_t i = 0; i < vpEdges12.size(); i++) {
    g2o::EdgeReprojectPRS* e12 = vpEdges12[i];
    g2o::EdgeReprojectPRSInv* e21 = vpEdges21[i];
    // here maybe true for outliers checked before
    if (!e12 || !e21) continue;

    if (e12->chi2() > th2 || e21->chi2() > th2)  // if outliers
    {
      // size_t idx = vnIndexEdge[i];
      // erase outlier matches immediately, rectified by zzh, here vnIndexEdge[i]==i
      // if vpMatches1[i] is always two good MPs' match
      vpMatches1[vnIndexEdge[i]] = nullptr;
    } else
      nIn++;
  }

  // Recover optimized Sim3
  // get optimized S12
  g2o::VertexNavStatePR* pSim3_recov = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(0));
  g2o::VertexScale* pScale_recov = static_cast<g2o::VertexScale*>(optimizer.vertex(1));
  // rectify g2o: S12 to the optimized S12
  // g2oS12 = vSim3_recov->estimate();
  ns = pSim3_recov->estimate();
  auto R12 = ns.mRwb.inverse();
  g2oS12 = g2o::Sim3(R12.unit_quaternion(), -(R12 * ns.mpwb), pScale_recov->estimate());

  return nIn;
}

}  // namespace VIEO_SLAM
