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
#include "FrameBase_impl.h"

namespace VIEO_SLAM {  // changed a lot refering to the JingWang's code

using namespace Eigen;

void Optimizer::LocalBAPRVIDP(KeyFrame* pCurKF, int Nlocal, bool* pbStopFlag, Map* pMap, cv::Mat& gw) {}
void Optimizer::LocalBundleAdjustmentNavStatePRV(KeyFrame* pKF, int Nlocal, bool* pbStopFlag, Map* pMap, cv::Mat gw) {
  // Gravity vector in world frame
  Vector3d GravityVec = Converter::toVector3d(gw);

#define ORB3_STRATEGY
  int optit = 5;
#ifdef ORB3_STRATEGY
  const int maxFixKF = 200;  // limit fixed vertex size to ensure speed
  bool bLarge = false;
  if (bLarge) {
    Nlocal *= 2.5;
    optit = 4;
  } else {
    optit = 10;
  }
#endif

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
#ifdef ORB3_STRATEGY
      if (lFixedCameras.size() >= maxFixKF) break;
#endif
    }
  }
  PRINT_INFO_MUTEX(blueSTR "Enter local BA..." << pKF->mnId << ", size of localKFs=" << lLocalKeyFrames.size()
                                               << "fixedkfs = " << lFixedCameras.size()
                                               << ", mps=" << lLocalMapPoints.size() << whiteSTR << endl);

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
#ifdef ORB3_STRATEGY
  if (bLarge) {
    solver->setUserLambdaInit(1e-2);  // to avoid iterating for finding optimal lambda
  } else {
    solver->setUserLambdaInit(1e0);
  }
#endif
  optimizer.setAlgorithm(solver);

  //#ifndef ORB3_STRATEGY
  if (pbStopFlag)  // if &mbAbortBA !=nullptr, true in LocalMapping
    optimizer.setForceStopFlag(pbStopFlag);
  //#endif

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

    int idKF0 = 3 * pKF0->mnId, idKF1 = 3 * pKF1->mnId;
    if (imupreint.mdeltatij) {
      // IMU_I/PRV(B) edges
      g2o::EdgeNavStatePRV* eprv = new g2o::EdgeNavStatePRV();
      eprv->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));      // PRi 0
      eprv->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));      // PRj 1
      eprv->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 1)));  // Vi 2
      eprv->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 1)));  // Vj 3
      eprv->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 4
      eprv->setMeasurement(imupreint);
      Matrix9d InfoijPRV = imupreint.GetProcessedInfoijPRV();  // mSigmaijPRV.inverse();
#ifdef ORB3_STRATEGY
      bool bfixedkf = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed();
      bool bRecInit = false;  // true;//
      if (bfixedkf || bRecInit) {
        if (bfixedkf) {
          eprv->setInformation(InfoijPRV * 1e-2);
        } else
          eprv->setInformation(InfoijPRV);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eprv->setRobustKernel(rk);
        rk->setDelta(thHuberNavStatePRV);
      } else
        eprv->setInformation(InfoijPRV);
#else
      if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed()) {
        eprv->setInformation(InfoijPRV * 1e-2);
      } else
        eprv->setInformation(InfoijPRV);
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      eprv->setRobustKernel(rk);
      rk->setDelta(thHuberNavStatePRV);
#endif
      eprv->SetParams(GravityVec);
      optimizer.addEdge(eprv);
      vpEdgesNavStatePRV.push_back(eprv);  // for robust processing/ erroneous edges' culling
    }
    // IMU_RW/Bias edge
    g2o::EdgeNavStateBias* ebias = new g2o::EdgeNavStateBias();
    ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 0
    ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 2)));  // Bj 1
    ebias->setMeasurement(imupreint);
    double deltatij = imupreint.mdeltatij ? imupreint.mdeltatij : pKF1->mTimeStamp - pKF0->mTimeStamp;
#ifdef ORB3_STRATEGY
    ebias->setInformation(InvCovBgaRW / deltatij);
#else
    // see Manifold paper (47), notice here is Omega_d/Sigma_d.inverse()
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed()) {
      ebias->setInformation(InvCovBgaRW / deltatij * 1e-2);
    } else
      ebias->setInformation(InvCovBgaRW / deltatij);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    ebias->setRobustKernel(rk);
    rk->setDelta(thHuberNavStateBias);
#endif
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
#ifdef ORB3_STRATEGY
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
#else
    rk = new g2o::RobustKernelHuber;
#endif
    eEnc->setRobustKernel(rk);
    rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
    eEnc->qRbe = qRbe;
    eEnc->pbe = tbe;  // SetParams
    optimizer.addEdge(eEnc);
  }

  // Set MapPoint vertices && MPs-KFs' edges
  const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();  // max edges'
                                                                                                       // size

  typedef struct _BaseEdgeMono {
    g2o::EdgeReprojectPR* pedge;
    size_t idx;
  } BaseEdgeMono;
  vector<BaseEdgeMono> vpEdgesMono;
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

  const float chi2Mono = 5.991;
  const float thHuberMono = sqrt(chi2Mono);  // sqrt(e_block)<=sqrt(chi2(0.05,2)) allow power 2 increasing(1/2*e_block),
  const float thHuberStereo = sqrt(7.815);   // chi2(0.05,3)
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

#ifdef ORB3_STRATEGY
      if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) continue;
#endif

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
            vpEdgesMono.push_back(BaseEdgeMono());
            BaseEdgeMono& pbaseedgemono = vpEdgesMono.back();
            pbaseedgemono.pedge = e;
            pbaseedgemono.idx = idx;
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
  PRINT_INFO_MUTEX("factor_visual num=" << vpEdgesMono.size() << endl);

  //#ifndef ORB3_STRATEGY
  if (pbStopFlag)     // true in LocalMapping
    if (*pbStopFlag)  // if mbAbortBA
      return;
  //#endif

  optimizer.initializeOptimization();
#ifdef ORB3_STRATEGY
  //  optimizer.computeActiveErrors();
  //  float err = optimizer.activeRobustChi2();
  bool bDoMore = false;
#else
  bool bDoMore = true;
#endif
  optimizer.optimize(optit);  // maybe stopped by *_forceStopFlag(mbAbortBA) in some step/iteration

  if (pbStopFlag)
    if (*pbStopFlag)  // judge mbAbortBA again
      bDoMore = false;

  if (bDoMore) {
    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeReprojectPR* e = vpEdgesMono[i].pedge;
      MapPoint* pMP = vpMapPointEdgeMono[i];
      // ref from ORB3
      bool bClose = pMP->GetTrackInfoRef().track_depth_ < 10.f;

      if (pMP->isBad())  // why this can be true?
        continue;

      // if chi2 error too big(5% wrong) or Zc<=0 then outlier
      if (e->chi2() > (bClose ? 1.5 * chi2Mono : chi2Mono) || !e->isDepthPositive()) {
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

  vector<tuple<KeyFrame*, MapPoint*, size_t>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    const BaseEdgeMono& e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];
    // ref from ORB3
    bool bClose = pMP->GetTrackInfoRef().track_depth_ < 10.f;

    if (pMP->isBad()) continue;

    if (e.pedge->chi2() > (bClose ? 1.5 * chi2Mono : chi2Mono) || !e.pedge->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.emplace_back(pKFi, pMP, e.idx);  // ready to erase outliers of pKFi && pMP in monocular edges
    }
  }
  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeReprojectPRStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.emplace_back(pKFi, pMP, -1);  // ready to erase outliers of pKFi && pMP in stereo edges
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

  //#ifdef ORB3_STRATEGY
  //  float err_end = optimizer.activeRobustChi2();
  //  if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) {
  //    PRINT_DEBUG_INFO("FAIL LOCAL-INERTIAL BA!!!!" << endl, imu_tightly_debug_path, "localmapping_thread_debug.txt");
  //    return;
  //  }
  //#endif

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {  // erase the relation between outliers of pKFi(matched mvpMapPoints) && pMP(mObservations)
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = get<0>(vToErase[i]);
      MapPoint* pMPi = get<1>(vToErase[i]);
      auto idx = get<2>(vToErase[i]);

      // here may erase pMP in mpMap
      ErasePairObs(pKFi, pMPi, -1);  // idx);
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

int Optimizer::GlobalBundleAdjustmentNavStatePRV(Map* pMap, const cv::Mat& cvgw, int nIterations, bool* pbStopFlag,
                                                 const unsigned long nLoopKF, const bool bRobust, bool bScaleOpt,
                                                 IMUInitialization* pimu_initiator) {
  vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  vector<MapPoint*> vpMP = pMap->GetAllMapPoints();

  // Gravity vector in world frame
  Vector3d wg = Converter::toVector3d(cvgw);

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

  long unsigned int maxKFid = 0, id_scale, id_g, id_mp_beg;

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
    id_scale = maxKFid + 1;
    pvScale->setId(id_scale);
    pvScale->setFixed(false);
    optimizer.addVertex(pvScale);
  }
  // opt GDir
  Vector3d GI;
  bool init_prior = false;
  if (pimu_initiator) {
    g2o::VertexGThetaXYRwI* vG = new g2o::VertexGThetaXYRwI();
    GI = Vector3d(0, 0, 1) * wg.norm();
    vG->setToOriginImpl(wg);
    id_g = maxKFid + 2;
    vG->setId(id_g);
    vG->setFixed(false);
    optimizer.addVertex(vG);
    init_prior = true;
  }
  vector<bool> vadd_prior_bias;
  vector<FrameBase*> pvvnsbias_beg;
  if (init_prior) {
    for (vector<KeyFrame*>::const_iterator lit = vpKFs.begin(), lend = vpKFs.end(); lit != lend; lit++) {
      FrameBase* const& pFBi = *lit;
      if (pFBi->isBad()) continue;

      if (1) {  //! pFBi->only_PR_in_BA_
        int8_t id_cam = pFBi->id_cam_;
        if (vadd_prior_bias.size() <= id_cam) {
          vadd_prior_bias.resize(id_cam + 1, true);
        }
        if (0) {  // pFBi->fixed_vbgba_in_BA_
          vadd_prior_bias[id_cam] = false;
        }
        if (!vadd_prior_bias[id_cam]) continue;
        if (pvvnsbias_beg.size() <= id_cam) {
          pvvnsbias_beg.resize(id_cam + 1, nullptr);
        }
        if (!pvvnsbias_beg[id_cam] || pFBi->timestamp_ < pvvnsbias_beg[id_cam]->timestamp_) {
          pvvnsbias_beg[id_cam] = pFBi;
        }
      }
    }
    for (int i = 0; i < pvvnsbias_beg.size(); ++i) {
      auto pFBtmp = pvvnsbias_beg[i];
      assert(!pFBtmp || i == pFBtmp->id_cam_);
      if (!pFBtmp || !vadd_prior_bias[i]) continue;

      g2o::VertexNavStateBias* vPriorBias = new g2o::VertexNavStateBias();
      NavStated ns(pFBtmp->GetNavState());
      vPriorBias->setEstimate(ns);
      vPriorBias->setId(maxKFid + 3 + i);
      vPriorBias->setFixed(true);
      optimizer.addVertex(vPriorBias);
    }
    id_mp_beg = maxKFid + 3 + pvvnsbias_beg.size();
  } else {
    id_mp_beg = maxKFid + 3;
  }
  vector<double> sum_dt_tmp(pvvnsbias_beg.size(), 0);

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
    while (!pKF1->isBad() && pKF0->isBad()) {  // to ensure imupreint is matched with pKF0
      pKF0 = pKF1->GetPrevKeyFrame();
      imupreint = pKF1->GetIMUPreInt();
    }
    // don't add the bad KFs to optimizer
    if (pKF1->isBad()) continue;  // way0

    double deltatij = imupreint.mdeltatij ? imupreint.mdeltatij : pKF1->mTimeStamp - pKF0->mTimeStamp;
    size_t device = pKF1->id_cam_;
    if (sum_dt_tmp.size() && vadd_prior_bias[device]) {
      sum_dt_tmp[device] += deltatij;
      //      ++num_dt_tmp[device];
    }

    int idKF0 = 3 * pKF0->mnId, idKF1 = 3 * pKF1->mnId;
    if (imupreint.mdeltatij) {
      // IMU_I/PRV(B) edges
      using BaseEdgeIMUnb = g2o::BaseMultiEdgeEx<9, IMUPreintegrator>;
      BaseEdgeIMUnb* eprv = new g2o::EdgeNavStatePRV();
      if (pimu_initiator) {
        eprv = new g2o::EdgeNavStatePRVG();
        eprv->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id_g)));
        ((g2o::EdgeNavStatePRVG*)eprv)->SetParams(GI);
      } else {
        eprv = new g2o::EdgeNavStatePRV();
        ((g2o::EdgeNavStatePRV*)eprv)->SetParams(wg);
      }
      eprv->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0)));      // PRi 0
      eprv->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1)));      // PRj 1
      eprv->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 1)));  // Vi 2
      eprv->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 1)));  // Vj 3
      eprv->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 4
      eprv->setMeasurement(imupreint);
      Matrix9d InfoijPRV = imupreint.GetProcessedInfoijPRV();  // mSigmaijPRV.inverse();
      if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
        eprv->setInformation(InfoijPRV * 1e-2);
      else
        eprv->setInformation(InfoijPRV);
      if (bRobust) {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        eprv->setRobustKernel(rk);
        rk->setDelta(thHuberNavStatePRV);
      }  // here false
      optimizer.addEdge(eprv);
    }
    // else way1
    // IMU_RW/Bias edge
    g2o::EdgeNavStateBias* ebias = new g2o::EdgeNavStateBias();
    ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0 + 2)));  // Bi 0
    ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF1 + 2)));  // Bj 1
    ebias->setMeasurement(imupreint);
    if (dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idKF0))->fixed())
      ebias->setInformation(InvCovBgaRW / deltatij * 1e-2);
    else
      // see Manifold paper (47), notice here is Omega_d/Sigma_d.inverse()
      ebias->setInformation(InvCovBgaRW / deltatij);
    if (bRobust) {
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      ebias->setRobustKernel(rk);
      rk->setDelta(thHuberNavStateBias);
    }  // here false
    optimizer.addEdge(ebias);

    // Set Enc edge(binary edge) between LastF-Frame
    EncPreIntegrator encpreint = pKF1->GetEncPreInt();
    while (!pKF1->isBad() && pKF0->isBad()) {  // to ensure encpreint is matched with pKF0
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
  if (init_prior) {
    const double coeff_deltat_prior = 1, coeff_deltat_priorg = 1;
    for (int i = 0; i < pvvnsbias_beg.size(); ++i) {
      auto pFBtmp = pvvnsbias_beg[i];
      if (pFBtmp && i != pFBtmp->id_cam_) {
        PRINT_DEBUG_INFO_MUTEX(
            redSTR << "Wrong pFBtmp: i,idcam=" << i << "," << pFBtmp->id_cam_ << ", check!" << whiteSTR << endl,
            imu_tightly_debug_path, "common_thread_debug.txt");
      }
      assert(!pFBtmp || i == pFBtmp->id_cam_);
      if (!pFBtmp || !vadd_prior_bias[i] || pFBtmp->isBad()) continue;

      g2o::EdgeNavStateBias* epb = new g2o::EdgeNavStateBias();
      epb->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(maxKFid + 3 + i)));
      int idFB = pFBtmp->mnId;  // mapfb2i[pFBtmp];
      epb->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idFB + 2)));
      Matrix6d InfoBias;
      InfoBias.setZero();
      double dtmp = sum_dt_tmp[i] * coeff_deltat_priorg;
      size_t device = pFBtmp->id_cam_;
      // assert(pFBtmp->GetIMUPreInt().device_ == device);
      InfoBias.block<3, 3>(0, 0) = IMUData::mInvSigmabg2 /*[device]*/ / dtmp * Matrix3d::Identity();
      dtmp = sum_dt_tmp[i] * coeff_deltat_prior;
      InfoBias.block<3, 3>(3, 3) = IMUData::mInvSigmaba2 /*[device]*/ / dtmp * Matrix3d::Identity();
      epb->setInformation(InfoBias);

      //      if (kRobustFull == robust) {
      //        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      //        epb->setRobustKernel(rk);
      //        rk->setDelta(sqrt_chi2_sig5_6);
      //      }

      optimizer.addEdge(epb);
    }
  }

  const float thHuber2D = sqrt(5.99);   // chi2(0.05,2), sqrt(e'*Omega*e)<=delta, here unused
  const float thHuber3D = sqrt(7.815);  // chi2(0.05,3)
  Pinhole CamInst;
  bool binitcaminst = false;
  // Extrinsics
  Matrix3d Rcb = Frame::meigRcb;
  Vector3d tcb = Frame::meigtcb;

  int nInitialCorrespondences = 0;
  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    MapPoint* pMP = vpMP[i];
    if (pMP->isBad()) continue;
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
    const int id = pMP->mnId + id_mp_beg;  // same as localBA
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

            if (bRobust) {  // here false
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
            e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id_scale)));
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
            g2o::EdgeReprojectPRSStereo* e = new g2o::EdgeReprojectPRSStereo();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId * 3)));
            e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id_scale)));
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

        ++nInitialCorrespondences;
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
  if (pimu_initiator) {
    g2o::VertexGThetaXYRwI* vG = static_cast<g2o::VertexGThetaXYRwI*>(optimizer.vertex(id_g));
    bool verbose = true;
    if (verbose) cout << "before gw=" << wg.transpose() << endl;
    pimu_initiator->SetGravityVec(Converter::toCvMat(vG->estimate() * GI));
    if (verbose) {
      cout << "after gw=" << pimu_initiator->GetGravityVec().t() << endl;
    }
  }
  // Scale for Xw
  double scale = 1.;
  if (bScaleOpt) {
    scale = static_cast<g2o::VertexScale*>(optimizer.vertex(id_scale))->estimate();
    PRINT_INFO_MUTEX(azureSTR "Recovered scale is " << scale << whiteSTR << endl);
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
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + id_mp_beg));

    if (!bScaleOpt) {
      // it's for initial/final Full BA
      if (nLoopKF == 0) {
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

  if (pimu_initiator) pMap->InformNewChange();

  return nInitialCorrespondences;
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
  PRINT_INFO_MUTEX(redSTR << "Enter GBA" << whiteSTR << endl);
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
    // here g2o vertex uses Twb(different in default VertexSE3 using EdgeSE3); edge/measurement formula input
    // R&p(<-p+dp)
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
      KeyFrame* pKF1 = vpKFs[i];                 // KFj, store the Enc pre-integration between previous-current
      KeyFrame* pKF0 = pKF1->GetPrevKeyFrame();  // Previous KF
      // if no KFi/EncPreInt's info, this EncPreInt edge cannot be added for lack of vertices i / edge ij
      if (!pKF0) continue;
      EncPreIntegrator encpreint = pKF1->GetEncPreInt();
      while (!pKF1->isBad() && pKF0->isBad()) {  // to ensure encpreint is matched with pKF0
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

          g2o::EdgeReprojectPR* e = new g2o::EdgeReprojectPR();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));          // 0 is point
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));  // 1 is pose
          e->setMeasurement(obs);
          const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];  // 1/sigma^2
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);    // Omega=Sigma^(-1)=(here)=I/sigma^2

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

          g2o::EdgeReprojectPRStereo* e = new g2o::EdgeReprojectPRStereo();
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
    if (nLoopKF == 0) {
      pKF->SetNavState(ns);  // update posematrices of pFrame, mainly Twb/Tcw
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

  if (pLastF != NULL && pFrame->GetEncPreInt().mdeltatij && !pLastF->GetTcwRef().empty()) {
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

    const auto& frame_mps = pFrame->GetMapPointMatches();
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

        ++nInitialCorrespondences;
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

Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, const int& start, const int& end) {
  // Goal
  // a  | ab | ac       a*  | 0 | ac*
  // ba | b  | bc  -->  0   | 0 | 0
  // ca | cb | c        ca* | 0 | c*

  // Size of block before block to marginalize
  const int a = start;
  // Size of block to marginalize
  const int b = end - start + 1;
  // Size of block after block to marginalize
  const int c = H.cols() - (end + 1);

  // Reorder as follows:
  // a  | ab | ac       a  | ac | ab
  // ba | b  | bc  -->  ca | c  | cb
  // ca | cb | c        ba | bc | b

  Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
    Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
    Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
  }
  if (a > 0 && c > 0) {
    Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
    Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
  }
  if (c > 0) {
    Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
    Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
    Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
  }
  Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

  // Perform marginalization (Schur complement)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
  for (int i = 0; i < b; ++i) {
    if (singularValues_inv(i) > 1e-6)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else
      singularValues_inv(i) = 0;
  }
  Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
  Hn.block(0, 0, a + c, a + c) =
      Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
  Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
  Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
  Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

  // Inverse reorder
  // a*  | ac* | 0       a*  | 0 | ac*
  // ca* | c*  | 0  -->  0   | 0 | 0
  // 0   | 0   | 0       ca* | 0 | c*
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
    res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
    res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
  }
  if (a > 0 && c > 0) {
    res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
    res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
  }
  if (c > 0) {
    res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
    res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
    res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
  }

  res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

  return res;
}
int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame* pFrame, KeyFrame* pLastF, const cv::Mat& gw,
                                                    const bool bComputeMarg) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setVerbose(false);
  optimizer.setAlgorithm(solver);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Frame vertex
  auto& nsj = pFrame->GetNavStateRef();
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  g2o::VertexGyrBias* VG = new g2o::VertexGyrBias();
  VG->setEstimate(nsj.mbg + nsj.mdbg);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  g2o::VertexGyrBias* VA = new g2o::VertexGyrBias();
  VA->setEstimate(nsj.mba + nsj.mdba);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  vector<size_t> vnIndexEdgeMono;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);

  const float thHuberMono = sqrt(5.991);
  const float thHuberStereo = sqrt(7.815);

  const auto& frame_mps = pFrame->GetMapPointsRef();  // GetMapPointMatches();
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = frame_mps[i];
      if (pMP) {
        cv::KeyPoint kpUn;

        // Left monocular observation
        if (1) {
          kpUn = pFrame->mvKeys[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto cami = get<0>(pFrame->mapn2in_[i]);
          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), cami);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = 1;  // pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
          e->computeError();
          // PRINT_DEBUG_INFO(cami << "e chi2=" << e->chi2() << " ", imu_tightly_debug_path,
          // "tracking_thread_debug.txt");
        }
      }
    }
  }
  // PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "tracking_thread_debug.txt");
  nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  KeyFrame* pKF = pLastF;
  auto nsi = pKF->GetNavState();
  VertexPose* VPk = new VertexPose(pKF);
  VPk->setId(4);
  VPk->setFixed(true);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pKF);
  VVk->setId(5);
  VVk->setFixed(true);
  optimizer.addVertex(VVk);
  g2o::VertexGyrBias* VGk = new g2o::VertexGyrBias();
  VGk->setEstimate(nsi.mbg + nsi.mdbg);
  VGk->setId(6);
  VGk->setFixed(true);
  optimizer.addVertex(VGk);
  g2o::VertexGyrBias* VAk = new g2o::VertexGyrBias();
  VAk->setEstimate(nsi.mba + nsi.mdba);
  VAk->setId(7);
  VAk->setFixed(true);
  optimizer.addVertex(VAk);

  IMUPreintegrator imupreint = pFrame->GetIMUPreInt();
  //  cout << fixed<<setprecision(9);
  //  cout << "check SigmaPRV raw=" << pFrame->GetIMUPreInt().mSigmaij << endl;
  EdgeInertial* ei = new EdgeInertial(&imupreint, Converter::toVector3d(gw), nsi.mbg, nsi.mba);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
  InvCovBgaRW.topLeftCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmabg2;  // Gyroscope bias random walk, covariance INVERSE
  InvCovBgaRW.bottomRightCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmaba2;  // Accelerometer bias random walk, covariance INVERSE
  // pFrame->mpImuPreintegratedFrame->C.rowRange(9, 12).colRange(9, 12).inv(cv::DECOMP_SVD);
  Eigen::Matrix3d InfoG = InvCovBgaRW.topLeftCorner(3, 3) / imupreint.mdeltatij;
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = InvCovBgaRW.bottomRightCorner(3, 3) / imupreint.mdeltatij;
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
  //  float chi2Stereo[4]={15.6,9.8,7.815,7.815};

  int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  bool bOut = false;
  for (size_t it = 0; it < 4; it++) {
    // PRINT_DEBUG_INFO(it << ":" << endl, imu_tightly_debug_path, "tracking_thread_debug.txt");
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    // For monocular observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (1 || pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      bool bClose = frame_mps[idx]->GetTrackInfoRef().track_depth_ < 10.f;

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        // PRINT_DEBUG_INFO("e chi2=" << chi2 << " ", imu_tightly_debug_path, "tracking_thread_debug.txt");
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    // PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "tracking_thread_debug.txt");
    if (optimizer.edges().size() < 10) {
      cout << "PIOLKF: NOT ENOUGH EDGES" << endl;
      break;
    }
  }

  // If not too much tracks, recover not too bad points
  if ((nInliers < 30)) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  // Recover optimized pose, velocity and biases
  nsj.mRwb = VP->estimate().Rwb;
  nsj.mpwb = VP->estimate().twb;
  nsj.mvwb = VV->estimate();
  nsj.mdbg = VG->estimate() - nsj.mbg;
  nsj.mdba = VA->estimate() - nsj.mba;
  pFrame->UpdatePoseFromNS();

  // Recover Hessian, marginalize keyFframe states and generate new prior for frame
  Eigen::Matrix<double, 15, 15> H;
  H.setZero();

  H.block<9, 9>(0, 0) += ei->GetHessian2();
  H.block<3, 3>(9, 9) += egr->GetHessian2();
  H.block<3, 3>(12, 12) += ear->GetHessian2();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  pFrame->mpcpi =
      new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(), VA->estimate(), H);

  return nInitialCorrespondences - nBad;
}
int Optimizer::PoseInertialOptimizationLastFrame(Frame* pFrame, Frame* pLastF, const cv::Mat& gw,
                                                 const bool bComputeMarg) {
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences = 0;

  // Set Current Frame vertex
  auto& nsj = pFrame->GetNavStateRef();
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  g2o::VertexGyrBias* VG = new g2o::VertexGyrBias();
  VG->setEstimate(nsj.mbg + nsj.mdbg);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  g2o::VertexGyrBias* VA = new g2o::VertexGyrBias();
  VA->setEstimate(nsj.mba + nsj.mdba);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int N = pFrame->N;

  vector<EdgeMonoOnlyPose*> vpEdgesMono;
  // vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeMono;
  // vector<size_t> vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  // vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  // vnIndexEdgeStereo.reserve(N);

  const float thHuberMono = sqrt(5.991);
  // const float thHuberStereo = sqrt(7.815);

  const auto& frame_mps = pFrame->GetMapPointsRef();  // GetMapPointMatches();
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = frame_mps[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if (1) {
          kpUn = pFrame->mvKeys[i];

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          auto cami = get<0>(pFrame->mapn2in_[i]);
          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), cami);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = 1;  // pFrame->mpCameras[cami]->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
          e->computeError();
          // PRINT_DEBUG_INFO(get<0>(pFrame->mapn2in_[i]) << "e chi2=" << e->chi2() << " ", imu_tightly_debug_path,
          // "tracking_thread_debug.txt");
        }
      }
    }
  }
  // PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "tracking_thread_debug.txt");

  nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // Set Previous Frame Vertex
  Frame* pFp = pLastF;
  auto& nsi = pLastF->GetNavStateRef();

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  bool blastfix = false;
  VPk->setFixed(blastfix);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(blastfix);
  optimizer.addVertex(VVk);
  g2o::VertexGyrBias* VGk = new g2o::VertexGyrBias();
  VGk->setEstimate(nsi.mbg + nsi.mdbg);
  VGk->setId(6);
  VGk->setFixed(blastfix);
  optimizer.addVertex(VGk);
  g2o::VertexGyrBias* VAk = new g2o::VertexGyrBias();
  VAk->setEstimate(nsi.mba + nsi.mdba);
  VAk->setId(7);
  VAk->setFixed(blastfix);
  optimizer.addVertex(VAk);

  IMUPreintegrator imupreint = pFrame->GetIMUPreInt();
  EdgeInertial* ei = new EdgeInertial(&imupreint, Converter::toVector3d(gw), nsi.mbg, nsi.mba);

  CV_Assert(VP == optimizer.vertex(0));
  CV_Assert(VV == optimizer.vertex(1));
  CV_Assert(VPk == optimizer.vertex(4));
  CV_Assert(VVk == optimizer.vertex(5));
  CV_Assert(VGk == optimizer.vertex(6));
  CV_Assert(VAk == optimizer.vertex(7));
  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  //  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  //  ei->setRobustKernel(rk);
  //  rk->setDelta(sqrt(16.919));

  CV_Assert(VG == optimizer.vertex(2));
  CV_Assert(VA == optimizer.vertex(3));
  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
  InvCovBgaRW.topLeftCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmabg2;  // Gyroscope bias random walk, covariance INVERSE
  InvCovBgaRW.bottomRightCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmaba2;  // Accelerometer bias random walk, covariance INVERSE
  // pFrame->mpImuPreintegratedFrame->C.rowRange(9, 12).colRange(9, 12).inv(cv::DECOMP_SVD);
  Eigen::Matrix3d InfoG = InvCovBgaRW.topLeftCorner(3, 3) / imupreint.mdeltatij;
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA = InvCovBgaRW.bottomRightCorner(3, 3) / imupreint.mdeltatij;
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  CV_Assert(pFp->mpcpi);
  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.

  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  // const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadMono = 0;
  int nBadStereo = 0;
  int nInliersMono = 0;
  int nInliersStereo = 0;
  int nInliers = 0;
  for (size_t it = 0; it < 4; it++) {
    // PRINT_DEBUG_INFO(it << ":" << endl, imu_tightly_debug_path, "tracking_thread_debug.txt");
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    nBadMono = 0;
    nBadStereo = 0;
    nInliers = 0;
    nInliersMono = 0;
    nInliersStereo = 0;
    float chi2close = 1.5 * chi2Mono[it];

    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      bool bClose = frame_mps[idx]->GetTrackInfoRef().track_depth_ < 10.f;

      if (1 || pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        // PRINT_DEBUG_INFO("e chi2=" << chi2 << " ", imu_tightly_debug_path, "tracking_thread_debug.txt");
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad = nBadMono + nBadStereo;

    // PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "tracking_thread_debug.txt");
    if (optimizer.edges().size() < 10) {
      cout << "PIOLF: NOT ENOUGH EDGES" << endl;
      break;
    }
  }

  if ((nInliers < 30)) {
    nBad = 0;
    const float chi2MonoOut = 18.f;
    // const float chi2StereoOut = 24.f;
    EdgeMonoOnlyPose* e1;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut)
        pFrame->mvbOutlier[idx] = false;
      else
        nBad++;
    }
  }

  nInliers = nInliersMono + nInliersStereo;

  // Recover optimized pose, velocity and biases
  nsj.mRwb = VP->estimate().Rwb;
  nsj.mpwb = VP->estimate().twb;
  nsj.mvwb = VV->estimate();
  nsj.mdbg = VG->estimate() - nsj.mbg;
  nsj.mdba = VA->estimate() - nsj.mba;
  pFrame->UpdatePoseFromNS();

  // Recover Hessian, marginalize previous frame states and generate new prior for frame
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += ei->GetHessian();

  Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += ep->GetHessian();

  int tot_in = 0, tot_out = 0;
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else
      tot_out++;
  }

  H = Marginalize(H, 0, 14);

  pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(),
                                        VA->estimate(), H.block<15, 15>(15, 15));
  delete pFp->mpcpi;
  pFp->mpcpi = NULL;

  return nInitialCorrespondences - nBad;
}
int Optimizer::PoseInertialOptimizationLastFrame2(Frame* pFrame, Frame* pLastKF, const cv::Mat& gw,
                                                  const bool bComputeMarg) {
  // automatically judge if fix lastF/KF(always fixed)
  //  bool bFixedLast = true;
  //  if (pLastKF->mbPrior) ;
  bool bFixedLast = false;
  // Extrinsics
  Matrix3d Rcb = pFrame->meigRcb;
  Vector3d tcb = pFrame->meigtcb;
  // Gravity vector in world frame
  Vector3d GravityVec = Converter::toVector3d(gw);

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType*
      linearSolver;  // 9*1 is Log(R),t,v/P/pvR, 6*1 bgi,bai/bi/Bias, (3*1 is location of landmark,) 3 types of
  // vertices so using BlockSolverX, though here 9_6 is also OK for unary edge in BA

  // linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  // sparse Cholesky factorization.
  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  //      new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();  // linear equation solver changed by
  //      JingWang

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  // descending/optimization strategy is still LM
  // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);//try this!
  optimizer.setAlgorithm(solver);

  // Set Frame & fixed KeyFrame's vertices, see VIORBSLAM paper (4)~(8)
  const int FramePVRId = 0, FrameBiasId = 1, LastKFPVRId = 2, LastKFBiasId = 3;
  NavState& nsj = pFrame->GetNavStateRef();
  // Set Frame vertex PVR/Bias
  g2o::VertexNavStatePVR* vNSFPVR = new g2o::VertexNavStatePVR();
  vNSFPVR->setEstimate(nsj);
  vNSFPVR->setId(FramePVRId);
  vNSFPVR->setFixed(false);
  optimizer.addVertex(vNSFPVR);
  g2o::VertexNavStateBias* vNSFBias = new g2o::VertexNavStateBias();
  vNSFBias->setEstimate(nsj);
  vNSFBias->setId(FrameBiasId);
  vNSFBias->setFixed(false);
  optimizer.addVertex(vNSFBias);
  // Set KeyFrame vertex PVR/Bias
  g2o::VertexNavStatePVR* vNSKFPVR = new g2o::VertexNavStatePVR();
  vNSKFPVR->setEstimate(pLastKF->GetNavState());
  vNSKFPVR->setId(LastKFPVRId);
  vNSKFPVR->setFixed(bFixedLast);
  optimizer.addVertex(vNSKFPVR);
  g2o::VertexNavStateBias* vNSKFBias = new g2o::VertexNavStateBias();
  vNSKFBias->setEstimate(pLastKF->GetNavState());
  vNSKFBias->setId(LastKFBiasId);
  vNSKFBias->setFixed(bFixedLast);
  optimizer.addVertex(vNSKFBias);

  // Set IMU_I/PVR(B) edge(ternary/multi edge) between LastKF-Frame
  const IMUPreintegrator& imupreint = pFrame->GetIMUPreInt();
  g2o::EdgeNavStatePVR* eNSPVR = new g2o::EdgeNavStatePVR();
  eNSPVR->setVertex(
      0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFPVRId)));  // PVRi, i is keyframe's id
  eNSPVR->setVertex(
      1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));  // PVRj, j here is frame's id
  eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));  // bi
  eNSPVR->setMeasurement(imupreint);                 // set delta~PVRij/delta~pij,delta~vij,delta~Rij
  Matrix9d Infoij = imupreint.GetProcessedInfoij();  // mSigmaij.inverse();
  eNSPVR->setInformation(Infoij);
  eNSPVR->SetParams(GravityVec);
  //  g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
  //  eNSPVR->setRobustKernel(rk);
  //  rk->setDelta(sqrt(16.919));  // chi2(0.05/0.01,9), 16.919/21.666 for 0.95/0.99 9DoF, but JingWang uses 100*21.666
  optimizer.addEdge(eNSPVR);
  // Set IMU_RW/Bias edge(binary edge) between LastKF-Frame
  g2o::EdgeNavStateBias* eNSBias = new g2o::EdgeNavStateBias();
  eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));  // bi
  eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameBiasId)));   // bj
  eNSBias->setMeasurement(imupreint);
  Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
  InvCovBgaRW.topLeftCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmabg2;  // Gyroscope bias random walk, covariance INVERSE
  InvCovBgaRW.bottomRightCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmaba2;  // Accelerometer bias random walk, covariance INVERSE
  // see Manifold paper (47), notice here is Omega_d/Sigma_d.inverse()
  eNSBias->setInformation(InvCovBgaRW / imupreint.mdeltatij);
  //  rk = new g2o::RobustKernelHuber;
  //  eNSBias->setRobustKernel(rk);
  //  rk->setDelta(sqrt(12.592));  // chi2(0.05/0.01,6), 12.592/16.812 for 0.95/0.99 6DoF, but JW uses 16.812
  optimizer.addEdge(eNSBias);
  // Set Prior edge(binary edge) for Last Frame, from mMargCovInv
  g2o::EdgeNavStatePriorPVRBias* eNSPrior = NULL;
  if (!bFixedLast) {
    eNSPrior = new g2o::EdgeNavStatePriorPVRBias();
    eNSPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFPVRId)));
    eNSPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));
    eNSPrior->setMeasurement(pLastKF->mNavStatePrior);
    Matrix15d H = pLastKF->mMargCovInv;
    H = (H + H) / 2;  // TODO(zzh):check t()
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
    Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 15; i++)
      if (eigs[i] < 1e-12) eigs[i] = 0;
    H = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    eNSPrior->setInformation(H);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    eNSPrior->setRobustKernel(rk);
    rk->setDelta(sqrt(25));  // thHuberNavState:chi2(0.05,15)=25 or chi2(0.01,15)=30.5779
    optimizer.addEdge(eNSPrior);
  }
  // Set Enc edge(binary) between LastKF-Frame
  g2o::EdgeEncNavStatePVR* eEnc = nullptr;
  if (pFrame->GetEncPreInt().mdeltatij > 0) {
    // Set Enc edge(binary edge) between LastF-Frame
    const EncPreIntegrator& encpreint = pFrame->GetEncPreInt();
    eEnc = new g2o::EdgeEncNavStatePVR();
    eEnc->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFPVRId)));  // lastF,i
    eEnc->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));   // curF,j
    eEnc->setMeasurement(encpreint.mdelxEij);
    eEnc->setInformation(encpreint.mSigmaEij.inverse());
    cv::Mat Tbe = Frame::mTbc * Frame::mTce;  // for Enc
    eEnc->qRbe = Quaterniond(Converter::toMatrix3d(Tbe.rowRange(0, 3).colRange(0, 3)));
    eEnc->pbe = Converter::toVector3d(Tbe.rowRange(0, 3).col(3));  // for Enc SetParams
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    eEnc->setRobustKernel(rk);
    rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
    optimizer.addEdge(eEnc);
  }

  int nInitialCorrespondences = 0;

  // Set MapPoint Unary edges/Set MapPoint vertices
  const int N = pFrame->N;  // for LastFrame JingWang use Nlast while the VIORBSLAM paper hasn't done this see its
  // Fig.2.! let's try his method!

  vector<g2o::EdgeReprojectPVR*> vpEdgesMono;  // 2*1(_measurement) binary edge<VertexSBAPointXYZ,VertexNavStatePVR>
  vector<size_t> vnIndexEdgeMono;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);  // this can be optimized in RGBD mode

  vector<g2o::EdgeReprojectPVRStereo*> vpEdgesStereo;  // 3*1(ul vl ur) binary edge
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);  // chi2(0.05,2)

  // configs for Prior Hessian
  const bool calc_cov_explicit = true;  // false;
  const int8_t exact_mode =
      calc_cov_explicit ? (int8_t)g2o::kExactNoRobust : (int8_t)g2o::kNotExact;  //(int8_t)g2o::kExactRobust
  const bool calc_cond_jac = false;                   // calculate conditional cov for only PVR or only Bias
  const auto& frame_mps = pFrame->GetMapPointsRef();  // GetMapPointMatches();
  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);  // forbid other threads to rectify pFrame->mvpMapPoints' Position
    int id_mp_beg = 4;

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

        {
          g2o::EdgeReprojectPVR* e = new g2o::EdgeReprojectPVR();
          {
            CV_Assert(pFrame->mapn2in_.size() > i);
            e->SetParams(pFrame->mpCameras[get<0>(pFrame->mapn2in_[i])], Rcb, tcb);
          }

          // 0 Xw, VertexSBAPointXYZ* corresponding to pMP->mWorldPos
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          // 1 Tbw, VertexNavStatePR* corresponding to pFB->mNavState
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));

          Eigen::Matrix<double, 2, 1> obs;
          const cv::KeyPoint& kpUn = pFrame->mvKeys[i];
          obs << kpUn.pt.x, kpUn.pt.y;
          e->setMeasurement(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          // diagonal matrix means independece between x and y pixel noise 2*2 matrix
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaMono);

          optimizer.addEdge(e);

          // record the edge recording feature index
          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
          // e->computeError();
          // PRINT_DEBUG_INFO(get<0>(pFrame->mapn2in_[i]) << "e chi2=" << e->chi2() << " ",
          // imu_tightly_debug_path,"tracking_thread_debug.txt");
        }
      }
    }
  }
  // PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "tracking_thread_debug.txt");

  // at least P3P（well posed equation） EPnP(n>3) (overdetermined equation)
  //  if (nInitialCorrespondences < 3) return 0;

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  // turning point in RobustKernelHuber)
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  int nBadIMU = 0;
  int nInliers = 0;
  // 4 optimizations, each 10 steps, initial value is the same, but inliers are different
  for (size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);  // only call _activeEdges[k].computeError()

    nInliers = 0;
    float chi2close = 1.5 * chi2Mono[it];  // ref from ORB3, can help process hard fast motion & close scene
    nBad = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)  // for 3D-monocular 2D matches, may entered in RGBD!
    {
      g2o::EdgeReprojectPVR* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      // exact_mode here will compute _error again for g2o wont' pop _error if final step's robust_chi2 is larger
      if ((int8_t)g2o::kNotExact > exact_mode || pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      bool bClose = frame_mps[idx]->GetTrackInfoRef().track_depth_ < 10.f;

      if (chi2 > (bClose ? chi2close : chi2Mono[it]) || !e->isDepthPositive()) {  // here e positive added for safety
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        // PRINT_DEBUG_INFO("e chi2=" << chi2 << " ", imu_tightly_debug_path, "tracking_thread_debug.txt");
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliers++;
      }

      if (it == 2) e->setRobustKernel(0);
    }

    // PRINT_DEBUG_INFO(endl, imu_tightly_debug_path, "tracking_thread_debug.txt");
    // it outliers+inliers(/_edges) number<10 only optimize once with RobustKernelHuber
    if (optimizer.edges().size() < 10) break;
    // we tested erasing erroneous IMU edge through chi2 error strategy, but not better, so we abandoned it
  }

  // ref from ORB3, can help process hard fast motion scene
  if (nInliers < 30) {
    nBad = 0;
    // about 3x
    const float chi2MonoOut = 18.f;
    g2o::EdgeReprojectPVR* e1;
    for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeMono[i];
      e1 = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut) {
        e1->setLevel(0);  // for H later
        pFrame->mvbOutlier[idx] = false;
      } else
        nBad++;
    }
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexNavStatePVR* vNSPVR_recov = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(FramePVRId));
  //   cout<<"recovered pwb="<<vNSPVR_recov->estimate().mpwb.transpose()<<" & matches by motion-only
  //   BA:"<<nInitialCorrespondences-nBad<<", before Optimized:"<<nInitialCorrespondences<<endl;
  nsj = vNSPVR_recov->estimate();
  g2o::VertexNavStateBias* vNSBias_recov = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(FrameBiasId));
  const NavState& nsBias_recov = vNSBias_recov->estimate();
  nsj.mdbg = nsBias_recov.mdbg;
  nsj.mdba = nsBias_recov.mdba;
  pFrame->UpdatePoseFromNS();  // update posematrices of pFrame

  // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization,
  // dx'Hdx should be small then next optimized result is appropriate for former BA
  if (0 && bComputeMarg) {
    //     if (nBadIMU>0){}else{
    // get the joint marginalized covariance of PVR&Bias
    if (calc_cov_explicit) {  // explicit will be about 7times faster than g2o's computeMarginals()
      //      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      using Matrix15d = Matrix<double, 15, 15>;
      using Vector15d = Matrix<double, 15, 1>;
      Matrix15d cov_inv;  // PVR BgBa, order is from update vector
      if ((int8_t)g2o::kNotExact > exact_mode) {
        eNSPVR->computeError();
        eNSBias->computeError();
        if (eEnc) {
          eEnc->computeError();
        }
      }
      if ((int8_t)g2o::kNotExact <= exact_mode) {
        cov_inv.block<9, 9>(0, 0) = dynamic_cast<g2o::BaseVertex<9, NavState>*>(vNSFPVR)->A();
        cov_inv.block<6, 6>(9, 9) = dynamic_cast<g2o::BaseVertex<6, NavState>*>(vNSFBias)->A();
        cov_inv.block<9, 6>(0, 9).setZero();
        cov_inv.block<6, 9>(9, 0).setZero();
      } else
        FillCovInv(eNSPVR, eNSBias, eEnc, 0, &vpEdgesMono, &vpEdgesStereo, cov_inv, nullptr, exact_mode);
      if (!bFixedLast) {  // schur complement to get marginalized(lastf) cov_inv(curf)
        Matrix15d cov_inv_last, cov_inv_cur_last;
        if ((int8_t)g2o::kNotExact <= exact_mode) {
          cov_inv_last.block<9, 9>(0, 0) = dynamic_cast<g2o::BaseVertex<9, NavState>*>(vNSKFPVR)->A();
          cov_inv_last.block<6, 6>(9, 9) = dynamic_cast<g2o::BaseVertex<6, NavState>*>(vNSKFBias)->A();
          cov_inv_last.block<9, 6>(0, 9) = eNSPVR->getHessianij(0, 2, exact_mode);
          cov_inv_last.block<6, 9>(9, 0) = cov_inv_last.block<9, 6>(0, 9).transpose();

          cov_inv_cur_last.block<9, 9>(0, 0) = eNSPVR->getHessianij(1, 0, exact_mode);
          cov_inv_cur_last.block<9, 6>(0, 9) = eNSPVR->getHessianij(1, 2, exact_mode);
          cov_inv_cur_last.block<6, 9>(9, 0).setZero();
          cov_inv_cur_last.block<6, 6>(9, 9) = eNSBias->getHessianXji(exact_mode);
        } else {
          eNSPrior->computeError();
          FillCovInv(eNSPVR, eNSBias, eEnc, 2, &vpEdgesMono, &vpEdgesStereo, cov_inv_last, eNSPrior, exact_mode);
          FillCovInv(eNSPVR, eNSBias, eEnc, 1, &vpEdgesMono, &vpEdgesStereo, cov_inv_cur_last, nullptr, exact_mode);
        }
        //[B|E;E^T|C]->[B-EC^(-1)E^T|0;ET|C] => margH = B-EC^(-1)E^T
        Eigen::JacobiSVD<Matrix<double, 15, Eigen::Dynamic>> svd_c(cov_inv_last,
                                                                   Eigen::ComputeThinU | Eigen::ComputeThinV);
        Vector15d w = svd_c.singularValues();
        Matrix15d c_inv = Matrix15d::Zero();
        const int thresh_cond = 1e6;
        const double w_max = w(0);
        //        cerr << "check cond_num=";
        for (int i = 0; i < w.size(); ++i) {
          double cond_num = w_max / w(i);
          //          cerr << cond_num << " " << w(i) << ";";
          // too large condition is sensitive to the error of corresponding input state dimension
          //          if (cond_num <= thresh_cond) {
          if (w(i) > 1e-6) {
            c_inv(i, i) = 1. / w(i);
          }  // so we try to decrease it through discarding the corresponding input state dimension info
          else {
            c_inv(i, i) = 0;
          }
        }
        //        cerr << endl;
        c_inv = svd_c.matrixV() * c_inv * svd_c.matrixU().transpose();  // C=USigmaVT=>C^(-1)=VSigma^(-1)UT
        cov_inv -= cov_inv_cur_last * c_inv * cov_inv_cur_last.transpose();
        if (calc_cond_jac) {
          Eigen::JacobiSVD<Matrix<double, 6, Eigen::Dynamic>> svd_c2(cov_inv.block<6, 6>(9, 9),
                                                                     Eigen::ComputeThinU | Eigen::ComputeThinV);
          Vector6d w2 = svd_c2.singularValues();
          Matrix6d c2_inv = Matrix6d::Zero();
          const double w2_max = w2(0);
          for (int i = 0; i < w2.size(); ++i) {
            c2_inv(i, i) = 1. / w2(i);
          }
          c2_inv = svd_c2.matrixV() * c2_inv * svd_c2.matrixU().transpose();
          Eigen::JacobiSVD<Matrix<double, 9, Eigen::Dynamic>> svd_c3(cov_inv.block<9, 9>(0, 0),
                                                                     Eigen::ComputeThinU | Eigen::ComputeThinV);
          cov_inv.block<9, 9>(0, 0) -= cov_inv.block<9, 6>(0, 9) * c2_inv * cov_inv.block<6, 9>(9, 0);

          Vector9d w3 = svd_c3.singularValues();
          Matrix9d c3_inv = Matrix9d::Zero();
          const double w3_max = w3(0);
          for (int i = 0; i < w3.size(); ++i) {
            c3_inv(i, i) = 1. / w3(i);
          }
          c3_inv = svd_c3.matrixV() * c3_inv * svd_c3.matrixU().transpose();
          cov_inv.block<6, 6>(9, 9) -= cov_inv.block<6, 9>(9, 0) * c3_inv * cov_inv.block<9, 6>(0, 9);
          cov_inv.block<9, 6>(0, 9).setZero();
          cov_inv.block<6, 9>(9, 0).setZero();
        }
      }
      pFrame->mMargCovInv = cov_inv;
      //      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    } else {
      // we could use computeMarginals where margVertices only push back curF's vertices
#ifdef USE_G2O_NEWEST
      g2o::SparseBlockMatrixX spinv;
#else
      g2o::SparseBlockMatrixXd spinv;
#endif
      std::vector<g2o::OptimizableGraph::Vertex*> margVertices;
      margVertices.push_back(optimizer.vertex(FramePVRId));
      margVertices.push_back(optimizer.vertex(FrameBiasId));
      if (!calc_cond_jac) {
        std::vector<std::pair<int, int>> indices;
        for (g2o::OptimizableGraph::VertexContainer::const_iterator it = margVertices.begin(); it != margVertices.end();
             ++it) {
          for (g2o::OptimizableGraph::VertexContainer::const_iterator it2 = it; it2 != margVertices.end(); ++it2)
            indices.push_back(std::pair<int, int>((*it)->hessianIndex(), (*it2)->hessianIndex()));
        }
        optimizer.computeMarginals(spinv, indices);
      } else {
        // each vertexi in margVertices will be marginalized by cholmod linearsolver
        optimizer.computeMarginals(spinv, margVertices);
      }
      // spinv include 2 blocks, 9x9-(0,0) for PVR, 6x6-(1,1) for Bias
      //      cerr<<"check before:"<<pFrame->mMargCovInv<<";fixedlast="<<(int)(bFixedLast)<<endl;
      if (bFixedLast) {
        // here it's already marginalized for lastKF is fixed and SigmaI ind. with
        // SigmaB then H(0,1)=0,H(1,0)=0
        Matrix<double, 15, 15> margCovInv = Matrix<double, 15, 15>::Zero();
        margCovInv.topLeftCorner(9, 9) =
            spinv.block(0,
                        0)
                ->inverse();  // 0 corresponding to the FramePVRId & fixed LastKFPVRId's hessianidx=-1
        margCovInv.bottomRightCorner(6, 6) = spinv.block(1, 1)->inverse();  // 1 corresponding to the FrameBiasId
        pFrame->mMargCovInv = margCovInv;
      } else {
        Matrix<double, 15, 15> margCov = Matrix<double, 15, 15>::Zero();
        margCov.topLeftCorner(9, 9) =
            spinv.block(0,
                        0)
                ->eval();  // I think eval() is useless for there's no confusion here/.noalias()=
        if (!calc_cond_jac) {
          margCov.topRightCorner(9, 6) = spinv.block(0, 1)->eval();
          margCov.bottomLeftCorner(6, 9) = margCov.topRightCorner(9, 6).transpose();
        }
        margCov.bottomRightCorner(6, 6) = spinv.block(1, 1)->eval();
        pFrame->mMargCovInv = margCov.inverse();
      }
      //      std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
      //      double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      //      double ttrack2 = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
      //      cerr<<"check after:"<<pFrame->mMargCovInv<<";cost explicit:"<<ttrack<<"s, g2o:"<<ttrack2<<endl;
    }
    pFrame->mNavStatePrior = nsj;  // pLastF->mNavStatePrior is needed for this func. will be called twice and
    // pLastF->mNavState will also be optimized
    pFrame->mbPrior = true;  // let next tracking uses unfixed lastF mode!
    //     }
  } else {
    bool brobust = exact_mode == g2o::kExactRobust;
    Eigen::Matrix<double, 30, 30> H;
    H.setZero();

    eNSPVR->computeError();
    eNSBias->computeError();
    eNSPrior->computeError();

    // H.block<24, 24>(0, 0) += eNSPVR->GetHessian();
    eNSPVR->linearizeOplus();
    H.block<9, 9>(0, 0) += eNSPVR->getHessian(0, brobust);
    H.block<6, 6>(9, 9) += eNSPVR->getHessian(2, brobust);
    H.block<9, 9>(15, 15) += eNSPVR->getHessian(1, brobust);
    H.block<9, 6>(0, 9) += eNSPVR->getHessianij(0, 2, exact_mode);
    H.block<6, 9>(9, 0) += eNSPVR->getHessianij(2, 0, exact_mode);
    H.block<9, 9>(0, 15) += eNSPVR->getHessianij(0, 1, exact_mode);
    H.block<9, 9>(15, 0) += eNSPVR->getHessianij(1, 0, exact_mode);
    H.block<6, 9>(9, 15) += eNSPVR->getHessianij(2, 1, exact_mode);
    H.block<9, 6>(15, 9) += eNSPVR->getHessianij(1, 2, exact_mode);

    eNSBias->linearizeOplus();
    //    H.block<6, 6>(9, 9) += eNSBias->getHessianXi(brobust);
    //    H.block<6, 6>(9, 24) += eNSBias->getHessianXij(exact_mode);
    //    H.block<6, 6>(24, 9) += eNSBias->getHessianXji(exact_mode);
    //    H.block<6, 6>(24, 24) += eNSBias->getHessianXj(brobust);
    H.block<3, 3>(9, 9) += eNSBias->getHessianXi(brobust).block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += eNSBias->getHessianXij(exact_mode).block<3, 3>(0, 0);
    H.block<3, 3>(24, 9) += eNSBias->getHessianXji(exact_mode).block<3, 3>(0, 0);
    H.block<3, 3>(24, 24) += eNSBias->getHessianXj(brobust).block<3, 3>(0, 0);

    H.block<3, 3>(12, 12) += eNSBias->getHessianXi(brobust).block<3, 3>(3, 3);
    H.block<3, 3>(12, 27) += eNSBias->getHessianXij(exact_mode).block<3, 3>(3, 3);
    H.block<3, 3>(27, 12) += eNSBias->getHessianXji(exact_mode).block<3, 3>(3, 3);
    H.block<3, 3>(27, 27) += eNSBias->getHessianXj(brobust).block<3, 3>(3, 3);

    // H.block<15, 15>(0, 0) += eNSPrior->GetHessian();
    eNSPrior->linearizeOplus();
    H.block<9, 9>(0, 0) += eNSPrior->getHessianXi(brobust);
    H.block<6, 6>(9, 9) += eNSPrior->getHessianXj(brobust);
    H.block<9, 6>(0, 9) += eNSPrior->getHessianXij(exact_mode);
    H.block<6, 9>(9, 0) += eNSPrior->getHessianXji(exact_mode);

    int tot_in = 0, tot_out = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeReprojectPVR* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (!pFrame->mvbOutlier[idx]) {
        e->linearizeOplus();
        H.block<9, 9>(15, 15) += e->getHessian(1, brobust);
        tot_in++;
      } else
        tot_out++;
    }

    H = Marginalize(H, 0, 14);

    pFrame->mMargCovInv = H.block<15, 15>(15, 15);  // cov_inv;//
    pFrame->mNavStatePrior = nsj;  // pLastF->mNavStatePrior is needed for this func. will be called twice and
    // pLastF->mNavState will also be optimized
    pFrame->mbPrior = true;  // let next tracking uses unfixed lastF mode!
  }

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
  PRINT_INFO_MUTEX(blueSTR "Enter local BA..." << pKF->mnId << ", size of localKFs=" << lLocalKeyFrames.size()
                                               << "fixedkfs = " << lFixedCameras.size()
                                               << ", mps=" << lLocalMapPoints.size() << whiteSTR << endl);

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
    g2o::VertexNavStatePR* vns = new g2o::VertexNavStatePR();
    pKFi->UpdateNavStatePVRFromTcw();  // TODO: delete this
    vns->setEstimate(pKFi->GetNavState());
    vns->setId(pKFi->mnId);
    if (pKFi->mnId == 0) ++num_fixed_kf;
    vns->setFixed(pKFi->mnId == 0);  // only fix the vertex of initial KF(KF.mnId==0)
    optimizer.addVertex(vns);
    if (pKFi->mnId > maxKFid) maxKFid = pKFi->mnId;
  }
  // Set Fixed KeyFrame vertices
  for (list<KeyFrame*>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
    KeyFrame* pKFi = *lit;
    g2o::VertexNavStatePR* vns = new g2o::VertexNavStatePR();
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
  typedef struct _BaseEdgeMono {
    g2o::EdgeReprojectPR* pedge;
    size_t idx;
  } BaseEdgeMono;
  vector<BaseEdgeMono> vpEdgesMono;
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

  const float thHuberMono = sqrt(5.991);    // sqrt(e_block)<=sqrt(chi2(0.05,2)) allow power 2 increasing(1/2*e_block)
  const float thHuberStereo = sqrt(7.815);  // chi2(0.05,3)

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
            vpEdgesMono.push_back(BaseEdgeMono());
            BaseEdgeMono& pbaseedgemono = vpEdgesMono.back();
            pbaseedgemono.pedge = e;
            pbaseedgemono.idx = idx;
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
      g2o::EdgeReprojectPR* e = vpEdgesMono[i].pedge;
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

  vector<tuple<KeyFrame*, MapPoint*, size_t>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    const BaseEdgeMono& e = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e.pedge->chi2() > 5.991 || !e.pedge->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.emplace_back(pKFi, pMP, e.idx);  // ready to erase outliers of pKFi && pMP in monocular edges
    }
  }

  for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
    g2o::EdgeReprojectPRStereo* e = vpEdgesStereo[i];
    MapPoint* pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.emplace_back(pKFi, pMP, -1);  // ready to erase outliers of pKFi && pMP in stereo edges
    }
  }

#ifndef NDEBUG
  {
    for (size_t i = 0, iend = vpEdgesEnc.size(); i < iend; i++) {
      g2o::EdgeEncNavStatePR* e = vpEdgesEnc[i];
      // if chi2 error too big(5% wrong)
      if (e->chi2() > 12.592) {
        PRINT_INFO_MUTEX("Enc edge " << redSTR << i << whiteSTR << ", chi2 " << e->chi2() << ". ");
      }
    }
  }
#endif

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty())  // erase the relation between outliers of pKFi(matched mvpMapPoints) && pMP(mObservations)
  {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFrame* pKFi = get<0>(vToErase[i]);
      MapPoint* pMPi = get<1>(vToErase[i]);
      auto idx = get<2>(vToErase[i]);

      // here may erase pMP in mpMap
      ErasePairObs(pKFi, pMPi, idx);
    }
  }

  // Recover optimized data

  // Keyframes update(Pose Tcw...)
  for (list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++) {
    KeyFrame* pKF = *lit;
    g2o::VertexNavStatePR* vns = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(pKF->mnId));
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
                                       const map<KeyFrame*, set<KeyFrame*>>& LoopConnections,
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

  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(
      nMaxKFid + 1);  //+1 for id is from 0, Eigen::aligned_allocator is for Eigen::Quaterniond in g2o::Sim3, vScw used
                      // mainly for all MPs' correction
  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
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

  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

  const Eigen::Matrix<double, 7, 7> matLambda =
      Eigen::Matrix<double, 7,
                    7>::Identity();  //information matrix/Omega/Sigma^(-1) uses default or use Euclidean distance \
    instead of Mahalonobis in error_block=e'e instead of e'Omega(!=I)e, so we also don't use RobustKernel/chi2/outliers/scale concept here, \
    so maybe we can use erroneous edges' concept in RGBDSLAM2 here!

  // Set Loop edges, these cannot be pure odom edges
  for (map<KeyFrame*, set<KeyFrame*>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end();
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
        double dTmp = encpreint.mSigmaEij(2, 2);
        if (dEncBase[0] > dTmp) {  // phi
          dEncBase[0] = dTmp;
        }
        // intuitively we choose sqrt(sigma2x^2+sigma2y^2) or p, makes (Sigma_p,0) to be (1,0) but (Sigma_p/2,
        // Sigma_p/2) to be near (1, 1)
        dTmp = encpreint.mSigmaEij.block<2, 2>(3, 3).norm();
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
          PRINT_INFO_MUTEX(matLambdaEnc << endl);
        }
        PRINT_INFO_MUTEX("Weight0/Odom link: " << pParentKF->mnId << " " << pKF->mnId << " "
                                               << pKF->GetWeight(pParentKF) << endl);
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
  PRINT_INFO_MUTEX("Spanning tree edge end!" << endl);

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
  PRINT_INFO_MUTEX("PoseGraph end!" << endl);
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
