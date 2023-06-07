/**
 * This file is part of VIEO_SLAM
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Converter.h"  //zzh for template
#include "g2otypes.h"

#ifdef USE_G2O_NEWEST
#include "g2o/core/block_solver.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#else

#include "optimizer/g2o/g2o/core/block_solver.h"
#include "optimizer/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "optimizer/g2o/g2o/solvers/linear_solver_eigen.h"  //must before linear_solver_cholmod...
#include "optimizer/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "optimizer/g2o/g2o/solvers/linear_solver_dense.h"
#include "optimizer/g2o/g2o/solvers/linear_solver_cholmod.h"
#include "optimizer/g2o/g2o/core/robust_kernel_impl.h"
#include "optimizer/g2o/g2o/types/types_seven_dof_expmap.h"

#endif

#include "IMUInitialization.h"

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "Pinhole.h"

namespace VIEO_SLAM {

class LoopClosing;

class Optimizer {
 public:
  template <class MatrixNVd = MatrixXd>
  static void FillCovInv(g2o::EdgeNavStatePVR *eNSPVR, g2o::EdgeNavStateBias *eNSBias, g2o::EdgeEncNavStatePVR *eEnc,
                         const int8_t schur_bec, const vector<g2o::EdgeReprojectPVR *> *pvpEdgesMono,
                         const vector<g2o::EdgeReprojectPVRStereo *> *pvpEdgesStereo, MatrixNVd &cov_inv,
                         g2o::EdgeNavStatePriorPVRBias *eNSPrior = nullptr,
                         const int8_t exact_mode = (int8_t)g2o::kNotExact);

  template <class KeyFrame>
  int static PoseOptimization(
      Frame *pFrame, KeyFrame *pLastKF, const cv::Mat &gw, const bool bComputeMarg = false,
      const bool bNoMPs = false);  // 2 frames' motion-only BA, automatically fix/unfix lastF/KF and optimize
                                   // curF/curF&last, if bComputeMarg then save its Hessian
  void static LocalBAPRVIDP(KeyFrame *pKF, int Nlocal, bool *pbStopFlag, Map *pMap, cv::Mat &gw);

  // Nlocal>=1(if <1 it's 1)
  void static LocalBundleAdjustmentNavStatePRV(KeyFrame *pKF, int Nlocal, bool *pbStopFlag, Map *pMap, cv::Mat gw,
                                               bool bLarge = false, bool bRecInit = false);
  // add all KFs && MPs(having edges(monocular/stereo) to some KFs) to optimizer, optimize their Pose/Pos and save it in
  // KF.mTcwGBA && MP.mPosGBA, nScaleOpt==0 no scale optimized, ==1 scale of MapPoints' Pw/Xw optimized, ==2 scale of
  // MapPoints' Xw && KeyFrames' pwb optimized
  int static GlobalBundleAdjustmentNavStatePRV(Map *pMap, const cv::Mat &gw, int nIterations = 5,
                                               bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                               const bool bRobust = true, bool bScaleOpt = false,
                                               IMUInitialization *pimu_initator = nullptr);

  // if bInfo==true, use the R part of SigmaPRV as the Infomation matrix else use Identity(); and if use<Frame> please
  // compute mOdomPreIntIMU before
  template <class IMUKeyFrameInit>
  static int OptimizeInitialGyroBias(const std::vector<IMUKeyFrameInit *> &vpKFInit, Vector3d &bg, bool bInfo = true,
                                     int8_t verbose = false);

  // created by zzh over.

 public:
  void static BundleAdjustment(
      const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP, int nIterations = 5,
      bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0, const bool bRobust = true,
      const bool bEnc = false);  // add all KFs && MPs(having edges(monocular/stereo) to some KFs) to optimizer,
                                 // optimize their Pose/Pos and save it in KF.mTcwGBA && MP.mPosGBA
  void static GlobalBundleAdjustment(
      Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
      const bool bRobust = true,
      const bool bEnc = false);  // pass all KFs && MPs in pMap to BundleAdjustment(KFs,MPs...)

  // local BA, pKF && its covisible neighbors->SetPose(optimizer,vertex(KFid)),
  // pMPs->SetWorldPos(optimizer.vertex(pMP->mnId+maxKFid+1));
  // (all 1st layer covisibility KFs as rectifying KFs(vertices1), MPs seen in these KFs as rectifying MPs(vertices0),
  // left KFs observing MPs as fixed KFs(vertices1,fixed), connecting edges between MPs && KFs as mono/stereo(KF has >=0
  // ur) edges, after addition of vertices and edges it still can return by pbStopFlag; optimize(5)(can be stopped by
  // pbStopFlag), then if mbAbortBA==false-> optimize only inliers(10), update KFs' Pose && MPs' Pos,normal)
  void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int Nlocal = 0);
  // motion-only BA, rectify pFrame->mvbOutlier && pFrame->SetPose(optimizer.vertex(0)), return number of inliers
  int static PoseOptimization(Frame *pFrame, Frame *pLastF = NULL);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
  // PoseGraph Opt., update all KFs' (in pMap) Pose && all MPs' Pos to optimized one and pLoopKF's Pose is fixed(so id0
  // KF's Pose may be changed a bit);
  // add new loop edges(including pCurKF-pLoopKF as previous loop edges next time entering this function) && normal far
  // edges(spanning tree edges/previous loop edges/far part of covisibility graph edges); notice there is validation
  // adding LoopConnections as new loop edges and optimization gives more believe on new loop edges if its number is
  // more
  void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                     const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                     const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                     const map<KeyFrame *, set<KeyFrame *>> &LoopConnections, const bool &bFixScale);

  // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
  // relativeMotionS12-only BA(fixed MPs' vertices);
  // rectify vpMatches1[i](erase outliers) && g2oS12(to optimized S12), return the number of inliers;
  // th2=chi2(1-proba_right,2), vpMatches1[i] matched to pKF1->mvpMapPoints[i]
  static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12,
                          const float th2, const bool bFixScale);
};

// created by zzh
using namespace Eigen;

template <class MatrixNVd>
void Optimizer::FillCovInv(g2o::EdgeNavStatePVR *eNSPVR, g2o::EdgeNavStateBias *eNSBias, g2o::EdgeEncNavStatePVR *eEnc,
                           const int8_t schur_bec, const vector<g2o::EdgeReprojectPVR *> *pvpEdgesMono,
                           const vector<g2o::EdgeReprojectPVRStereo *> *pvpEdgesStereo, MatrixNVd &cov_inv,
                           g2o::EdgeNavStatePriorPVRBias *eNSPrior, const int8_t exact_mode) {
  const bool robust = (int8_t)g2o::kExactRobust == exact_mode;
  // Notice g2o's jacobians of some vertices will be shared for all edges by allocating max vertices number needed by
  // these edges and max size(rows*cols) of jacobians of these vertices and used by starting from data of each
  // vertex jacobian; so getHessian should be done after linearizeOplus for each edge for safety!
  if (eNSPVR) eNSPVR->linearizeOplus();
  if (!schur_bec) {
    if (eNSPVR)
      cov_inv.template block<9, 9>(0, 0) = eNSPVR->getHessian(1, robust);
    else
      cov_inv.template block<9, 9>(0, 0).setZero();
    // notice On-Manifold paper puts 1st-order approximation of dbgdba to ePVR, so J_etaij_dbgdba = 0
    // but J_ePVR_dbgdba(last) != 0, J_ePVR_dbgdba(cur) = 0, J_eBias_dbgdba = I, J_eBias_PVR = 0
    cov_inv.template block<9, 6>(0, 9).setZero();
    cov_inv.template block<6, 9>(9, 0).setZero();
  } else if (2 == schur_bec) {
    if (eNSPVR) {
      cov_inv.template block<9, 9>(0, 0) = eNSPVR->getHessian(0, robust);
      cov_inv.template block<9, 6>(0, 9) = eNSPVR->getHessianij(0, 2, exact_mode);
      // eNSPVR->getHessianij(2, 0, exact_mode);  //
      cov_inv.template block<6, 9>(9, 0) = cov_inv.template block<9, 6>(0, 9).template transpose();
      cov_inv.template block<6, 6>(9, 9) = eNSPVR->getHessian(2, robust);
    } else
      cov_inv.setZero();
  } else {
    if (eNSPVR) {
      cov_inv.template block<9, 9>(0, 0) = eNSPVR->getHessianij(1, 0, exact_mode);
      cov_inv.template block<9, 6>(0, 9) = eNSPVR->getHessianij(1, 2, exact_mode);
    } else
      cov_inv.template block<9, 15>(0, 0).setZero();
    cov_inv.template block<6, 9>(9, 0).setZero();  // no cur dbgdba enter ePVR edge, so J_ePVR_dbgdba(cur) = 0
  }
  assert(eNSBias);
  eNSBias->linearizeOplus();
  if (!schur_bec) {
    cov_inv.template block<6, 6>(9, 9) = eNSBias->getHessianXj(robust);

    if (pvpEdgesMono)
      for (auto iter = pvpEdgesMono->begin(), iterend = pvpEdgesMono->end(); iterend != iter; ++iter) {
        g2o::EdgeReprojectPVR *e = *iter;
        if (!e->level()) {
          e->linearizeOplus();
          cov_inv.template block<9, 9>(0, 0) += e->getHessian(1, robust);
        }
      }
    if (pvpEdgesStereo)
      for (auto iter = pvpEdgesStereo->begin(), iterend = pvpEdgesStereo->end(); iterend != iter; ++iter) {
        g2o::EdgeReprojectPVRStereo *e = *iter;
        if (!e->level()) {
          e->linearizeOplus();
          cov_inv.template block<9, 9>(0, 0) += e->getHessian(1, robust);
        }
      }
  } else if (2 == schur_bec) {
    cov_inv.template block<6, 6>(9, 9) += eNSBias->getHessianXi(robust);
    assert(eNSPrior);
    eNSPrior->linearizeOplus();
    cov_inv.template block<9, 9>(0, 0) += eNSPrior->getHessianXi(robust);
    cov_inv.template block<6, 6>(9, 9) += eNSPrior->getHessianXj(robust);
    // fix old bug in VIORBSLAM2 old code! now H can correlate PVR with Bias, useful in hard scene!
    cov_inv.template block<9, 6>(0, 9) += eNSPrior->getHessianXij(exact_mode);
    // +=eNSPrior->getHessianXji(exact_mode);  //
    cov_inv.template block<6, 9>(9, 0) = cov_inv.template block<9, 6>(0, 9).template transpose();
  } else {
    cov_inv.template block<6, 6>(9, 9) = eNSBias->getHessianXji(exact_mode);
  }
  if (eEnc) {
    eEnc->linearizeOplus();
    if (!schur_bec)
      cov_inv.template block<9, 9>(0, 0) += eEnc->getHessianXj(robust);
    else if (2 == schur_bec)
      cov_inv.template block<9, 9>(0, 0) += eEnc->getHessianXi(robust);
    else {
      cov_inv.template block<9, 9>(0, 0) += eEnc->getHessianXji(exact_mode);
    }
  }
}

template <class KeyFrame>
int Optimizer::PoseOptimization(Frame *pFrame, KeyFrame *pLastKF, const cv::Mat &gw, const bool bComputeMarg,
                                const bool bNoMPs) {
  // automatically judge if fix lastF/KF(always fixed)
  bool bFixedLast = true;
  if (pLastKF->mbPrior) bFixedLast = false;
  // Extrinsics
  Matrix3d Rcb = pFrame->meigRcb;
  Vector3d tcb = pFrame->meigtcb;
  // Gravity vector in world frame
  Vector3d GravityVec = Converter::toVector3d(gw);

  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
      unique_ptr<g2o::BlockSolverX>(new g2o::BlockSolverX(unique_ptr<g2o::BlockSolverX::LinearSolverType>(
          new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>()))));  // descending/optimization strategy is
                                                                                  // still LM
#else
  // 9*1 is Log(R),t,v/P/pvR, 6*1 bgi,bai/bi/Bias, (3*1 is location of landmark,) 3 types of
  // vertices so using BlockSolverX, though here 9_6 is also OK for unary edge in BA
  g2o::BlockSolverX::LinearSolverType *linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  // sparse Cholesky factorization. linear equation solver changed by JingWang
  //  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

  // g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  // descending/optimization strategy is still LM
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif
  // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);//try this!
  optimizer.setAlgorithm(solver);

  // Set Frame & fixed KeyFrame's vertices, see VIORBSLAM paper (4)~(8)
  const int FramePVRId = 0, FrameBiasId = 1, LastKFPVRId = 2, LastKFBiasId = 3;
  NavState &nsj = pFrame->GetNavStateRef();
  // Set Frame vertex PVR/Bias
  g2o::VertexNavStatePVR *vNSFPVR = new g2o::VertexNavStatePVR();
  vNSFPVR->setEstimate(nsj);
  vNSFPVR->setId(FramePVRId);
  vNSFPVR->setFixed(false);
  optimizer.addVertex(vNSFPVR);
  g2o::VertexNavStateBias *vNSFBias = new g2o::VertexNavStateBias();
  vNSFBias->setEstimate(nsj);
  vNSFBias->setId(FrameBiasId);
  vNSFBias->setFixed(false);
  optimizer.addVertex(vNSFBias);
  // Set KeyFrame vertex PVR/Bias
  g2o::VertexNavStatePVR *vNSKFPVR = new g2o::VertexNavStatePVR();
  vNSKFPVR->setEstimate(pLastKF->GetNavState());
  vNSKFPVR->setId(LastKFPVRId);
  vNSKFPVR->setFixed(bFixedLast);
  optimizer.addVertex(vNSKFPVR);
  g2o::VertexNavStateBias *vNSKFBias = new g2o::VertexNavStateBias();
  vNSKFBias->setEstimate(pLastKF->GetNavState());
  vNSKFBias->setId(LastKFBiasId);
  vNSKFBias->setFixed(bFixedLast);
  optimizer.addVertex(vNSKFBias);

  bool bodom_edge = false;
  g2o::EdgeNavStatePVR *eNSPVR = nullptr;
  // Set IMU_I/PVR(B) edge(ternary/multi edge) between LastKF-Frame
  const IMUPreintegrator &imupreint = pFrame->GetIMUPreInt();
  if (imupreint.mdeltatij) {
    bodom_edge = true;
    eNSPVR = new g2o::EdgeNavStatePVR();
    eNSPVR->setVertex(
        0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFPVRId)));  // PVRi, i is keyframe's id
    eNSPVR->setVertex(
        1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));  // PVRj, j here is frame's id
    eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFBiasId)));  // bi
    eNSPVR->setMeasurement(imupreint);                 // set delta~PVRij/delta~pij,delta~vij,delta~Rij
    Matrix9d Infoij = imupreint.GetProcessedInfoij();  // mSigmaij.inverse();
#define USE_ZZH_IMU_EDGE_FEBA
#ifdef USE_ZZH_IMU_EDGE_FEBA
    if (bFixedLast) {
      eNSPVR->setInformation(Infoij * 1e-2);

      g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
      eNSPVR->setRobustKernel(rk);
      // chi2(0.05/0.01,9), 16.919/21.666 for 0.95/0.99 9DoF, but JingWang uses 100*21.666
      rk->setDelta(sqrt(16.919));
    } else
#endif
      eNSPVR->setInformation(Infoij);
    eNSPVR->SetParams(GravityVec);
    optimizer.addEdge(eNSPVR);
    // Set IMU_RW/Bias edge(binary edge) between LastKF-Frame
  }
  // eNSBias always exists for prior H calc.
  g2o::EdgeNavStateBias *eNSBias = new g2o::EdgeNavStateBias();
  eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFBiasId)));  // bi
  eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FrameBiasId)));   // bj
  eNSBias->setMeasurement(imupreint);
  Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
  InvCovBgaRW.topLeftCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmabg2;  // Gyroscope bias random walk, covariance INVERSE
  InvCovBgaRW.bottomRightCorner(3, 3) =
      Matrix3d::Identity() * IMUDataBase::mInvSigmaba2;  // Accelerometer bias random walk, covariance INVERSE
  // see Manifold paper (47), notice here is Omega_d/Sigma_d.inverse()
  double deltatij = imupreint.mdeltatij ? imupreint.mdeltatij : pFrame->mTimeStamp - pLastKF->mTimeStamp;
#ifdef USE_ZZH_IMU_EDGE_FEBA
  if (bFixedLast) {
    eNSBias->setInformation(InvCovBgaRW / deltatij * 1e-2);

    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    eNSBias->setRobustKernel(rk);
    rk->setDelta(sqrt(12.592));  // chi2(0.05/0.01,6), 12.592/16.812 for 0.95/0.99 6DoF, but JW uses 16.812
  } else
#endif
    eNSBias->setInformation(InvCovBgaRW / deltatij);
  optimizer.addEdge(eNSBias);
  // Set Prior edge(binary edge) for Last Frame, from mMargCovInv
  g2o::EdgeNavStatePriorPVRBias *eNSPrior = NULL;
  if (!bFixedLast) {
    eNSPrior = new g2o::EdgeNavStatePriorPVRBias();
    eNSPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFPVRId)));
    eNSPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFBiasId)));
    eNSPrior->setMeasurement(pLastKF->mNavStatePrior);
    Matrix<double, 15, 15> H = pLastKF->mMargCovInv;
    //    H = (H + H) / 2;
    //    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
    //    Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
    //    for (int i = 0; i < 15; i++)
    //      if (eigs[i] < 1e-12) eigs[i] = 0;
    //    H = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    eNSPrior->setInformation(H);
    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    eNSPrior->setRobustKernel(rk);
    rk->setDelta(sqrt(25));  // thHuberNavState:chi2(0.05,15)=25 or chi2(0.01,15)=30.5779
    optimizer.addEdge(eNSPrior);
  }
  // Set Enc edge(binary) between LastKF-Frame
  g2o::EdgeEncNavStatePVR *eEnc = nullptr;
  if (pFrame->GetEncPreInt().mdeltatij) {
    bodom_edge = true;
    // Set Enc edge(binary edge) between LastF-Frame
    const EncPreIntegrator &encpreint = pFrame->GetEncPreInt();
    eEnc = new g2o::EdgeEncNavStatePVR();
    eEnc->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFPVRId)));  // lastF,i
    eEnc->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));   // curF,j
    eEnc->setMeasurement(encpreint.mdelxEij);
    eEnc->setInformation(encpreint.mSigmaEij.inverse());
    cv::Mat Tbe = Frame::mTbc * Frame::mTce;  // for Enc
    eEnc->qRbe = Quaterniond(Converter::toMatrix3d(Tbe.rowRange(0, 3).colRange(0, 3)));
    eEnc->pbe = Converter::toVector3d(Tbe.rowRange(0, 3).col(3));  // for Enc SetParams
    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    eEnc->setRobustKernel(rk);
    rk->setDelta(sqrt(12.592));  // chi2(0.05,6)=12.592//chi2(0.05,3)=7.815
    optimizer.addEdge(eEnc);
  }

  int nInitialCorrespondences = 0;

  // Set MapPoint Unary edges/Set MapPoint vertices
  const int N = pFrame->N;  // for LastFrame JingWang use Nlast while the VIORBSLAM paper hasn't done this see its
                            // Fig.2.! let's try his method!

  vector<g2o::EdgeReprojectPVR *> vpEdgesMono;  // 2*1(_measurement) binary edge<VertexSBAPointXYZ,VertexNavStatePVR>
  vector<size_t> vnIndexEdgeMono;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);  // this can be optimized in RGBD mode

  vector<g2o::EdgeReprojectPVRStereo *> vpEdgesStereo;  // 3*1(ul vl ur) binary edge
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);    // chi2(0.05,2)
  const float deltaStereo = sqrt(7.815);  // chi2 distribution chi2(0.05,3), the huber kernel delta

  Pinhole CamInst;
  bool usedistort = Frame::usedistort_ && pFrame->mpCameras.size();
  // configs for Prior Hessian
  const bool calc_cov_explicit = true;  // false;
#define USE_ROBUST_PRIOR_EDGE
  const int8_t exact_mode = calc_cov_explicit ?
#ifdef USE_ROBUST_PRIOR_EDGE
                                              (int8_t)g2o::kExactRobust
#else
                                              (int8_t)g2o::kExactNoRobust
#endif
                                              : (int8_t)g2o::kNotExact;  //(int8_t)g2o::kExactRobust
  const bool calc_cond_jac = false;                   // calculate conditional cov for only PVR or only Bias
  const auto &frame_mps = pFrame->GetMapPointsRef();  // GetMapPointMatches();
  {
    if (!usedistort) {
      CamInst.setParameter(pFrame->fx, 0);
      CamInst.setParameter(pFrame->fy, 1);
      CamInst.setParameter(pFrame->cx, 2);
      CamInst.setParameter(pFrame->cy, 3);
    }

    unique_lock<mutex> lock(MapPoint::mGlobalMutex);  // forbid other threads to rectify pFrame->mvpMapPoints' Position
    int id_mp_beg = 4;

    for (int i = 0; i < N; i++) {
      MapPoint *pMP = frame_mps[i];
      if (pMP) {
        // add fixed mp vertices for motion_only BA
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();  //<3,Eigen::Vector3d>, for MPs' Xw
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
          g2o::EdgeReprojectPVR *e = new g2o::EdgeReprojectPVR();
          if (!usedistort)
            e->SetParams(&CamInst, Rcb, tcb);
          else {
            CV_Assert(pFrame->mapn2in_.size() > i);
            e->SetParams(pFrame->mpCameras[get<0>(pFrame->mapn2in_[i])], Rcb, tcb);
          }

          // 0 Xw, VertexSBAPointXYZ* corresponding to pMP->mWorldPos
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
          // 1 Tbw, VertexNavStatePR* corresponding to pFB->mNavState
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));

          Eigen::Matrix<double, 2, 1> obs;
          const cv::KeyPoint &kpUn = !usedistort ? pFrame->mvKeysUn[i] : pFrame->mvKeys[i];
          obs << kpUn.pt.x, kpUn.pt.y;
          e->setMeasurement(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          // diagonal matrix means independece between x and y pixel noise 2*2 matrix
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaMono);

          optimizer.addEdge(e);

          // record the edge recording feature index
          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
          // e->computeError();
          // PRINT_DEBUG_INFO(get<0>(pFrame->mapn2in_[i]) << "e chi2=" << e->chi2() << " ",
          // mlog::vieo_slam_debug_path,"tracking_thread_debug.txt");
        } else  // Stereo observation
        {
          g2o::EdgeReprojectPVRStereo *e = new g2o::EdgeReprojectPVRStereo();
          if (!usedistort)
            e->SetParams(&CamInst, Rcb, tcb, &pFrame->mbf);
          else {
            CV_Assert(pFrame->mapn2in_.size() > i);
            e->SetParams(pFrame->mpCameras[get<0>(pFrame->mapn2in_[i])], Rcb, tcb, &pFrame->mbf);
          }

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));

          // SET EDGE
          Eigen::Matrix<double, 3, 1> obs;
          const cv::KeyPoint &kpUn = !usedistort ? pFrame->mvKeysUn[i] : pFrame->mvKeys[i];
          const float &kp_ur = pFrame->mvuRight[i];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
          e->setMeasurement(obs);  // edge parameter/measurement formula output z
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          // optimization target=KernelHuber(block)=H(e)={1/2*e sqrt(e)<=delta;delta(sqrt(e)-1/2*delta) others}
          g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaStereo);

          optimizer.addEdge(e);  //_error is the edge output

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);  // record the edge recording feature index
        }
      }
    }
  }
  // PRINT_DEBUG_INFO(endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  PRINT_INFO_FILE("moba enterobs=" << nInitialCorrespondences << endl, mlog::vieo_slam_debug_path,
                  "tracking_thread_debug.txt");

  // at least P3P（well posed equation） EPnP(n>3) (overdetermined equation)
  int nBad = 0;
  bool bno_iter = false;
  if ((nInitialCorrespondences < 3 && !bNoMPs)) {
    bodom_edge = false;
    if (!bodom_edge) {
      return 0;
    } else {
      // no P3P limit when odom edge exist, where we need to update H
      for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeReprojectPVR *e = vpEdgesMono[i];
        const size_t idx = vnIndexEdgeMono[i];
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        ++nBad;
      }
      for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeReprojectPVRStereo *e = vpEdgesStereo[i];
        const size_t idx = vnIndexEdgeStereo[i];
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        ++nBad;
      }
      vpEdgesMono.clear();
      vpEdgesStereo.clear();
    }
    bno_iter = true;
  }

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  // chi2(0.05,3), error_block limit(over will be outliers,here also lead to
  // turning point in RobustKernelHuber)
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  const int its[4] = {10, 10, 10, 10};

  int nBadIMU = 0;
  // 4 optimizations, each 10 steps, initial value is the same, but inliers are different
  for (size_t it = 0; it < 4; it++) {
    // PRINT_DEBUG_INFO(it << ":" << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
    // Reset estimate for vertexj
    if (!bodom_edge) {
      vNSFPVR->setEstimate(nsj);
      vNSFBias->setEstimate(nsj);
      if (!bFixedLast) {
        vNSKFPVR->setEstimate(pLastKF->GetNavState());
        vNSKFBias->setEstimate(pLastKF->GetNavState());
      }
    }

    // default edges' level is 0, so initially use all edges to optimize, after it=0, just use inlier
    // edges(_activeEdges) to optimize
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);  // only call _activeEdges[k].computeError()

    if (bno_iter) break;

    float chi2close = 1.5 * chi2Mono[it];  // ref from ORB3, can help process hard fast motion & close scene
    nBad = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)  // for 3D-monocular 2D matches, may entered in RGBD!
    {
      g2o::EdgeReprojectPVR *e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      // exact_mode here will compute _error again for g2o wont' pop _error if final step's robust_chi2 is larger
      if ((int8_t)g2o::kNotExact > exact_mode || pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();
      bool bClose = frame_mps[idx]->GetTrackInfoRef().track_depth_ < (10 < pFrame->mThDepth ? pFrame->mThDepth : 10);

      if (chi2 > (bClose ? chi2close : chi2Mono[it]) || !e->isDepthPositive()) {  // here e positive added for safety
        pFrame->mvbOutlier[idx] = true;
        //        if ((int8_t)g2o::kNotExact <= exact_mode || it < 3 && optimizer.edges().size() >= 10)
        e->setLevel(1);
        nBad++;
      } else {
        // PRINT_DEBUG_INFO("e chi2=" << chi2 << " ", mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
        pFrame->mvbOutlier[idx] = false;
        //        if ((int8_t)g2o::kNotExact <= exact_mode || it < 3 && optimizer.edges().size() >= 10)
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)  // for 3D-stereo 2D matches
    {
      g2o::EdgeReprojectPVRStereo *e = vpEdgesStereo[i];

      const size_t idx = vnIndexEdgeStereo[i];

      // at 1st time, all false for all edges is at level 0 or inliers(supposed),so e._error is computed by g2o
      if ((int8_t)g2o::kNotExact > exact_mode || pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();  // chi2=e'*Omiga*e

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        //        if ((int8_t)g2o::kNotExact <= exact_mode || it < 3 && optimizer.edges().size() >= 10)
        e->setLevel(1);  // adjust the outlier edges' level to 1
        nBad++;
      } else {
        //        if ((int8_t)g2o::kNotExact <= exact_mode || it < 3 && optimizer.edges().size() >= 10)
        e->setLevel(0);  // maybe adjust the outliers' level to inliers' level
        pFrame->mvbOutlier[idx] = false;
      }

      // let the final(it==3) optimization use no RobustKernel; this function will delete _robustkernel first
      if (it == 2) e->setRobustKernel(0);
    }

    // PRINT_DEBUG_INFO(endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
    // it outliers+inliers(/_edges) number<10 only optimize once with RobustKernelHuber
    if (optimizer.edges().size() < 10) break;
    // we tested erasing erroneous IMU edge through chi2 error strategy, but not better, so we abandoned it
  }

  // ref from ORB3, can help process hard fast motion scene
  int nInliers = nInitialCorrespondences - nBad;
  if (!bno_iter && nInliers < 30) {
    nBad = 0;
    // about 3x
    const float chi2MonoOut = 18.f;
    const float chi2StereoOut = 24.f;
    g2o::EdgeReprojectPVR *e1;
    g2o::EdgeReprojectPVRStereo *e2;
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
    for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++) {
      const size_t idx = vnIndexEdgeStereo[i];
      e2 = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut) {
        e2->setLevel(0);  // for H later
        pFrame->mvbOutlier[idx] = false;
      } else
        nBad++;
    }
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexNavStatePVR *vNSPVR_recov = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(FramePVRId));
  //   cout<<"recovered pwb="<<vNSPVR_recov->estimate().mpwb.transpose()<<" & matches by motion-only
  //   BA:"<<nInitialCorrespondences-nBad<<", before Optimized:"<<nInitialCorrespondences<<endl;
  nsj = vNSPVR_recov->estimate();
  g2o::VertexNavStateBias *vNSBias_recov = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(FrameBiasId));
  const NavState &nsBias_recov = vNSBias_recov->estimate();
  nsj.mdbg = nsBias_recov.mdbg;
  nsj.mdba = nsBias_recov.mdba;
  pFrame->UpdatePoseFromNS();  // update posematrices of pFrame

  // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization,
  // dx'Hdx should be small then next optimized result is appropriate for former BA
  if (bComputeMarg) {
    //     if (nBadIMU>0){}else{
    // get the joint marginalized covariance of PVR&Bias
    if (calc_cov_explicit) {  // explicit will be about 7times faster than g2o's computeMarginals()
      //      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      using Matrix15d = Matrix<double, 15, 15>;
      using Vector15d = Matrix<double, 15, 1>;
      Matrix15d cov_inv;  // PVR BgBa, order is from update vector
      if ((int8_t)g2o::kNotExact > exact_mode) {
        if (eNSPVR) eNSPVR->computeError();
        eNSBias->computeError();
        if (eEnc) {
          eEnc->computeError();
        }
      }
      if ((int8_t)g2o::kNotExact <= exact_mode) {
        cov_inv.block<9, 9>(0, 0) = dynamic_cast<g2o::BaseVertex<9, NavState> *>(vNSFPVR)->A();
        cov_inv.block<6, 6>(9, 9) = dynamic_cast<g2o::BaseVertex<6, NavState> *>(vNSFBias)->A();
        cov_inv.block<9, 6>(0, 9).setZero();
        cov_inv.block<6, 9>(9, 0).setZero();
      } else
        FillCovInv(eNSPVR, eNSBias, eEnc, 0, &vpEdgesMono, &vpEdgesStereo, cov_inv, nullptr, exact_mode);
      if (!bFixedLast) {  // schur complement to get marginalized(lastf) cov_inv(curf)
        Matrix15d cov_inv_last, cov_inv_cur_last;
        if ((int8_t)g2o::kNotExact <= exact_mode) {
          cov_inv_last.block<9, 9>(0, 0) = dynamic_cast<g2o::BaseVertex<9, NavState> *>(vNSKFPVR)->A();
          cov_inv_last.block<6, 6>(9, 9) = dynamic_cast<g2o::BaseVertex<6, NavState> *>(vNSKFBias)->A();
          if (eNSPVR)
            cov_inv_last.block<9, 6>(0, 9) = eNSPVR->getHessianij(0, 2, exact_mode);
          else
            cov_inv_last.block<9, 6>(0, 9).setZero();
          cov_inv_last.block<6, 9>(9, 0) = cov_inv_last.block<9, 6>(0, 9).transpose();

          if (eNSPVR) {
            cov_inv_cur_last.block<9, 9>(0, 0) = eNSPVR->getHessianij(1, 0, exact_mode);
            cov_inv_cur_last.block<9, 6>(0, 9) = eNSPVR->getHessianij(1, 2, exact_mode);
          } else
            cov_inv_cur_last.block<9, 15>(0, 0).setZero();
          cov_inv_cur_last.block<6, 9>(9, 0).setZero();
          cov_inv_cur_last.block<6, 6>(9, 9) = eNSBias->getHessianXji(exact_mode);
          if (eEnc) cov_inv_cur_last.block<9, 9>(0, 0) += eEnc->getHessianXji(exact_mode);
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
          //          if (w(i) > 1e-6) {
          c_inv(i, i) = 1. / w(i);
          //          }  // so we try to decrease it through discarding the corresponding input state dimension info
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
      std::vector<g2o::OptimizableGraph::Vertex *> margVertices;
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
    pFrame->mbPrior = true;        // let next tracking uses unfixed lastF mode!
    //     }
  }

  return nInitialCorrespondences - nBad;  // number of inliers
}

extern int8_t kCoeffPriorDefault;
template <class IMUKeyFrameInit>
int Optimizer::OptimizeInitialGyroBias(const std::vector<IMUKeyFrameInit *> &vpKFInit, Vector3d &bg, bool bInfo,
                                       int8_t verbose) {
  Matrix3d Rcb = Frame::meigRcb;

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEWEST
  g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(
      unique_ptr<g2o::BlockSolverX>(new g2o::BlockSolverX(unique_ptr<g2o::BlockSolverX::LinearSolverType>(
          new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()))));  // suggested by VIORBSLAM paper IV-A
#else
  g2o::BlockSolverX::LinearSolverType *linearSolver;  // canbe improved here for using fixed BlockSolver

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton *solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);  // suggested by VIORBSLAM paper IV-A
#endif
  optimizer.setAlgorithm(solver);

  // Add vertex of gyro bias, to optimizer graph
  g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias();
  vBiasg->setEstimate(Vector3d::Zero());  // zero bias seed, see IV-A in VIORBSLAM paper
  vBiasg->setId(0);
  optimizer.addVertex(vBiasg);

  double sum_dt_tmp = 0;

  // Add unary edges for gyro bias vertex
  int N = vpKFInit.size();
  int num_equations = 0;
  // Eigen::Matrix3d lastRcb;
  double inv_sigmabg2 = 1.;
  for (int i = 0; i < N; i++) {
    if (i == 0) continue;  // Ignore the first KF
    // notice this should be computed before calling this function
    const IMUPreintegrator &imupreint = vpKFInit[i]->GetIMUPreInt();
    if (imupreint.mdeltatij == 0) continue;
    ++num_equations;
    assert(imupreint.mdeltatij > 0);

    if (!sum_dt_tmp) inv_sigmabg2 = IMUData::mInvSigmabg2;  //[imupreint.device_];
    sum_dt_tmp += imupreint.mdeltatij;

    g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
    eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
    // measurement is not used in EdgeGyrBias
    eBiasg->deltaRij = imupreint.mRij;  // deltaRij/deltaRii+1
    eBiasg->JgRij = imupreint.mJgRij;   // Jg_deltaR
    // Rwbi=Rwci*Rcb
    eBiasg->Rwbi = Converter::toMatrix3d(vpKFInit[i - 1]->GetTcwRef().rowRange(0, 3).colRange(0, 3).t()) * Rcb;
    // Rwbj/Rwbi+1=Rwcj/Rwci+1 * Rcb
    eBiasg->Rwbj = Converter::toMatrix3d(vpKFInit[i]->GetTcwRef().rowRange(0, 3).colRange(0, 3).t()) * Rcb;
    if (bInfo)
      eBiasg->setInformation(imupreint.mSigmaijPRV.block<3, 3>(3, 3).inverse());
    else
      eBiasg->setInformation(Matrix3d::Identity());  // JingWang uses it in vector<Frame>
    optimizer.addEdge(eBiasg);
  }
  if (num_equations < 1) return num_equations;

  // It's approximately a linear estimator, so 1 iteration is enough. for dbg=bg << 1 and here uses GN not LM
  if (mlog::kVerbDeb < verbose) optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(1);

  g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));
  bg += vBgEst->estimate();

  return num_equations;
}

}  // namespace VIEO_SLAM

#endif  // OPTIMIZER_H
