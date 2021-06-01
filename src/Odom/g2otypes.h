#ifndef G2OTYPES_H
#define G2OTYPES_H

#ifdef USE_G2O_NEWEST
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#else
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#endif
#include "NavState.h"
#include "OdomPreIntegrator.h"

namespace g2o {

using namespace VIEO_SLAM;
using namespace Eigen;

// extend edges to get H

typedef enum HessianExactMode {
  kExactNoRobust,
  kExactRobust,
  kNotExact
} eHessianExactMode;

template <int D, typename E, typename VertexXi>
class BaseUnaryEdgeEx : public BaseUnaryEdge<D, E, VertexXi> {
 protected:
  using Base = BaseUnaryEdge<D, E, VertexXi>;
  using Base::_jacobianOplusXi;
  using MatrixXid = Matrix<double, VertexXi::Dimension, VertexXi::Dimension>;
  using Base::robustInformation;
  using Base::vertices;

 public:
  using Base::information;
  using typename Base::JacobianXiOplusType;
  using typename Base::InformationType;
  using Base::robustKernel;
  using Base::chi2;
  virtual void getRho(bool &robust, Vector3d &rho) const {
    if (robust) {
      const RobustKernel* robustkernel = robustKernel();
      if (robustkernel)
        robustkernel->robustify(chi2(), rho);
      else
        robust = false;
    }
  }

  //please call linearizeOplus() before calling this function for each edge for g2o share jacobians workspace for all
  // edges
  virtual MatrixXid getHessianXi(bool robust = true) const {
    const JacobianXiOplusType& jac = _jacobianOplusXi;
    Vector3d rho;
    getRho(robust, rho);
    const InformationType& rinfo = robust ? rho[1] * information() : information();
    return jac.transpose() * rinfo * jac;
  }
};

template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseBinaryEdgeEx : public BaseBinaryEdge<D, E, VertexXi, VertexXj> {
 protected:
  using Base = BaseBinaryEdge<D, E, VertexXi, VertexXj>;
  using Base::_jacobianOplusXi;
  using Base::_jacobianOplusXj;
  using MatrixXid = Matrix<double, VertexXi::Dimension, VertexXi::Dimension>;
  using MatrixXjd = Matrix<double, VertexXj::Dimension, VertexXj::Dimension>;
  using MatrixXijd = Matrix<double, VertexXi::Dimension, VertexXj::Dimension>;
  using MatrixXjid = Matrix<double, VertexXj::Dimension, VertexXi::Dimension>;
  using Base::robustInformation;
  using Base::_hessian;
  using Base::_hessianTransposed;
  using Base::_hessianRowMajor;

 public:
  using Base::information;
  using typename Base::JacobianXiOplusType;
  using typename Base::JacobianXjOplusType;
  using typename Base::InformationType;
  using Base::robustKernel;
  using Base::chi2;
  using typename Base::HessianBlockType;
  using typename Base::HessianBlockTransposedType;
  virtual void getRho(bool &robust, Vector3d &rho) const {
    if (robust) {
      const RobustKernel* robustkernel = robustKernel();
      if (robustkernel)
        robustkernel->robustify(chi2(), rho);
      else
        robust = false;
    }
  }

  virtual MatrixXid getHessianXi(bool robust = true) const {
    const JacobianXiOplusType& jac = _jacobianOplusXi;
    Vector3d rho;
    getRho(robust, rho);
    const InformationType & rinfo = robust ? rho[1] * information() : information();
    return jac.transpose() * rinfo * jac;
  }
  virtual MatrixXjd getHessianXj(bool robust = true) const {
    const JacobianXjOplusType& jac = _jacobianOplusXj;
    Vector3d rho;
    getRho(robust, rho);
    const InformationType & rinfo = robust ? rho[1] * information() : information();
    return jac.transpose() * rinfo * jac;
  }
  virtual MatrixXijd getHessianXij(int8_t exact_mode = (int8_t)kExactRobust) const {
    if ((int8_t)kNotExact == exact_mode) {
      if (_hessianRowMajor) {
        return MatrixXijd(_hessianTransposed.transpose());
      } else {
        return MatrixXijd(_hessian);
      }
    } else {
      const JacobianXiOplusType& jaci = _jacobianOplusXi;
      const JacobianXjOplusType& jacj = _jacobianOplusXj;
      Vector3d rho;
      bool robust = (int8_t)kExactRobust == exact_mode;
      getRho(robust, rho);
      const InformationType& rinfo = robust ? rho[1] * information() : information();
      return jaci.transpose() * rinfo * jacj;
    }
  }
  virtual MatrixXjid getHessianXji(int8_t exact_mode = (int8_t)kExactRobust) const {
    if ((int8_t)kNotExact == exact_mode) {
      if (_hessianRowMajor) {
        return MatrixXjid(_hessianTransposed);
      } else {
        return MatrixXjid(_hessian.transpose());
      }
    } else {
      const JacobianXiOplusType& jaci = _jacobianOplusXi;
      const JacobianXjOplusType& jacj = _jacobianOplusXj;
      Vector3d rho;
      bool robust = (int8_t)kExactRobust == exact_mode;
      getRho(robust, rho);
      const InformationType& rinfo = robust ? rho[1] * information() : information();
      return jacj.transpose() * rinfo * jaci;
    }
  }
};

template <int D, typename E>
class BaseMultiEdgeEx : public BaseMultiEdge<D, E> {
 protected:
  using Base = BaseMultiEdge<D, E>;
  using Base::_jacobianOplus;
  using Base::robustInformation;//TODO: add const postfix to g2o robustInformation
  using Base::_hessian;

 public:
  using Base::information;
  using typename Base::JacobianType;
  using typename Base::InformationType;
  using Base::robustKernel;
  using Base::chi2;
  using typename Base::HessianHelper;
  virtual void getRho(bool &robust, Vector3d &rho) const {
    if (robust) {
      const RobustKernel* robustkernel = robustKernel();
      if (robustkernel)
        robustkernel->robustify(chi2(), rho);
      else
        robust = false;
    }
  }

  virtual MatrixXd getHessian(int iv, bool robust = true) const {
    const JacobianType& jac = _jacobianOplus[iv];
    Vector3d rho;
    getRho(robust, rho);
    const InformationType & rinfo = robust ? rho[1] * information() : information();
    return jac.transpose() * rinfo * jac;
  }
  virtual MatrixXd getHessianij(int iv, int jv, int8_t exact_mode = (int8_t)kExactRobust) const {
    if ((int8_t)kNotExact == exact_mode) {
      bool btranspose = false;
      if (iv > jv) {//keep same order to get idx as buildStructure in block_solver.hpp
        std::swap(iv, jv);
        btranspose = true;
      } else if (iv == jv) {
        assert("wrong usage, iv==jv && kNotExact, please get A() from vertex");
        return MatrixXd();
      }
      int idx = internal::computeUpperTriangleIndex(iv, jv);
      assert(idx < (int)_hessian.size());
      const HessianHelper& h = _hessian[idx];
      btranspose = btranspose ^ h.transposed;
      if (btranspose) {
          return h.matrix.transpose();
      } else {
        return MatrixXd(h.matrix);
      }
    } else {
      const JacobianType& jaci = _jacobianOplus[iv];
      const JacobianType& jacj = _jacobianOplus[jv];
      Vector3d rho;
      bool robust = (int8_t)kExactRobust == exact_mode;
      getRho(robust, rho);
      const InformationType& rinfo = robust ? rho[1] * information() : information();
      return jaci.transpose() * rinfo * jacj;
    }
  }
};

/**
 * \brief template Vertex for VertexScale
 */
class VertexScale : public BaseVertex<1, double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  virtual bool read(std::istream& is) {
    is >> _estimate;
    return true;
  }

  bool write(std::ostream& os) const {
    os << estimate() << " ";
    return os.good();
  }

  void setToOriginImpl() { _estimate = 1; }  // default scale_truth_from_currentworld=1
  void oplusImpl(const double* update_) { _estimate += *update_; }
};

/**
 * \brief template Vertex for VertexNavStatePR,VertexNavStateV,VertexNavStatePVR
 */
template <int DV>
bool readVertex(std::istream& is, Matrix<double, DV, 1>& _estimate) {
  for (int i = 0; i < DV; i++) is >> _estimate[i];
  return true;  // is.good()
}
template <int DV>
bool writeVertex(std::ostream& os, const Matrix<double, DV, 1>& estimate) {
  Vector3d lv = estimate;
  for (int i = 0; i < DV; i++) os << estimate[i] << " ";
  return os.good();
}
template <int D>
class VertexNavState : public BaseVertex<D, NavState> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  virtual bool read(std::istream& is) { return true; }

  virtual bool write(std::ostream& os) const { return true; }

  void setToOriginImpl() { this->_estimate = NavState(); }  // virtual
  void oplusImpl(const double* update_) {
    Eigen::Map<const Matrix<double, D, 1> > update(update_);
    this->_estimate.template IncSmall<D>(update);
  }  // virtual
};

typedef VertexNavState<6> VertexNavStatePR;
typedef VertexNavState<3> VertexNavStateV;
typedef VertexNavState<9> VertexNavStatePVR;

// have to define Bias independently for it has the same dimension with PR
class VertexNavStateBias : public BaseVertex<6, NavState> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  virtual bool read(std::istream& is) { return true; }

  virtual bool write(std::ostream& os) const { return true; }

  void setToOriginImpl() { this->_estimate = NavState(); }  // virtual
  void oplusImpl(const double* update_) {
    Eigen::Map<const Matrix<double, 6, 1> > update(update_);
    this->_estimate.IncSmallBias(update);
  }  // virtual
};

/**
 * \brief template for EdgeProjectXYZOnlyPose(unary edge, mono/stereo)
 */
template <int DE>
bool readEdge(std::istream& is, Matrix<double, DE, 1>& _measurement, Matrix<double, DE, DE>& information) {
  for (int i = 0; i < DE; ++i) is >> _measurement[i];
  for (int i = 0; i < DE; ++i)
    for (int j = i; j < DE; ++j) {
      is >> information(i, j);
      if (i != j) information(j, i) = information(i, j);
    }
  return true;  // is.good();
}
template <int DE>
bool writeEdge(std::ostream& os, const Matrix<double, DE, 1>& measurement, const Matrix<double, DE, DE>& information) {
  for (int i = 0; i < DE; ++i) os << measurement[i] << " ";
  for (int i = 0; i <= 2; i++)
    for (int j = i; j <= 2; j++) os << " " << information(i, j);
  return os.good();
}
template <int DE, int DV>
class EdgeNavStateProjectXYZOnlyPose : public BaseUnaryEdgeEx<DE, Matrix<double, DE, 1>, VertexNavState<DV> > {
  Matrix<double, DE, 1> cam_project(const Vector3d& trans_xyz) const {
    const float invz = 1.0f / trans_xyz[2];  // normalize
    Matrix<double, DE, 1> res;
    res[0] = trans_xyz[0] * invz * fx + cx;
    res[1] = trans_xyz[1] * invz * fy + cy;
    if (DE == 3) res[2] = res[0] - bf * invz;  // ur=ul-b*fx/dl or u in right image
    return res;
  }

  typedef BaseUnaryEdge<DE, Matrix<double, DE, 1>, VertexNavState<DV> > Base;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool read(std::istream& is) { return readEdge<DE>(is, this->_measurement, this->information()); }
  bool write(std::ostream& os) const { return writeEdge<DE>(os, this->measurement(), this->information()); }
  virtual void computeError() {
    const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[0]);  // Tbw
    const NavState& ns = vNS->estimate();  // transform Xw to Xc through Tbw&&Tcb
    this->_error =
        this->_measurement -
        cam_project(Rcb * ns.getRwb().transpose() * (Pw - ns.mpwb) +
                    tcb);  // Pc=Tcb*Tbw*Pw=Rcb*Rbw*Pw+Rcb*tbw(-Rcb*Rbw*twb)+tcb(-Rcb*tbc)=Rcb*Rbw*(Pw-twb)+tcb;
  }
  virtual void linearizeOplus();

  void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_, const Matrix3d& Rcb_,
                 const Vector3d& tcb_, const Vector3d& Pw_,
                 const float* bf_ = NULL) {  // if u use const double*, u have to do some extra work
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
    Rcb = Rcb_;
    tcb = tcb_;
    Pw = Pw_;
    if (bf_ != NULL) bf = *bf_;
  }
  bool isDepthPositive() {  // unused in IMU motion-only BA, but used in localBA&GBA
    const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[0]);  // Tbw
    const NavState& ns = vNS->estimate();
    return (Rcb * ns.getRwb().transpose() * (Pw - ns.mpwb) + tcb)(2) > 0.0;  // Xc.z>0
  }

 protected:
  double fx, fy, cx, cy, bf;  // Camera intrinsics
  Matrix3d Rcb;
  Vector3d tcb;  // Camera-IMU extrinsics
  Vector3d Pw;   // Point position in world frame

  using Base::_jacobianOplusXi;
  using Base::_vertices;
};
template <int DE, int DV>
void EdgeNavStateProjectXYZOnlyPose<DE, DV>::linearizeOplus() {
  const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[0]);
  const NavState& ns = vNS->estimate();
  Matrix3d Rwb = ns.getRwb();

  Vector3d Pc = Rcb * Rwb.transpose() * (Pw - ns.mpwb) + tcb;  // Pc=Rcb*Rbw*(Pw-twb)+tcb
  double x = Pc[0], y = Pc[1], invz = 1 / Pc[2], invz_2 = invz * invz;

  // Jacobian of camera projection, par((K*Pc)(0:1))/par(Pc)=J_e_Pc, error = obs - pi( Pc )
  Matrix<double, DE, 3> Jproj;  // J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2], here Xc->Xc+dXc
  Jproj.template block<2, 3>(0, 0) << -fx * invz, 0, x * fx * invz_2, 0, -fy * invz, y * fy * invz_2;
  if (DE > 2)
    Jproj.template block<1, 3>(2, 0) << Jproj(0, 0), 0,
        Jproj(0, 2) - bf * invz_2;  // ur=ul-b*fx/dl,dl=z => J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2; fx/z 0
                                    // -fx*x/z^2+bf/z^2]

  // Jacobian of error w.r.t dPwb = JdPwb=J_e_Pc*J_Pc_dPwb, notice we use pwb->pwb+dpwb increment model in the
  // corresponding Vertex, so here is the same, a bit dfferent from (21)
  //   Matrix<double,DE,3> JdPwb=Jproj*(-Rcb);//J_Pc_dPwb = -Rcw*Rwb= -Rcb(p<-p+R*dp)
  Matrix<double, DE, 3> JdPwb = Jproj * (-Rcb * Rwb.transpose());  // J_Pc_dPwb = -Rcw(p<-p+dp)

  // Jacobian of error w.r.t dRwb
  Vector3d Paux = Rcb * Rwb.transpose() *
                  (Pw - ns.mpwb);  // J_Pc_dRwb=(Rcw*(Pw-twb))^Rcb, using right disturbance model/Rwb->Rwb*Exp(dphi) or
                                   // Rbw->Exp(-dphi)*Rbw, see Manifold paper (20)
  Matrix<double, DE, 3> JdRwb = Jproj * (Sophus::SO3exd::hat(Paux) * Rcb);

  // Jacobian of error w.r.t NavStatePR, order in 'update_': dP, dPhi
  Matrix<double, DE, DV> JNavState = Matrix<double, DE, DV>::Zero();
  JNavState.template block<DE, 3>(0, 0) = JdPwb;       // J_error_dnotPR=0 so we'd better use PR&V instead of PVR/PVRB
  JNavState.template block<DE, 3>(0, DV - 3) = JdRwb;  // only for 9(J_e_dV=0)/6
  _jacobianOplusXi = JNavState;
}

typedef EdgeNavStateProjectXYZOnlyPose<2, 6> EdgeNavStatePRPointXYZOnlyPose;
typedef EdgeNavStateProjectXYZOnlyPose<3, 6> EdgeStereoNavStatePRPointXYZOnlyPose;
typedef EdgeNavStateProjectXYZOnlyPose<2, 9> EdgeNavStatePVRPointXYZOnlyPose;
typedef EdgeNavStateProjectXYZOnlyPose<3, 9> EdgeStereoNavStatePVRPointXYZOnlyPose;

#ifdef USE_G2O_NEWEST
typedef VertexPointXYZ VertexSBAPointXYZ;  // for their oplusImpl&&setToOriginImpl is the same/normal
#endif

/**
 * \brief template for EdgeProjectXYZ(binary edge, mono/stereo), similar to EdgeProjectXYZOnlyPose(change the parameter
 * Pw to optimized vertex _vertices[0] & Tbw to _vertices[1])
 */
template <int DE, int DV>
class EdgeNavStateProjectXYZ
    : public BaseBinaryEdge<DE, Matrix<double, DE, 1>, VertexSBAPointXYZ, VertexNavState<DV> > {
  Matrix<double, DE, 1> cam_project(const Vector3d& trans_xyz) const {
    const float invz = 1.0f / trans_xyz[2];  // normalize
    Matrix<double, DE, 1> res;
    res[0] = trans_xyz[0] * invz * fx + cx;
    res[1] = trans_xyz[1] * invz * fy + cy;
    if (DE == 3) res[2] = res[0] - bf * invz;  // ur=ul-b*fx/dl or u in right image
    return res;
  }

  typedef BaseBinaryEdge<DE, Matrix<double, DE, 1>, VertexSBAPointXYZ, VertexNavState<DV> > Base;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool read(std::istream& is) { return readEdge<DE>(is, this->_measurement, this->information()); }
  bool write(std::ostream& os) const { return writeEdge<DE>(os, this->measurement(), this->information()); }
  virtual void computeError() {
    const VertexSBAPointXYZ* pXw = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);    // Xw/Pw
    const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);  // Tbw
    const NavState& ns = vNS->estimate();  // transform Xw to Xc through Tbw&&Tcb
    this->_error =
        this->_measurement -
        cam_project(Rcb * ns.getRwb().transpose() * (pXw->estimate() - ns.mpwb) +
                    tcb);  // Pc=Tcb*Tbw*Pw=Rcb*Rbw*Pw+Rcb*tbw(-Rcb*Rbw*twb)+tcb(-Rcb*tbc)=Rcb*Rbw*(Pw-twb)+tcb;
  }
  virtual void linearizeOplus();

  void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_, const Matrix3d& Rcb_,
                 const Vector3d& tcb_, const float* bf_ = NULL) {
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
    Rcb = Rcb_;
    tcb = tcb_;
    if (bf_ != NULL) bf = *bf_;
  }
  bool isDepthPositive() {  // unused in IMU motion-only BA, but used in localBA&GBA
    const VertexSBAPointXYZ* pXw = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);    // Xw/Pw
    const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);  // Tbw
    const NavState& ns = vNS->estimate();
    return (Rcb * ns.getRwb().transpose() * (pXw->estimate() - ns.mpwb) + tcb)(2) > 0.0;  // Xc.z>0
  }

 protected:
  double fx, fy, cx, cy, bf;  // Camera intrinsics
  Matrix3d Rcb;
  Vector3d tcb;  // Camera-IMU extrinsics

  using Base::_jacobianOplusXi;
  using Base::_jacobianOplusXj;
  using Base::_vertices;
};
template <int DE, int DV>
void EdgeNavStateProjectXYZ<DE, DV>::linearizeOplus() {
  const VertexSBAPointXYZ* pXw = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);  // Xw/Pw
  const Vector3d& Pw = pXw->estimate();
  const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);  // Tbw
  const NavState& ns = vNS->estimate();
  Matrix3d Rwb = ns.getRwb();

  Vector3d Pc = Rcb * ns.getRwb().transpose() * (Pw - ns.mpwb) + tcb;  // Pc=Rcb*Rbw*(Pw-twb)+tcb
  double x = Pc[0], y = Pc[1], invz = 1 / Pc[2], invz_2 = invz * invz;

  // Jacobian of camera projection, par((K*Pc)(0:1))/par(Pc)=J_e_Pc, error = obs - pi( Pc )
  Matrix<double, DE, 3> Jproj;  // J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2], here Xc->Xc+dXc
  Jproj.template block<2, 3>(0, 0) << -fx * invz, 0, x * fx * invz_2, 0, -fy * invz, y * fy * invz_2;
  if (DE > 2)
    Jproj.template block<1, 3>(2, 0) << Jproj(0, 0), 0,
        Jproj(0, 2) - bf * invz_2;  // ur=ul-b*fx/dl,dl=z => J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2; fx/z 0
                                    // -fx*x/z^2+bf/z^2]

  // Jacobian of error w.r.t dPwb = JdPwb=J_e_Pc*J_Pc_dPwb, notcie we use pwb->pwb+dpwb increment model in the
  // corresponding Vertex, so here is the same, a bit dfferent from (21)
  //   Matrix<double,DE,3> JdPwb=Jproj*(-Rcb);//J_Pc_dPwb = -Rcw*Rwb= -Rcb(p<-p+R*dp)
  Matrix<double, DE, 3> JdPwb = Jproj * (-Rcb * Rwb.transpose());  // J_Pc_dPwb = -Rcw(p<-p+dp)

  // Jacobian of error w.r.t dRwb
  Vector3d Paux = Rcb * Rwb.transpose() *
                  (Pw - ns.mpwb);  // J_Pc_dRwb=(Rcw*(Pw-twb))^Rcb, using right disturbance model/Rwb->Rwb*Exp(dphi) or
                                   // Rbw->Exp(-dphi)*Rbw, see Manifold paper (20)
  Matrix<double, DE, 3> JdRwb = Jproj * (Sophus::SO3exd::hat(Paux) * Rcb);

  // Jacobian of error w.r.t NavStatePR, order in 'update_': dP, dPhi
  Matrix<double, DE, DV> JNavState = Matrix<double, DE, DV>::Zero();
  JNavState.template block<DE, 3>(0, 0) = JdPwb;       // J_error_dnotPR=0 so we'd better use PR&V instead of PVR/PVRB
  JNavState.template block<DE, 3>(0, DV - 3) = JdRwb;  // only for 9(J_e_dV=0)/6
  _jacobianOplusXj = JNavState;

  // Jacobian of error(-pc) w.r.t dXw/dPw: J_e_dXw=JdXw=J_e_Pc*J_Pc_dPw=Jproj*Rcw=-JdPwb
  //   _jacobianOplusXi=-JdPwb*Rwb.transpose();//for (p<-p+R*dp)
  _jacobianOplusXi = -JdPwb;  // Jproj*Rcb*Rwb.transpose(); it's a fast form for (p<-p+dp)
}

typedef EdgeNavStateProjectXYZ<2, 6> EdgeNavStatePRPointXYZ;
typedef EdgeNavStateProjectXYZ<3, 6> EdgeStereoNavStatePRPointXYZ;
typedef EdgeNavStateProjectXYZ<2, 9> EdgeNavStatePVRPointXYZ;
typedef EdgeNavStateProjectXYZ<3, 9> EdgeStereoNavStatePVRPointXYZ;

/**
 * \brief template for EdgeProjectXYZ(trinary edge, mono/stereo), similar to EdgeProjectXYZ(binary edge, just add one
 * VertexScale)
 */
template <int DE, int DV>
class EdgeNavStateProjectXYZWithScale : public BaseMultiEdge<DE, Matrix<double, DE, 1> > {
  Matrix<double, DE, 1> cam_project(const Vector3d& trans_xyz) const {
    const float invz = 1.0f / trans_xyz[2];  // normalize
    Matrix<double, DE, 1> res;
    res[0] = trans_xyz[0] * invz * fx + cx;
    res[1] = trans_xyz[1] * invz * fy + cy;
    if (DE == 3) res[2] = res[0] - bf * invz;  // ur=ul-b*fx/dl or u in right image
    return res;
  }

  typedef BaseMultiEdge<DE, Matrix<double, DE, 1> > Base;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeNavStateProjectXYZWithScale() : Base() { this->resize(3); }
  bool read(std::istream& is) { return readEdge<DE>(is, this->_measurement, this->information()); }
  bool write(std::ostream& os) const { return writeEdge<DE>(os, this->measurement(), this->information()); }
  virtual void computeError() {
    const VertexSBAPointXYZ* pXw = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);    // Xw/Pw
    const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);  // Tbw
    const NavState& ns = vNS->estimate();                                       // transform Xw to Xc through Tbw&&Tcb
    const VertexScale* vScale = static_cast<const VertexScale*>(_vertices[2]);  // Scale
    double scale = vScale->estimate();
    Matrix3d Rcw = Rcb * ns.getRwb().transpose();
    // here we use Xw_truescale=s*Xw_currentscale;twb is nearly true scale at the end of execution, so here we don't use
    // twb_true=s*twc+Rwc*tcb=s*(twb_current-Rwc*tcb)+Rwc*tcb for normal fast speed
    this->_error =
        this->_measurement -
        cam_project(
            Rcw * (scale * pXw->estimate() - ns.mpwb) +
            tcb);  // Pc=Tcb*Tbw*Pw=Rcb*Rbw*Pw+Rcb*tbw(-Rcb*Rbw*twb)+tcb(-Rcb*tbc)=Rcb*Rbw*(Pw_true-twb_true)+tcb;
  }
  virtual void linearizeOplus();

  void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_, const Matrix3d& Rcb_,
                 const Vector3d& tcb_, const float* bf_ = NULL) {
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
    Rcb = Rcb_;
    tcb = tcb_;
    if (bf_ != NULL) bf = *bf_;
  }
  bool isDepthPositive() {  // unused in IMU motion-only BA, but used in localBA&GBA
    const VertexSBAPointXYZ* pXw = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);    // Xw/Pw
    const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);  // Tbw
    const NavState& ns = vNS->estimate();
    const VertexScale* vScale = static_cast<const VertexScale*>(_vertices[2]);  // Scale
    double scale = vScale->estimate();
    Matrix3d Rcw = Rcb * ns.getRwb().transpose();
    return (Rcw * (scale * pXw->estimate() - ns.mpwb) + tcb)(2) > 0.0;  // Xc.z>0
  }

 protected:
  double fx, fy, cx, cy, bf;  // Camera intrinsics
  Matrix3d Rcb;
  Vector3d tcb;  // Camera-IMU extrinsics

  using Base::_jacobianOplus;
  using Base::_vertices;
};
template <int DE, int DV>
void EdgeNavStateProjectXYZWithScale<DE, DV>::linearizeOplus() {
  const VertexSBAPointXYZ* pXw = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);  // Xw/Pw
  const Vector3d& Pw = pXw->estimate();
  const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);  // Tbw
  const NavState& ns = vNS->estimate();
  Matrix3d Rwb = ns.getRwb();
  const VertexScale* vScale = static_cast<const VertexScale*>(_vertices[2]);  // Scale
  double scale = vScale->estimate();

  Matrix3d Rcw = Rcb * Rwb.transpose();
  Vector3d Pc = Rcw * (scale * Pw - ns.mpwb) + tcb;  // Pc=Rcb*Rbw*(Pw_true-twb_true)+tcb
  double x = Pc[0], y = Pc[1], invz = 1 / Pc[2], invz_2 = invz * invz;

  // Jacobian of camera projection, par((K*Pc)(0:1))/par(Pc)=J_e_Pc, error = obs - pi( Pc )
  Matrix<double, DE, 3> Jproj;  // J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2], here Xc->Xc+dXc
  Jproj.template block<2, 3>(0, 0) << -fx * invz, 0, x * fx * invz_2, 0, -fy * invz, y * fy * invz_2;
  if (DE > 2)
    Jproj.template block<1, 3>(2, 0) << Jproj(0, 0), 0,
        Jproj(0, 2) - bf * invz_2;  // ur=ul-b*fx/dl,dl=z => J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2; fx/z 0
                                    // -fx*x/z^2+bf/z^2]

  // Jacobian of error w.r.t dPwb = JdPwb=J_e_Pc*J_Pc_dPwb, notcie we use pwb->pwb+dpwb increment model in the
  // corresponding Vertex, so here is the same, a bit dfferent from (21)
  //   Matrix<double,DE,3> JdPwb=Jproj*(-Rcb);//J_Pc_dPwb = -Rcw*Rwb= -Rcb(p<-p+R*dp)
  Matrix<double, DE, 3> JdPwb = Jproj * (-Rcw);  // J_Pc_dPwb = -Rcw(p<-p+dp)
  // Jacobian of error w.r.t dRwb
  Vector3d Paux = Rcb * Rwb.transpose() *
                  (scale * Pw - ns.mpwb);  // J_Pc_dRwb=(Rcw*(s*Pw-twb))^Rcb, using right disturbance
                                           // model/Rwb->Rwb*Exp(dphi) or Rbw->Exp(-dphi)*Rbw, see Manifold paper (20)
  Matrix<double, DE, 3> JdRwb = Jproj * (Sophus::SO3exd::hat(Paux) * Rcb);

  // Jacobian of error w.r.t NavStatePR, order in 'update_': dP, dPhi
  Matrix<double, DE, DV> JNavState = Matrix<double, DE, DV>::Zero();
  JNavState.template block<DE, 3>(0, 0) = JdPwb;       // J_error_dnotPR=0 so we'd better use PR&V instead of PVR/PVRB
  JNavState.template block<DE, 3>(0, DV - 3) = JdRwb;  // only for 9(J_e_dV=0)/6
  _jacobianOplus[1] = JNavState;

  // Jacobian of error(-pc) w.r.t dXw/dPw: J_e_dXw=JdXw=J_e_Pc*J_Pc_dPw=Jproj*Rcw*scale=-JdPwb*scale
  _jacobianOplus[0] = JdPwb * (-scale);  // Jproj*Rcw*scale; Pw<-Pw+dPw, Pw_true=sPw

  // Jacobian of error(-pc) w.r.t ds: J_e_ds=Jds=J_e_Pc*J_Pc_ds
  _jacobianOplus[2] = Jproj * Rcw * Pw;  // J_Pc_ds=Rcw*Pw, easy to prove
}

typedef EdgeNavStateProjectXYZWithScale<2, 6> EdgeNavStatePRSPointXYZ;
typedef EdgeNavStateProjectXYZWithScale<3, 6> EdgeStereoNavStatePRSPointXYZ;
// typedef EdgeNavStateProjectXYZWithScale<2,6,true> EdgeNavStatePRSPointXYZStable;

/**
 * \brief template for EdgeEnc(binary edge)
 */
class EdgeEnc : public BaseBinaryEdge<6, Vector6d, VertexSE3Expmap, VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }

  void computeError();
  virtual void linearizeOplus();

  Quaterniond qRce;
  Vector3d pce;

 protected:
};
template <int DV>
class EdgeEncNavState : public BaseBinaryEdgeEx<6, Vector6d, VertexNavState<DV>, VertexNavState<DV> > {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }

  void computeError();
  virtual void linearizeOplus();

  Quaterniond qRbe;
  Vector3d pbe;

 protected:
};
template <int DV>
void EdgeEncNavState<DV>::computeError() {
  const VertexNavState<DV>* vi = static_cast<const VertexNavState<DV>*>(this->_vertices[0]);
  const VertexNavState<DV>* vj = static_cast<const VertexNavState<DV>*>(this->_vertices[1]);
  const NavState &nsi = vi->estimate(), &nsj = vj->estimate();
  Quaterniond qRiw = nsi.mRwb.inverse().unit_quaternion(), qRwj = nsj.mRwb.unit_quaternion();
  Vector3d pwi = nsi.mpwb, pwj = nsj.mpwb;
  Sophus::SO3exd so3Reiej = Sophus::SO3exd(qRbe.conjugate() * qRiw * qRwj * qRbe);
  this->_error.template segment<3>(0) =
      Sophus::SO3exd(Sophus::SO3exd::exp(this->_measurement.template segment<3>(0)).inverse() * so3Reiej)
          .log();  // Log(delta~Rij.t()*Reiej)
  Vector3d deltapij = qRbe.conjugate() *
                      (qRiw * (pwj - pwi) - pbe + qRiw * qRwj * pbe);  // Reb*[Rbiw*(pwbj-pwbi)-pbe+Rbiw*Rbjw.t()*pbe]
  this->_error.template segment<3>(3) = deltapij - this->_measurement.template segment<3>(3);  // deltapij-delta~pij
}
template <int DV>
void EdgeEncNavState<DV>::linearizeOplus() {
  const VertexNavState<DV>* vi = static_cast<const VertexNavState<DV>*>(this->_vertices[0]);
  const VertexNavState<DV>* vj = static_cast<const VertexNavState<DV>*>(this->_vertices[1]);
  const NavState &nsi = vi->estimate(), &nsj = vj->estimate();
  Quaterniond qRiw = nsi.mRwb.inverse().unit_quaternion(), qRwj = nsj.mRwb.unit_quaternion();
  Vector3d pwi = nsi.mpwb, pwj = nsj.mpwb;

  // calculate Je_dxi xi=ksi=(phii,rhoi)
  Matrix3d O3x3 = Matrix3d::Zero();  // JeR_dpi/j=0
  Matrix3d Reb = qRbe.conjugate().toRotationMatrix();
  Quaterniond qRij = qRiw * qRwj;
  Vector3d eR = this->_error.template segment<3>(0);
  Matrix3d JeR_dphii = -Sophus::SO3exd::JacobianRInv(eR) * Reb *
                       qRij.conjugate().toRotationMatrix();  // JeR_dphii=-Jrinv(eR)*(Rbiw*Rwbj*Rbe).t()
  Matrix3d RebRiw = (qRbe.conjugate() * qRiw).toRotationMatrix();
  Matrix3d Jep_dpi = -RebRiw;  // Jep_dpi=-Reb*Rbiw
  Matrix3d Jep_dphii =
      Reb * Sophus::SO3exd::hat(qRij * pbe + qRiw * (pwj - pwi));  // Jep_dphii=Reb*[Rbiw*(Rwbj*pbe+pwbj-pwbi)]^
  // calculate Je_dxj xj=ksj=(phij,rhoj)
  Matrix3d JeR_dphij = Sophus::SO3exd::JacobianRInv(eR) * Reb;  // JeR_dphij=Jrinv(eR)*Reb
  Matrix3d Jep_dpj = RebRiw;                                    // Jep_dpj=Reb*Rbiw
  Matrix3d Jep_dphij =
      -(qRbe.conjugate() * qRij).toRotationMatrix() * Sophus::SO3exd::hat(pbe);  // Jep_dphij=-Reb*Rbiw*Rwbj*pbe^

  if (DV == 9) {
    this->_jacobianOplusXi.setZero();  // Je_dVi/j=0
    this->_jacobianOplusXj.setZero();
  }
  int idR = DV == 6 ? 3 : 6;
  this->_jacobianOplusXi.template block<3, 3>(0, 0) = O3x3;
  this->_jacobianOplusXi.template block<3, 3>(3, 0) = Jep_dpi;
  this->_jacobianOplusXi.template block<3, 3>(0, idR) = JeR_dphii;
  this->_jacobianOplusXi.template block<3, 3>(3, idR) = Jep_dphii;
  this->_jacobianOplusXj.template block<3, 3>(0, 0) = O3x3;
  this->_jacobianOplusXj.template block<3, 3>(3, 0) = Jep_dpj;
  this->_jacobianOplusXj.template block<3, 3>(0, idR) = JeR_dphij;
  this->_jacobianOplusXj.template block<3, 3>(3, idR) = Jep_dphij;
}

typedef EdgeEncNavState<6> EdgeEncNavStatePR;
typedef EdgeEncNavState<9> EdgeEncNavStatePVR;

/**
 * \brief template for EdgeNavStateI(multi edge)
 */
template <int NV>
class EdgeNavStateI : public BaseMultiEdgeEx<9, IMUPreintegrator> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeNavStateI() : BaseMultiEdgeEx<9, IMUPreintegrator>() { resize(NV); }

  bool read(std::istream& is) { return true; }

  bool write(std::ostream& os) const { return true; }

  void computeError();

  virtual void linearizeOplus();

  void SetParams(const Vector3d& gw_) { gw = gw_; }

 protected:
  Vector3d gw;  // gw: Gravity vector in 'world' frame
  typedef VertexNavState<(NV >= 5 ? 6 : 9)>
      VertexNavStateNV;  // NV==5/6 -> <6>PR NV==3 -> <9>PVR, this is very important typedef!
};
template <int NV>
void EdgeNavStateI<NV>::computeError() {
  const VertexNavStateNV* vPRi = static_cast<const VertexNavStateNV*>(_vertices[0]);
  const VertexNavStateNV* vPRj = static_cast<const VertexNavStateNV*>(_vertices[1]);
  const NavState &nsPRi = vPRi->estimate(), &nsPRj = vPRj->estimate(), *pBiasi;
  const Sophus::SO3exd RiT = nsPRi.mRwb.inverse();
  // get vi,vj,dbi in PRV/PVR situation
  Vector3d vi, vj, dbgi, dbai;
  if (NV >= 5) {                                                              // PRV
    vi = static_cast<const VertexNavStateV*>(_vertices[2])->estimate().mvwb;  // vwbi
    vj = static_cast<const VertexNavStateV*>(_vertices[3])->estimate().mvwb;  // vwbj
    pBiasi = &static_cast<const VertexNavStateBias*>(_vertices[4])->estimate();
  } else {  // PVR
    vi = nsPRi.mvwb;
    vj = nsPRj.mvwb;  // nsPVRi/j
    pBiasi = &static_cast<const VertexNavStateBias*>(_vertices[2])->estimate();
  }
  dbgi = pBiasi->mdbg;
  dbai = pBiasi->mdba;
  double deltat = _measurement.mdeltatij;  // deltatij

  // see VIORBSLAM paper (6)/ Manifold (45)
  //  r_deltapij/ep=Rbiw*(pwbj-pwbi-vwbi*deltatij-1/2*gw*deltatij^2)-
  //  (deltapij+J_g_deltap*dbgi+J_a_deltap*dbai),here deltapij=delta~pij(bi_bar)
  _error.segment<3>(0) = RiT * (nsPRj.mpwb - nsPRi.mpwb - vi * deltat - gw * (deltat * deltat / 2)) -
                         (_measurement.mpij + _measurement.mJgpij * dbgi + _measurement.mJapij * dbai);
  int idR = (NV >= 5) ? 3 : 6;  // if NV==5 then error_PRV else ePVR
  _error.segment<3>(idR) =
      ((Sophus::SO3exd(Sophus::SO3exd(_measurement.mRij)) * Sophus::SO3exd::exp(_measurement.mJgRij * dbgi)).inverse() *
       RiT * nsPRj.mRwb)
          .log();                                               // eR=Log((deltaRij*Exp(Jg_deltaR*dbgi)).t()*Rbiw*Rwbj)
  _error.segment<3>(9 - idR) = RiT * (vj - vi - gw * deltat) -  // ev=Rwbi.t()*(vwbj-vwbi-gw*deltatij)-
                               (_measurement.mvij + _measurement.mJgvij * dbgi +
                                _measurement.mJavij * dbai);  //(deltavij+J_g_deltav*dbgi+J_a_deltav*dbai)
}
template <int NV>
void EdgeNavStateI<NV>::linearizeOplus() {
  const VertexNavStateNV* vPRi = static_cast<const VertexNavStateNV*>(_vertices[0]);
  const VertexNavStateNV* vPRj = static_cast<const VertexNavStateNV*>(_vertices[1]);
  const NavState &nsPRi = vPRi->estimate(), &nsPRj = vPRj->estimate(), *pBiasi;
  const Vector3d &pi = nsPRi.mpwb, &pj = nsPRj.mpwb;  // here is regarded as true scaled pwb
  const Matrix3d RiT = nsPRi.getRwb().transpose();
  Vector3d vi, vj, dbgi;
  if (NV >= 5) {                                                              // PRV
    vi = static_cast<const VertexNavStateV*>(_vertices[2])->estimate().mvwb;  // vwbi
    vj = static_cast<const VertexNavStateV*>(_vertices[3])->estimate().mvwb;  // vwbj
    pBiasi = &static_cast<const VertexNavStateBias*>(_vertices[4])->estimate();
  } else {  // PVR
    vi = nsPRi.mvwb;
    vj = nsPRj.mvwb;  // nsPVRi/j
    pBiasi = &static_cast<const VertexNavStateBias*>(_vertices[2])->estimate();
  }
  dbgi = pBiasi->mdbg;
  double deltat = _measurement.mdeltatij;  // deltatij
  // see Manifold (74)~(81)
  Matrix<double, 9, 6> JBiasi;
  Matrix3d O3x3 = Matrix3d::Zero();
  int idR, idV;
  Matrix<double, 9, 9> JPRVi, JPRVj;  // when NV==3 it's JPVRi,JPVRj
  if (NV >= 5) {                      // J_ePRV_PRi,PRj,Vi,Vj,Bi
    idR = 3;
    idV = 6;
  } else {  // J_ePVR_PVRi,PVRj,Bi
    idR = 6;
    idV = 3;  // please notice everything after meaning row/col should use idR/idV except bias term!
  }
  Vector3d eR = _error.segment<3>(idR);  // r_deltaRij/eR/r_Phiij
  // J_rpij_dxi
  JPRVi.block<3, 3>(0, idR) =
      Sophus::SO3exd::hat(RiT * (pj - pi - vi * deltat - gw * (deltat * deltat / 2)));  // J_rpij_dPhi_i
  //   JPRVi.block<3,3>(0,0)=-Matrix3d::Identity();//J_rpij_dpi=-I3x3, notice here use pi<-pi+Ri*dpi
  // J_rpij_dpi, notice here use pi<-pi+dpi not the form pi<-pi+Ri*dpi in the paper!
  JPRVi.block<3, 3>(0, 0) = -RiT;
  JPRVi.block<3, 3>(0, idV) = -RiT * deltat;        // J_rpij_dvi
  JBiasi.block<3, 3>(0, 0) = -_measurement.mJgpij;  // J_rpij_ddbgi
  JBiasi.block<3, 3>(0, 3) = -_measurement.mJapij;  // J_rpij_ddbai
  // J_rpij_dxj
  JPRVj.block<3, 3>(0, idR) = O3x3;  // J_rpij_dPhi_j
  //   JPRVj.block<3,3>(0,0)=RiT*nsPRj.getRwb();//J_rpij_dpj=Ri.t()*Rj, notice here use pj<-pj+Rj*dpj
  // J_rpij_dpj, notice here use pj<-pj+dpj not the form pj<-pj+Rj*dpj in the paper!
  JPRVj.block<3, 3>(0, 0) = RiT;
  JPRVj.block<3, 3>(0, idV) = O3x3;  // J_rpij_dvj
  // J_rvij_dxi
  JPRVi.block<3, 3>(idV, idR) = Sophus::SO3exd::hat(RiT * (vj - vi - gw * deltat));  // J_rvij_dPhi_i
  JPRVi.block<3, 3>(idV, 0) = O3x3;                   // J_rvij_dpi(pi<-pi+dpi), also(pi<-pi+Ri*dpi)
  JPRVi.block<3, 3>(idV, idV) = -RiT;                 // J_rvij_dvi
  JBiasi.block<3, 3>(idV, 0) = -_measurement.mJgvij;  // J_rvij_ddbgi
  JBiasi.block<3, 3>(idV, 3) = -_measurement.mJavij;  // J_rvij_ddbai
  // J_rvij_dxj
  JPRVj.block<3, 3>(idV, idR) = O3x3;  // J_rvij_dPhi_j
  JPRVj.block<3, 3>(idV, 0) = O3x3;    // J_rvij_dpj(pj<-pj+dpj), also(pj<-pj+Rj*dpj)
  JPRVj.block<3, 3>(idV, idV) = RiT;   // J_rvij_dvj
  // J_rRij_dxi
  Matrix3d Jrinv = Sophus::SO3exd::JacobianRInv(eR);  // JrInv_rPhi/Jrinv_rdeltaRij/Jrinv_eR
  JPRVi.block<3, 3>(idR, idR) = -Jrinv * (nsPRj.mRwb.inverse() * nsPRi.mRwb).matrix();  // J_rRij_dPhi_i
  JPRVi.block<3, 3>(idR, 0) = O3x3;    // J_rRij_dpi(pi<-pi+dpi), also(pi<-pi+Ri*dpi)
  JPRVi.block<3, 3>(idR, idV) = O3x3;  // J_rRij_dvi
  // right is Exp(rdeltaRij).t(), same as Sophus::SO3exd::exp(rPhiij).inverse().matrix()
  //  J_rRij_ddbgi, notice Jr_b=Jr(Jg_deltaR*dbgi)
  JBiasi.block<3, 3>(idR, 0) = -Jrinv * IMUPreintegrator::Expmap(-eR) *
                               Sophus::SO3exd::JacobianR(_measurement.mJgRij * dbgi) * _measurement.mJgRij;
  JBiasi.block<3, 3>(idR, 3) = O3x3;  // J_rRij_ddbai
  // J_rRij_dxj
  JPRVj.block<3, 3>(idR, idR) = Jrinv;  // J_rRij_dPhi_j
  JPRVj.block<3, 3>(idR, 0) = O3x3;     // J_rRij_dpj(pj<-pj+dpj), also(pj<-pj+Rj*dpj)
  JPRVj.block<3, 3>(idR, idV) = O3x3;   // J_rRij_dvj

  if (NV >= 5) {  // J_ePRV_PRi,PRj,Vi,Vj,Bi = 9*24
    _jacobianOplus[0] = JPRVi.block<9, 6>(0, 0);
    _jacobianOplus[1] = JPRVj.block<9, 6>(0, 0);
    _jacobianOplus[2] = JPRVi.block<9, 3>(0, 6);
    _jacobianOplus[3] = JPRVj.block<9, 3>(0, 6);
    _jacobianOplus[4] = JBiasi;  // 9*6
  } else {                       // J_ePVR_PVRi,PVRj,Bi
    _jacobianOplus[0] = JPRVi;
    _jacobianOplus[1] = JPRVj;  // when NV==3 it's JPVRi,JPVRj, 9*9,9*9
    _jacobianOplus[2] = JBiasi;
  }
}

typedef EdgeNavStateI<5> EdgeNavStatePRV;  // PRi, PRj, Vi, Vj, Bi, total 5 vertices
typedef EdgeNavStateI<3> EdgeNavStatePVR;  // PVRi, PVRj, Bi, total 3 vertices
// typedef EdgeNavStateI<6> EdgeNavStatePRVBS;//add scale optimization variable(for unscaled pwb)
// typedef EdgeNavStateI<7> EdgeNavStatePRVBSV;//add scale optimization variable(for unscaled pwb & vwb)

/**
 * \brief EdgeNavStateBias(binary edge),EdgeNavStatePriorPRVBias,EdgeNavStatePriorPVRBias
 */
class EdgeNavStateBias : public BaseBinaryEdgeEx<6, IMUPreintegrator, VertexNavStateBias, VertexNavStateBias> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  bool read(std::istream& is) { return true; }

  bool write(std::ostream& os) const { return true; }

  void computeError();

  virtual void linearizeOplus();
};  // v0 is Bi, v1 is Bj
class EdgeNavStatePriorPRVBias : public BaseMultiEdge<15, NavState> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeNavStatePriorPRVBias() : BaseMultiEdge<15, NavState>() { resize(3); }

  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }

  void computeError();
  virtual void linearizeOplus();
};
class EdgeNavStatePriorPVRBias : public BaseBinaryEdgeEx<15, NavState, VertexNavStatePVR, VertexNavStateBias> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // default constructor is enough
  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }

  void computeError();
  virtual void linearizeOplus();
};

/**
 * @brief The VertexGyrBias class
 * For gyroscope bias compuation in Visual-Inertial initialization
 */
typedef VertexSBAPointXYZ VertexGyrBias;  // for their oplusImpl&&setToOriginImpl is the same/normal

/**
 * @brief The EdgeGyrBias class
 * For gyroscope bias compuation in Visual-Inertial initialization
 */
class EdgeGyrBias : public BaseUnaryEdge<3, Vector3d, VertexGyrBias> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // default constructor is enough
  bool read(std::istream& is) { return readEdge<3>(is, this->_measurement, this->information()); }
  bool write(std::ostream& os) const { return writeEdge<3>(os, this->measurement(), this->information()); }

  Matrix3d deltaRij;    // or deltaRi_i+1
  Matrix3d JgRij;       // Jg_deltaR
  Matrix3d Rwbi, Rwbj;  // or Rwbi+1

  void computeError() {  // part of EdgeNavStateI
    const VertexGyrBias* v = static_cast<const VertexGyrBias*>(_vertices[0]);
    Vector3d bg = v->estimate();                           // bgi, here we think bg_=0, dbgi=bgi, see VIORBSLAM IV-A
    Matrix3d dRbg = IMUPreintegrator::Expmap(JgRij * bg);  // right dR caused by dbgi
    Sophus::SO3exd errR((deltaRij * dRbg).transpose() * Rwbi.transpose() * Rwbj);  // deltaRij^T * Riw * Rwj
    _error = errR.log();  // eR=Log((deltaRij*Exp(Jg_deltaR*dbgi)).t()*Rbiw*Rwbj), _error.segment<3>(0) is _error itself
  }
  virtual void linearizeOplus() {  // I think JingWang may be wrong here?both not best, JW avoids large bg's influence
                                   // but lose variance on Jac(could be used in low freq/cov IMU)
    const VertexGyrBias* v = static_cast<const VertexGyrBias*>(_vertices[0]);
    Vector3d bg = v->estimate();
    Matrix3d Jrinv = Sophus::SO3exd::JacobianRInv(_error);  // JrInv_rPhi/Jrinv_rdeltaRij/Jrinv_eR
    _jacobianOplusXi =
        -Jrinv *
        IMUPreintegrator::Expmap(
            -_error) *  // right is Exp(rdeltaRij).t(), same as Sophus::SO3exd::exp(rPhiij).inverse().matrix()
        Sophus::SO3exd::JacobianR(JgRij * bg) *
        JgRij;  // J_rRij_ddbgi(3*3), notice Jr_b=Jr(Jg_deltaR*dbgi), here dbgi=bgi or bg

    //      Sophus::SO3exd errR(deltaRij.transpose()*Rwbi.transpose()*Rwbj);//JW: deltaRij^T * Riw * Rwj, omit the
    //      dRbg/bg? Matrix3d Jlinv = Sophus::SO3exd::JacobianLInv(errR.log()); _jacobianOplusXi =-Jlinv*JgRij;
  }
};

// Later part is unused and left undesigned
/*
 * IDP, (I)nverse (d)epth vertex for a map(p)oint
 */
class VertexIDP : public BaseVertex<1, double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }

  virtual void setToOriginImpl() { _estimate = 1; }

  virtual void oplusImpl(const double* update_) {
    _estimate += update_[0];
    if (_estimate < 1e-6) _estimate = 1e-6;  // todo
  }
};

/*
 * Edge of reprojection error in one frame.
 * Vertex 0: mappoint IDP
 * Veretx 1: reference KF PR
 * Vertex 2: current frame PR
 * Vertex 3: extrinsic pose Tbc(or Tcb)
 */
class EdgePRIDP : public BaseMultiEdge<2, Vector2d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePRIDP() : BaseMultiEdge<2, Vector2d>() { resize(4); }
  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }
  void computeError() {}
  virtual void linearizeOplus() {}

  void SetParams(double x, double y, double fx_, double fy_, double cx_, double cy_) {
    refnormxy[0] = x;
    refnormxy[1] = y;
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
  }

  inline Vector2d project2d(const Vector3d& v) const {
    Vector2d res;
    res(0) = v(0) / v(2);
    res(1) = v(1) / v(2);
    return res;
  }
  Vector2d cam_project(const Vector3d& trans_xyz) const {
    Vector2d proj = project2d(trans_xyz);
    Vector2d res;
    res[0] = proj[0] * fx + cx;
    res[1] = proj[1] * fy + cy;
    return res;
  }
  bool isDepthPositive() {
    Vector3d Pc = computePc();
    return Pc(2) > 0.01;
  }
  Vector3d computePc() {}

 protected:
  // [x,y] in normalized image plane in reference KF
  double refnormxy[2];
  double fx, fy, cx, cy;
};

}  // namespace g2o

#endif
