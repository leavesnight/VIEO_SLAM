#ifndef G2OTYPES_H
#define G2OTYPES_H

#ifdef USE_G2O_NEWEST
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#else
#include "optimizer/g2o/g2o/core/base_vertex.h"
#include "optimizer/g2o/g2o/core/base_unary_edge.h"
#include "optimizer/g2o/g2o/core/base_binary_edge.h"
#include "optimizer/g2o/g2o/core/base_multi_edge.h"
#include "optimizer/g2o/g2o/types/types_six_dof_expmap.h"
#endif
#include "GeometricCamera.h"
#include "NavState.h"
#include "OdomPreIntegrator.h"

namespace VIEO_SLAM {
// TODO: define template NavState<>
typedef NavState NavStated;
}  // namespace VIEO_SLAM

namespace g2o {

using namespace VIEO_SLAM;
using namespace Eigen;

// extend edges to get H

typedef enum HessianExactMode { kExactNoRobust, kExactRobust, kNotExact } eHessianExactMode;

template <int D, typename E, typename VertexXi>
class BaseUnaryEdgeEx : public BaseUnaryEdge<D, E, VertexXi> {
 protected:
  using Base = BaseUnaryEdge<D, E, VertexXi>;
  using Base::_jacobianOplusXi;
  using MatrixXid = Matrix<double, VertexXi::Dimension, VertexXi::Dimension>;
  using Base::robustInformation;
  using Base::vertices;

 public:
  using Base::chi2;
  using Base::information;
  using Base::robustKernel;
  using typename Base::InformationType;
#ifdef USE_G2O_NEWEST
  using JacobianXiOplusType =
      typename BaseFixedSizedEdge<D, E, VertexXi>::template JacobianType<D, VertexXi::Dimension>;
#else
  using typename Base::JacobianXiOplusType;
#endif
  virtual void getRho(bool& robust, Vector3d& rho) const {
    if (robust) {
      const RobustKernel* robustkernel = robustKernel();
      if (robustkernel)
        robustkernel->robustify(chi2(), rho);
      else
        robust = false;
    }
  }

  // please call linearizeOplus() before calling this function for each edge for g2o share jacobians workspace for all
  // edges
  virtual MatrixXid getHessianXi(bool robust = true) const {
    const JacobianXiOplusType& jac = _jacobianOplusXi;
    Vector3d rho;
    getRho(robust, rho);
    const InformationType& rinfo = robust ? InformationType(rho[1] * information()) : information();
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
#ifdef USE_G2O_NEWEST
  using Base::_hessianTuple;
  using Base::_hessianTupleTransposed;
#else
  using Base::_hessian;
  using Base::_hessianTransposed;
#endif
  using Base::_hessianRowMajor;
  using Base::robustInformation;

 public:
  using Base::chi2;
  using Base::information;
  using Base::robustKernel;
#ifdef USE_G2O_NEWEST
  using typename Base::HessianTuple;
  using typename Base::HessianTupleTransposed;
  using JacobianXiOplusType =
      typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj>::template JacobianType<D, VertexXi::Dimension>;
  using JacobianXjOplusType =
      typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj>::template JacobianType<D, VertexXj::Dimension>;
#else
  using typename Base::HessianBlockTransposedType;
  using typename Base::HessianBlockType;
  using typename Base::JacobianXiOplusType;
  using typename Base::JacobianXjOplusType;
#endif
  using typename Base::InformationType;
  virtual void getRho(bool& robust, Vector3d& rho) const {
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
    const InformationType& rinfo = robust ? InformationType(rho[1] * information()) : information();
    return jac.transpose() * rinfo * jac;
  }
  virtual MatrixXjd getHessianXj(bool robust = true) const {
    const JacobianXjOplusType& jac = _jacobianOplusXj;
    Vector3d rho;
    getRho(robust, rho);
    const InformationType& rinfo = robust ? InformationType(rho[1] * information()) : information();
    return jac.transpose() * rinfo * jac;
  }
  virtual MatrixXijd getHessianXij(int8_t exact_mode = (int8_t)kExactRobust) const {
    if ((int8_t)kNotExact == exact_mode) {
#ifdef USE_G2O_NEWEST
      if (_hessianRowMajor[0]) {
        return MatrixXjid(std::get<0>(_hessianTupleTransposed)).transpose();
#else
      if (_hessianRowMajor) {
        return MatrixXijd(_hessianTransposed.transpose());
#endif
      } else {
#ifdef USE_G2O_NEWEST
        return MatrixXijd(std::get<0>(_hessianTuple));
#else
        return MatrixXijd(_hessian);
#endif
      }
    } else {
      const JacobianXiOplusType& jaci = _jacobianOplusXi;
      const JacobianXjOplusType& jacj = _jacobianOplusXj;
      Vector3d rho;
      bool robust = (int8_t)kExactRobust == exact_mode;
      getRho(robust, rho);
      const InformationType& rinfo = robust ? InformationType(rho[1] * information()) : information();
      return jaci.transpose() * rinfo * jacj;
    }
  }
  virtual MatrixXjid getHessianXji(int8_t exact_mode = (int8_t)kExactRobust) const {
    if ((int8_t)kNotExact == exact_mode) {
#ifdef USE_G2O_NEWEST
      if (_hessianRowMajor[0]) {
        return MatrixXjid(std::get<0>(_hessianTupleTransposed));
#else
      if (_hessianRowMajor) {
        return MatrixXjid(_hessianTransposed);
#endif
      } else {
#ifdef USE_G2O_NEWEST
        return MatrixXijd(std::get<0>(_hessianTuple)).transpose();
#else
        return MatrixXjid(_hessian.transpose());
#endif
      }
    } else {
      const JacobianXiOplusType& jaci = _jacobianOplusXi;
      const JacobianXjOplusType& jacj = _jacobianOplusXj;
      Vector3d rho;
      bool robust = (int8_t)kExactRobust == exact_mode;
      getRho(robust, rho);
      const InformationType& rinfo = robust ? InformationType(rho[1] * information()) : information();
      return jacj.transpose() * rinfo * jaci;
    }
  }
};

template <int D, typename E>
class BaseMultiEdgeEx : public BaseMultiEdge<D, E> {
 protected:
  using Base = BaseMultiEdge<D, E>;
  using Base::_hessian;
  using Base::_jacobianOplus;
  using Base::robustInformation;  // TODO: add const postfix to g2o robustInformation

 public:
  using Base::chi2;
  using Base::information;
  using Base::robustKernel;
  using typename Base::HessianHelper;
  using typename Base::InformationType;
  using typename Base::JacobianType;
  virtual void getRho(bool& robust, Vector3d& rho) const {
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
    const InformationType& rinfo = robust ? InformationType(rho[1] * information()) : information();
    return jac.transpose() * rinfo * jac;
  }
  virtual MatrixXd getHessianij(int iv, int jv, int8_t exact_mode = (int8_t)kExactRobust) const {
    if ((int8_t)kNotExact == exact_mode) {
      bool btranspose = false;
      if (iv > jv) {  // keep same order to get idx as buildStructure in block_solver.hpp
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
      const InformationType& rinfo = robust ? InformationType(rho[1] * information()) : information();
      return jaci.transpose() * rinfo * jacj;
    }
  }
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
class VertexNavState : public BaseVertex<D, NavStated> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  virtual bool read(std::istream& is) { return true; }

  virtual bool write(std::ostream& os) const { return true; }

  void setToOriginImpl() { this->_estimate = NavStated(); }  // virtual
  void oplusImpl(const double* update_) {
    Eigen::Map<const Matrix<double, D, 1>> update(update_);
    this->_estimate.IncSmall(update);
  }  // virtual
};

typedef VertexNavState<6> VertexNavStatePR;
typedef VertexNavState<3> VertexNavStateV;
typedef VertexNavState<9> VertexNavStatePVR;  // TODO: delete this for code simplicity

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

#ifdef USE_G2O_NEWEST
typedef VertexPointXYZ VertexSBAPointXYZ;  // for their oplusImpl&&setToOriginImpl is the same/normal
#endif

/**
 * \brief template for EdgeProjectXYZ(binary edge, mono/stereo), similar to EdgeProjectXYZOnlyPose(change the parameter
 * Pw to optimized vertex _vertices[0] & Tbw to _vertices[1])
 */
template <int DE, int DV, int NV, int MODE_OPT_VAR = 0>
class EdgeReproject : public BaseMultiEdgeEx<DE, Matrix<double, DE, 1>> {
  using VectorDEd = Matrix<double, DE, 1>;

  typedef BaseMultiEdgeEx<DE, VectorDEd> Base;

  const int offset_Twbh_ = (0 == MODE_OPT_VAR && 3 <= NV) ? 2 : -1;
  const int offset_scale_ =
      (0 == MODE_OPT_VAR && 4 <= NV) ? 3 : (((1 == MODE_OPT_VAR || 2 == MODE_OPT_VAR) && 3 <= NV) ? 2 : -1);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeReproject() : Base() { Base::resize(NV); }

  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }
  static VectorDEd cam_project(GeometricCamera* intr, Vector3d x_C, double bf = 0) {
    VectorDEd res;
    res.template segment<2>(0) = intr->project(x_C);
    if (DE > 2) res[2] = res[0] - bf / x_C[2];
    return res;
  }
  inline void GetTcw_wX(Matrix3d& Rcw, Vector3d& tcw, Vector3d& Xw, double* pscale = nullptr,
                        Vector3d* phX_unscale = nullptr, Matrix3d* pRwh = nullptr, Matrix3d* pRwbh = nullptr,
                        Matrix3d* pRbw = nullptr, Vector3d* ptwh = nullptr) {
    const VertexSBAPointXYZ* pXh = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);  // Xh/Ph
    // Tbs_w, bs is b for slam
    const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);
    const NavStated& ns = vNS->estimate();  // transform Xh to Xc through Tbw&&Tcb
    double scale = 1;
    if (-1 != offset_scale_) {
      const VertexScale* vScale;
      vScale = static_cast<const VertexScale*>(_vertices[offset_scale_]);  // Scale
      scale = vScale->estimate();
    }
    {
      Matrix3d Rwb = ns.getRwb();
      Vector3d twb = ns.mpwb;
      if (pRbw) *pRbw = ns.getRwb();
      if (2 == MODE_OPT_VAR) {
        scale = 1. / scale;
        Rwb.transposeInPlace();
        twb = -(Rwb * twb);
      }
      Rcw = Rcb * Rwb.transpose();
      tcw = -(Rcw * twb) + tcb;
      // notice that tcb&twb could be scale near 1 and Tcb is Tcicr used in MODE2
      if (2 == MODE_OPT_VAR) tcw *= scale;
      if (pscale) *pscale = scale;
    }
    Xw = pXh->estimate() * scale;
    if (phX_unscale) *phX_unscale = pXh->estimate();
    if (-1 != offset_Twbh_) {
      const VertexNavState<DV>* vNSh = static_cast<const VertexNavState<DV>*>(_vertices[offset_Twbh_]);  // Tbw,handler
      const NavStated& nsh = vNSh->estimate();
      Matrix3d Rwh;
      Vector3d twh;
      {
        Matrix3d Rwbh = nsh.getRwb();
        Vector3d twbh = nsh.mpwb;
        if (2 == MODE_OPT_VAR) {
          Rwbh.transposeInPlace();
          twbh = -(Rwbh * twbh);
        }
        Rwh = Rwbh * Rbch_;
        twh = Rwbh * tbch_ + twbh;
        if (pRwh) *pRwh = Rwh.matrix();
        if (pRwbh) *pRwbh = Rwbh.matrix();
        if (2 == MODE_OPT_VAR) twh *= scale;
      }
      // wX=Twh*hX=Rwh*hX+twh=Rwb*Rbh*hX + (Rwb*tbh+twb)=Rwb(Rbh*hX + tbh) + twb
      Xw = Rwh * Xw + twh;

      if (ptwh) *ptwh = std::move(twh);
    }
  }
  void computeError() override {
    Matrix3d Rcw;
    Vector3d tcw, wX;
    GetTcw_wX(Rcw, tcw, wX);
    // Pc=Tcb*Tbw*Pw=Rcb*Rbw*Pw+Rcb*tbw(-Rcb*Rbw*twb)+tcb(-Rcb*tbc)=Rcb*Rbw*(Pw-twb)+tcb;
    this->_error = this->_measurement - cam_project(intr_, Rcw * wX + tcw, bf_);
  }
  void linearizeOplus() override;

  void SetParams(GeometricCamera* intr, const Matrix3d Rcrb_ = Matrix3d::Identity(),
                 const Vector3d tcrb_ = Vector3d::Zero(), const float* bf = NULL) {
    intr_ = intr;
    Matrix3d Rccr = intr_->GetTcr().rotationMatrix();
    Rcb = Rccr * Rcrb_;
    tcb = Rccr * tcrb_ + intr_->GetTcr().translation();
    if (bf) bf_ = *bf;
  }
  void SetParams(const Matrix3d& Rbch, const Vector3d& tbch) {
    Rbch_ = Rbch;
    tbch_ = tbch;
  }
  bool isDepthPositive() {  // unused in IMU motion-only BA, but used in localBA&GBA
    Matrix3d Rcw;
    Vector3d tcw, wX;
    GetTcw_wX(Rcw, tcw, wX);
    return (Rcw * wX + tcw)(2) > 0.0;  // Xc.z>0
  }

 protected:
  double bf_;
  GeometricCamera* intr_;  // Camera intrinsics
  // Camera-IMU extrinsics
  Matrix3d Rcb, Rbch_;
  Vector3d tcb, tbch_;

  using Base::_jacobianOplus;
  using Base::_vertices;
};
template <int DE, int DV, int NV, int MODE_OPT_VAR>
void EdgeReproject<DE, DV, NV, MODE_OPT_VAR>::linearizeOplus() {
  Matrix3d Rcw, Rwh, Rwbh, Rbw;
  Vector3d tcw, Ph_unscale, Pw, twh;
  double scale;
  // TODO(zzh): put it in update
  if (2 != MODE_OPT_VAR)
    GetTcw_wX(Rcw, tcw, Pw, &scale, &Ph_unscale, &Rwh, &Rwbh);
  else
    GetTcw_wX(Rcw, tcw, Pw, &scale, &Ph_unscale, &Rwh, &Rwbh, &Rbw, &twh);
  Vector3d Ph = Ph_unscale * scale;

  Vector3d Pc = Rcw * Pw + tcw;  // Pc=Rcb*Rbw*(Pw-twb)+tcb
  double invz = 1 / Pc[2], invz_2 = invz * invz;

  // Jacobian of camera projection, par((K*Pc)(0:1))/par(Pc)=J_e_Pc, error = obs - pi( Pc )
  Matrix<double, DE, 3> Jproj;  // J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2], here Xc->Xc+dXc
  Jproj.template block<2, 3>(0, 0) = -intr_->projectJac(Pc);
  // ur=ul-b*fx/dl,dl=z => J_e_P'=J_e_Pc=-[fx/z 0 -fx*x/z^2; 0 fy/z -fy*y/z^2; fx/z 0 -fx*x/z^2+bf/z^2]
  if (DE > 2) Jproj.template block<1, 3>(2, 0) << Jproj(0, 0), Jproj(0, 1), Jproj(0, 2) - bf_ * invz_2;

    // Jacobian of error w.r.t dPwb = JdPwb=J_e_Pc*J_Pc_dPwb, notcie we use pwb->pwb+dpwb increment model in the
    // corresponding Vertex, so here is the same, a bit dfferent from (21)
#ifdef USE_P_PLUS_RDP
  Matrix<double, DE, 3> JdPwb = Jproj * (-Rcb);  // J_Pc_dPwb = -Rcw*Rwb= -Rcb(p<-p+R*dp)
#else
  Matrix<double, DE, 3> JdPwb = Jproj * (-Rcw.matrix());  // J_Pc_dPwb = -Rcw(p<-p+dp)
#endif

  // Jacobian of error w.r.t dRwb
  // J_Pc_dRwb=(Rcw*(Pw-twb))^Rcb, using right disturbance model/Rwb->Rwb*Exp(dphi) or Rbw->Exp(-dphi)*Rbw,
  // see Manifold paper (20)
  Matrix<double, DE, 3> JdRwb;
  {
    Vector3d Paux;
    if (2 != MODE_OPT_VAR) {
      const VertexNavState<DV>* vNS = static_cast<const VertexNavState<DV>*>(_vertices[1]);
      const NavStated& ns = vNS->estimate();  // transform Xh to Xc through Tbw&&Tcb
      Paux = ns.getRwb().transpose() * (Pw - ns.mpwb);
      JdRwb = Jproj * Rcb * Sophus::SO3exd::hat(Paux);  // Jproj * (Sophus::SO3exd::hat(Paux) * Rcb.matrix());
    } else {
      Paux = Rcw * Pw + (tcw - tcb * scale);
      JdRwb = Jproj * (Sophus::SO3exd::hat(Paux) * Rcb.matrix());
    }
  }

  // Jacobian of error w.r.t NavStatePR, order in 'update_': dP, dPhi
  Matrix<double, DE, DV> JNavState = Matrix<double, DE, DV>::Zero();
  JNavState.template block<DE, 3>(0, 0) = JdPwb;       // J_error_dnotPR=0 so we'd better use PR&V instead of PVR/PVRB
  JNavState.template block<DE, 3>(0, DV - 3) = JdRwb;  // only for 9(J_e_dV=0)/6
  _jacobianOplus[1] = JNavState;

  // Jacobian of error(-pc) w.r.t dXh/dPh: J_e_dXh=JdXh=J_e_Pc*J_Pc_dPw*J_Pw_dPh=Jproj*Rcw*Rwh=-JdPwb*Rwh
#ifdef USE_P_PLUS_RDP
  _jacobianOplus[0] = Jproj * (Rcw.matrix());  // JdPwb * Rwb.transpose();  // for (p<-p+R*dp)
#else
  _jacobianOplus[0] = -JdPwb;  // Jproj*Rcb*Rwb.transpose()*Rwh; it's a fast form for (p<-p+dp);Rwh=I for NV==2
#endif
  if (-1 != offset_Twbh_) {
    // J_e_dPwb = J_e_dXw * J_Xw_dPwbh = Jproj*Rcw*I
    _jacobianOplus[offset_Twbh_].template block<DE, 3>(0, 0) = _jacobianOplus[0];
    // J_e_dRwb = J_e_dXw * J_Xw_dRwbh = Jproj*Rcw*(-Rwbh*(Rbh*hX+tbh)^), or J_Xw_dRwbh= -(Rwh(hX-thb)^Rhb)
    _jacobianOplus[offset_Twbh_].template block<DE, 3>(0, DV - 3) =
        _jacobianOplus[0] * (Rwbh * Sophus::SO3exd::hat(-(Rbch_ * Ph + tbch_)));
    _jacobianOplus[0] *= Rwh;
  }
  if (-1 != offset_scale_) {
    // Jacobian of error(-pc) w.r.t ds: J_e_ds=Jds=J_e_Pc*J_Pc_ds
    _jacobianOplus[offset_scale_] = _jacobianOplus[0] * Ph_unscale;  // J_Pc_ds=Rcw*Pw, easy to prove
  }
  _jacobianOplus[0] *= scale;  // chain rule, for wX= s*hX now
  if (2 == MODE_OPT_VAR) {
    // chain rule
    Matrix3d Rwb = Rbw.transpose();
    // J_phiwb_phibw=extend_J_ARwbB(RD)/phibw_ARwbB(RD)/phiwb
    _jacobianOplus[1].template block<DE, 3>(0, DV - 3) *= -Rbw;
    _jacobianOplus[1].template block<DE, 3>(0, 0) *= -Rwb;  // J_twb_tbw

    // J_s(=1./scale) = J_s(=scale) + Jproj * (tcw_unscale + (twh_unscale)), J_s_1/s = -1/s^2 = -scale^2
    Vector3d tscale_ext = tcw;
    if (-1 != offset_Twbh_) {
      tscale_ext += twh;
      Matrix3d Rbhw = Rwbh.transpose();
      _jacobianOplus[offset_Twbh_].template block<DE, 3>(0, DV - 3) *= -Rbhw;
      _jacobianOplus[offset_Twbh_].template block<DE, 3>(0, 0) *= -Rwbh;
    }
    if (-1 != offset_scale_)
      _jacobianOplus[offset_scale_] = (_jacobianOplus[offset_scale_] * scale + Jproj * tscale_ext) * (-scale);
  }
}

typedef EdgeReproject<2, 6, 2> EdgeReprojectPR;
typedef EdgeReproject<2, 6, 3> EdgeReprojectPR3V;  // designed for 3d points with known model coordinates
typedef EdgeReproject<3, 6, 2> EdgeReprojectPRStereo;
typedef EdgeReproject<2, 9, 2> EdgeReprojectPVR;
typedef EdgeReproject<3, 9, 2> EdgeReprojectPVRStereo;
typedef EdgeReproject<2, 6, 3, 1> EdgeReprojectPRS;
typedef EdgeReproject<2, 6, 3, 2> EdgeReprojectPRSInv;
typedef EdgeReproject<3, 6, 3, 1> EdgeReprojectPRSStereo;

// have to define Bias independently for it has the same dimension with PR
class VertexNavStateBias : public BaseVertex<6, NavState> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  virtual bool read(std::istream& is) { return true; }

  virtual bool write(std::ostream& os) const { return true; }

  void setToOriginImpl() { this->_estimate = NavState(); }  // virtual
  void oplusImpl(const double* update_) {
    Eigen::Map<const Matrix<double, 6, 1>> update(update_);
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

template <int DV>
class EdgeEncNavState : public BaseBinaryEdgeEx<6, Vector6d, VertexNavState<DV>, VertexNavState<DV>> {
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
  // Log(delta~Rij.t()*Reiej)
  this->_error.template segment<3>(0) =
      Sophus::SO3exd(Sophus::SO3exd::exp(this->_measurement.template segment<3>(0)).inverse() * so3Reiej).log();
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
#ifdef USE_P_PLUS_RDP
  Jep_dpi *= nsi.mRwb.matrix();
#endif
  Matrix3d Jep_dphii =
      Reb * Sophus::SO3exd::hat(qRij * pbe + qRiw * (pwj - pwi));  // Jep_dphii=Reb*[Rbiw*(Rwbj*pbe+pwbj-pwbi)]^
  // calculate Je_dxj xj=ksj=(phij,rhoj)
  Matrix3d JeR_dphij = Sophus::SO3exd::JacobianRInv(eR) * Reb;  // JeR_dphij=Jrinv(eR)*Reb
  Matrix3d Jep_dpj = RebRiw;                                    // Jep_dpj=Reb*Rbiw
#ifdef USE_P_PLUS_RDP
  Jep_dpj *= nsj.mRwb.matrix();
#endif
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
 * @brief The VertexGThetaXYRwI class
 * For Initial-Only BA, right disturbance model of Theta_wI_xy
 */
class VertexGThetaXYRwI : public BaseVertex<2, Sophus::SO3exd> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  bool read(std::istream& is) { return true; }
  bool write(std::ostream& os) const { return true; }

  void setToOriginImpl() {}
  void setToOriginImpl(Vector3d& gw) {
    Vector3d gwn = gw / gw.norm();
    Vector3d gIn;
    gIn << 0, 0, 1;
    Vector3d a_wI = gIn.cross(gwn);
    Vector3d vhat = a_wI.normalized();  // notice that even norm_gIn=1 and norm_gwn=1, norm_a_wI may not be 1!
    double theta_wI_val = std::acos(gIn.dot(gwn));         // ORB3 code means acos(gIn.dot(gwn)) is enough
    _estimate = Sophus::SO3exd::exp(vhat * theta_wI_val);  // RwI=Exp(theta_wI)
  }
  void oplusImpl(const double* update) {
    Vector3d update3;
    update3 << update[0], update[1], 0;
    //            estimate << _estimate[0], _estimate[1], _estimate[2];
    _estimate = _estimate * Sophus::SO3exd::exp(update3);
  }
};

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

  void SetParams(const Vector3d& gw) { gw_ = gw; }

 protected:
  Vector3d gw_;  // gw: Gravity vector in 'world' frame
  // NV==5/6 -> <6>PR NV==3 -> <9>PVR, this is very important typedef!
  typedef VertexNavState<(NV >= 5 ? 6 : 9)> VertexNavStateNV;
};
template <int NV>
void EdgeNavStateI<NV>::computeError() {
  const VertexNavStateNV* vPRi = static_cast<const VertexNavStateNV*>(_vertices[0]);
  const VertexNavStateNV* vPRj = static_cast<const VertexNavStateNV*>(_vertices[1]);
  const NavState &nsPRi = vPRi->estimate(), &nsPRj = vPRj->estimate(), *pBiasi;
  // q*p=qpq.conj=15x+15+,Rp=9x+6+,R(q)=24 and we found bad result in hard fast motion dataset for q*p here
  const Sophus::SO3exd so3RiT = nsPRi.mRwb.inverse();
  const Matrix3d RiT = nsPRi.getRwb().transpose();
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

  Vector3d gw = gw_;
  if (6 == NV) {
    const VertexGThetaXYRwI* vG = static_cast<const VertexGThetaXYRwI*>(_vertices[NV - 1]);
    gw = vG->estimate() * gw_;  // here gw_ is GI
  }

  // see VIORBSLAM paper (6)/ Manifold (45)
  //  r_deltapij/ep=Rbiw*(pwbj-pwbi-vwbi*deltatij-1/2*gw*deltatij^2)-
  //  (deltapij+J_g_deltap*dbgi+J_a_deltap*dbai),here deltapij=delta~pij(bi_bar)
  _error.segment<3>(0) = RiT * (nsPRj.mpwb - nsPRi.mpwb - vi * deltat - gw * (deltat * deltat / 2)) -
                         (_measurement.mpij + _measurement.mJgpij * dbgi + _measurement.mJapij * dbai);
  // if NV==5 then error_PRV else ePVR
  int idR = (NV >= 5) ? 3 : 6;
  // eR=Log((deltaRij*Exp(Jg_deltaR*dbgi)).t()*Rbiw*Rwbj)
  _error.segment<3>(idR) =
      ((Sophus::SO3exd(_measurement.mRij) * Sophus::SO3exd::exp(_measurement.mJgRij * dbgi)).inverse() *
       (so3RiT * nsPRj.mRwb))
          .log();
  //  _error.segment<3>(idR) = Sophus::SO3exd::Log(
  //      (_measurement.mRij * Sophus::SO3exd::Exp(_measurement.mJgRij * dbgi)).transpose() * RiT * nsPRj.getRwb());
  // ev=Rwbi.t()*(vwbj-vwbi-gw*deltatij)-(deltavij+J_g_deltav*dbgi+J_a_deltav*dbai)
  _error.segment<3>(9 - idR) =
      RiT * (vj - vi - gw * deltat) - (_measurement.mvij + _measurement.mJgvij * dbgi + _measurement.mJavij * dbai);
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

  Vector3d gw = gw_;
  Matrix<double, 3, 2> RwIGIhat;  // Exp(thetag_wI) * GI^(col:[0,2))
  if (6 == NV) {
    const VertexGThetaXYRwI* vG = static_cast<const VertexGThetaXYRwI*>(_vertices[NV - 1]);
    Sophus::SO3exd RwI = vG->estimate();
    gw = RwI * gw_;  // here gw_ is GI
    RwIGIhat = RwI.matrix() * Sophus::SO3exd::hat(gw_).block<3, 2>(0, 0);
  }

  // J_rpij_dxi; J_rpij_dPhi_i
  JPRVi.block<3, 3>(0, idR) = Sophus::SO3exd::hat(RiT * (pj - pi - vi * deltat - gw * (deltat * deltat / 2)));
#ifdef USE_P_PLUS_RDP
  // J_rpij_dpi=-I3x3, notice here use pi<-pi+Ri*dpi
  JPRVi.block<3, 3>(0, 0) = -Matrix3d::Identity();
#else
  // J_rpij_dpi, notice here use pi<-pi+dpi not the form pi<-pi+Ri*dpi in the paper!
  JPRVi.block<3, 3>(0, 0) = -RiT;
#endif
  JPRVi.block<3, 3>(0, idV) = -RiT * deltat;        // J_rpij_dvi
  JBiasi.block<3, 3>(0, 0) = -_measurement.mJgpij;  // J_rpij_ddbgi
  JBiasi.block<3, 3>(0, 3) = -_measurement.mJapij;  // J_rpij_ddbai
  // J_rpij_dxj
  JPRVj.block<3, 3>(0, idR) = O3x3;  // J_rpij_dPhi_j
#ifdef USE_P_PLUS_RDP
  // J_rpij_dpj=Ri.t()*Rj, notice here use pj<-pj+Rj*dpj
  JPRVj.block<3, 3>(0, 0) = RiT * nsPRj.getRwb();
#else
  // J_rpij_dpj, notice here use pj<-pj+dpj not the form pj<-pj+Rj*dpj in the paper!
  JPRVj.block<3, 3>(0, 0) = RiT;
#endif
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
  JBiasi.block<3, 3>(idR, 0) =
      -Jrinv * Sophus::SO3exd::Exp(-eR) * Sophus::SO3exd::JacobianR(_measurement.mJgRij * dbgi) * _measurement.mJgRij;
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

    if (6 == NV) {
      Matrix<double, 9, 2> JG;           // ref ORB3: Inertial-Only Optimization for Visual-Inertial Initialization
      JG.block<3, 2>(idR, 0).setZero();  // J_rRij_ddthetag_xy = 0
      // ORB3 use gI(z:-G) but gI'withG is z:G so he has - here, but we don't
      // J_rvij_ddthetag_xy = Rwbi.t() * delta_tij * Exp(thetag_wI) * GI^(col:[0,2))
      JG.block<3, 2>(idV, 0) = RiT * deltat * RwIGIhat;
      // J_rpij_ddthetag_xy = Rwbi.t()/2*delta_tij^2*Exp(thetag_wI) * GI^(col:[0,2))
      JG.block<3, 2>(0, 0) = RiT * (deltat * deltat / 2.0) * RwIGIhat;

      _jacobianOplus[5] = JG;  // 9*2
    }
  } else {  // J_ePVR_PVRi,PVRj,Bi
    _jacobianOplus[0] = JPRVi;
    _jacobianOplus[1] = JPRVj;  // when NV==3 it's JPVRi,JPVRj, 9*9,9*9
    _jacobianOplus[2] = JBiasi;
  }
}

typedef EdgeNavStateI<5> EdgeNavStatePRV;   // PRi, PRj, Vi, Vj, Bi, total 5 vertices
typedef EdgeNavStateI<6> EdgeNavStatePRVG;  // for Inertial-Only BA, ThetaXY_G added at end
typedef EdgeNavStateI<3> EdgeNavStatePVR;   // PVRi, PVRj, Bi, total 3 vertices
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
    Vector3d bg = v->estimate();                      // bgi, here we think bg_=0, dbgi=bgi, see VIORBSLAM IV-A
    Matrix3d dRbg = Sophus::SO3exd::Exp(JgRij * bg);  // right dR caused by dbgi
    // eR=Log((deltaRij*Exp(Jg_deltaR*dbgi)).t()*Rbiw*Rwbj), _error.segment<3>(0) is _error itself
    // Exp(eR) = deltaRij^T * Riw * Rwj
    _error = Sophus::SO3exd::Log((deltaRij * dRbg).transpose() * Rwbi.transpose() * Rwbj);
  }
  virtual void linearizeOplus() {  // I think JingWang may be wrong here?both not best, JW avoids large bg's influence
                                   // but lose variance on Jac(could be used in low freq/cov IMU)
    const VertexGyrBias* v = static_cast<const VertexGyrBias*>(_vertices[0]);
    Vector3d bg = v->estimate();
    Matrix3d Jrinv = Sophus::SO3exd::JacobianRInv(_error);  // JrInv_rPhi/Jrinv_rdeltaRij/Jrinv_eR
    // right is Exp(rdeltaRij).t(), same as Sophus::SO3exd::exp(rPhiij).inverse().matrix()
    // J_rRij_ddbgi(3*3), notice Jr_b=Jr(Jg_deltaR*dbgi), here dbgi=bgi or bg
    _jacobianOplusXi = -Jrinv * Sophus::SO3exd::Exp(-_error) * Sophus::SO3exd::JacobianR(JgRij * bg) * JgRij;

    // Sophus::SO3exd errR(deltaRij.transpose()*Rwbi.transpose()*Rwbj);
    // JW: deltaRij^T * Riw * Rwj, omit the dRbg/bg?
    // Matrix3d Jlinv = Sophus::SO3exd::JacobianLInv(errR.log());
    // _jacobianOplusXi =-Jlinv*JgRij;
  }
};

/**
 * \brief EdgeGyrBiasPrior(binary edge)
 */
class EdgeGyrBiasPrior : public BaseBinaryEdge<3, IMUPreintegrator, VertexGyrBias, VertexGyrBias> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor is enough
  bool read(std::istream& is) { return true; }

  bool write(std::ostream& os) const { return true; }

  void computeError();

  virtual void linearizeOplus();
};  // v0 is Bi, v1 is Bj

}  // namespace g2o

#endif
