//
// Created by leavesnight on 5/30/23.
//

#pragma once

#include <cmath>
#include "camera_pinhole.h"

namespace VIEO_SLAM {
namespace camm {
class COMMON_API KB8Camera : public PinholeCamera {
  using Base = PinholeCamera;
  using Base::camera_model_;
  using Base::parameters_;
  using Base::Tcalc;
  using Base::unproject_type_;

  using typename Base::CameraModel;
  using typename Base::CamId;
  using typename Base::SE3data;
  using typename Base::Tsize;
  using typename Base::Vec2data;
  using typename Base::Vec3calc;

 public:
  using Ptr = std::shared_ptr<KB8Camera>;
  using typename Base::Mat23io;
  using typename Base::Mat2Xio;
  using typename Base::Mat32io;
  using typename Base::Mat3io;
  using typename Base::Mat3Xio;
  using typename Base::Tdata;
  using typename Base::Tio;
  using typename Base::Vec3io;

  // parameters: fx, fy, cx, cy, k1, k2, k3, k4
  KB8Camera(CamId camera_id, Tsize width, Tsize height, const std::vector<Tdata> &parameters,
            const SE3data &Tbc = SE3data())
      : Base(camera_id, width, height, parameters, Tbc) {
    assert(8 == parameters_.size());
    camera_model_ = CameraModel::kKB8;
  }
  KB8Camera(const vector<Tdata> &distcoef, cv::FileStorage &fSettings, int id, bool &bmiss_param);
  static bool ParseCamParamFile(cv::FileStorage &fSettings, int id, Camera::Ptr &pCameraInstance);

  inline void Project(const Vec3io &p_3d, Vec2data *p_img, Mat23io *d_img_d_p3d = nullptr,
                      Mat2Xio *d_img_d_param = nullptr) const override;

  inline void UnProject(const Vec2data &p_img, Vec3io *p_3d, Mat32io *d_p3d_d_img = nullptr,
                        Mat3Xio *d_p3d_d_param = nullptr) const override;

  inline bool epipolarConstrain(typename Base::Base *otherCamera, const Vec2data &kp1, const Vec2data &kp2,
                                const Mat3io &R12, const Vec3io &t12, const float sigmaLevel, const float unc,
                                bool bkp_distort = true) const override;

  const vector<int> &GetvLappingArea() const { return vlappingarea_; }
  void SetvLappingArea(const vector<int> &vlappingarea) { vlappingarea_ = vlappingarea; }

 private:
  const float precision_r_ = 1e-5;
  vector<int> vlappingarea_ = {0, INT_MAX};

  // GN iterative method to UnProject
  inline Tcalc SolveTheta(const Tcalc &thetad, Tcalc &d_func_d_theta) const;
};

void KB8Camera::Project(const Vec3io &p_3d, Vec2data *p_img, Mat23io *d_img_d_p3d, Mat2Xio *d_img_d_param) const {
  const Tcalc x = (Tcalc)p_3d[0];
  const Tcalc y = (Tcalc)p_3d[1];

  const Tcalc x2 = x * x, y2 = y * y;
  const Tcalc r2 = x2 + y2;
  const Tcalc r = std::sqrt(r2);

  // Avoid overflow from divison by r
  if (r > precision_r_) {
    const Tdata &fx = parameters_[0];
    const Tdata &fy = parameters_[1];
    const Tdata &k1 = parameters_[4];
    const Tdata &k2 = parameters_[5];
    const Tdata &k3 = parameters_[6];
    const Tdata &k4 = parameters_[7];

    const Tcalc z = (Tcalc)p_3d[2];

    const Tcalc theta = std::atan2(r, z);
    const Tcalc theta2 = theta * theta;

    // thetad = theta * (1 + theta2 * (k1 + theta2 * (k2 + theta2 * (k3 + k4 * theta2)))) for efficiency
    Tcalc thetad = k4 * theta2;
    thetad += k3;
    thetad *= theta2;
    thetad += k2;
    thetad *= theta2;
    thetad += k1;
    thetad *= theta2;
    thetad += 1;
    thetad *= theta;

    const Tcalc mx = x * thetad / r;
    const Tcalc my = y * thetad / r;

    Base::Project(Vec3io(mx, my, 1.), p_img);

    if (d_img_d_p3d) {
      const Tcalc invr = 1. / r;
      const Tcalc d_r_d_x = x * invr;
      const Tcalc d_r_d_y = y * invr;

      const Tcalc tmp = 1. / (z * z + r2);
      const Tcalc d_thetad_x = d_r_d_x * z * tmp;
      const Tcalc d_thetad_y = d_r_d_y * z * tmp;

      Tcalc d_thetad_d_theta = Tcalc(9) * k4 * theta2;
      d_thetad_d_theta += Tcalc(7) * k3;
      d_thetad_d_theta *= theta2;
      d_thetad_d_theta += Tcalc(5) * k2;
      d_thetad_d_theta *= theta2;
      d_thetad_d_theta += Tcalc(3) * k1;
      d_thetad_d_theta *= theta2;
      d_thetad_d_theta += Tcalc(1);

      const Tcalc invr2 = invr * invr;
      // (0, 0) = fx * (thetad * r + x * r * d_thetad_d_theta * d_thetad_x - x * x * thetad / r) / r2
      (*d_img_d_p3d)(0, 0) = fx * (x * r * d_thetad_d_theta * d_thetad_x + y2 * thetad / r) * invr2;
      (*d_img_d_p3d)(0, 1) = fx * x * (d_thetad_d_theta * d_thetad_y * r - y * thetad / r) * invr2;
      // d_thetad_z = -r * tmp; (0, 2) = fx * x * d_thetad_d_theta * d_thetad_z * invr
      (*d_img_d_p3d)(0, 2) = -fx * x * d_thetad_d_theta * tmp;
      // (1, 0) = fy * y * (d_thetad_d_theta * d_thetad_x * r - x * thetad / r) * invr2
      (*d_img_d_p3d)(1, 0) = (*d_img_d_p3d)(0, 1) * fy / fx;
      (*d_img_d_p3d)(1, 1) = fy * (y * r * d_thetad_d_theta * d_thetad_y + x2 * thetad / r) * invr2;
      (*d_img_d_p3d)(1, 2) = -fy * y * d_thetad_d_theta * tmp;
    }

    if (d_img_d_param) {
      d_img_d_param->resize(2, 8);
      (*d_img_d_param).setZero();
      (*d_img_d_param)(0, 0) = mx;
      (*d_img_d_param)(0, 2) = 1;
      (*d_img_d_param)(0, 4) = fx * x * theta * theta2 / r;
      (*d_img_d_param)(1, 1) = my;
      (*d_img_d_param)(1, 3) = 1;
      (*d_img_d_param)(1, 4) = fy * y * theta * theta2 / r;
      d_img_d_param->col(5) = d_img_d_param->col(4) * theta2;
      d_img_d_param->col(6) = d_img_d_param->col(5) * theta2;
      d_img_d_param->col(7) = d_img_d_param->col(6) * theta2;
    }
  } else {
    // degenerate to pinhole at (0, 0) nearby
    Base::Project(p_3d, p_img, d_img_d_p3d, d_img_d_param);
    if (d_img_d_param) {
      d_img_d_param->conservativeResize(2, 8);
      d_img_d_param->block(0, 4, 2, 4).setZero();
    }
  }
}

void KB8Camera::UnProject(const Vec2data &p_img, Vec3io *p_3d, Mat32io *d_p3d_d_img, Mat3Xio *d_p3d_d_param) const {
  const Tdata &fx = parameters_[0];
  const Tdata &fy = parameters_[1];

  Vec3io p_3dtmp;
  Base::UnProject(p_img, &p_3dtmp);
  if (this->kUnProject2Plane != unproject_type_) p_3dtmp /= p_3dtmp[2];
  const Tcalc mx = (Tcalc)p_3dtmp[0];
  const Tcalc my = (Tcalc)p_3dtmp[1];

  Tcalc theta = 0, sin_theta = 0, cos_theta = 1, thetad, scaling;
  Tcalc d_func_d_theta = 0;

  scaling = 1.0;
  thetad = std::sqrt(mx * mx + my * my);
  //  if (this->kUnProject2Plane == unproject_type_)
  thetad = std::min(std::max(-M_PI / 2., thetad), M_PI / 2.);  // ref from orb3, which is useful in our fisheye corner

  if (thetad > this->precision_) {  // old thresh is 1e-5
    theta = SolveTheta(thetad, d_func_d_theta);

    if (this->kUnProject2Sphere == unproject_type_) {
      sin_theta = std::sin(theta);
      cos_theta = std::cos(theta);
    } else {
      sin_theta = std::tan(theta);  // kUnProject2Plane will cause singularity on theta=+-pi/2
      cos_theta = 1.;
    }
    scaling = sin_theta / thetad;
  }

  if (p_3d) {
    auto &pt_3d = *p_3d;
    pt_3d[0] = mx * scaling;
    pt_3d[1] = my * scaling;
    pt_3d[2] = cos_theta;
  }

  if (d_p3d_d_img || d_p3d_d_param) {
    Tcalc d_thetad_d_mx = 0;
    Tcalc d_thetad_d_my = 0;
    Tcalc d_scaling_d_thetad = 0;
    Tcalc d_cos_d_thetad = 0;
    Tcalc d_scaling_d_k1 = 0;
    Tcalc d_cos_d_k1 = 0;
    Tcalc theta2 = 0;

    if (thetad > this->precision_) {  // old thresh is 1e-5
      d_thetad_d_mx = mx / thetad;
      d_thetad_d_my = my / thetad;
      theta2 = theta * theta;

      Tcalc cos_theta_tmp;
      if (this->kUnProject2Sphere == unproject_type_) {
        cos_theta_tmp = cos_theta;
        d_cos_d_thetad = -sin_theta / d_func_d_theta;
      } else {  // here sin_theta is tan_theta
        Tcalc inv_cos_theta = 1. / std::cos(theta);
        cos_theta_tmp = inv_cos_theta * inv_cos_theta;
        d_cos_d_thetad = 0;
      }

      d_scaling_d_thetad = (thetad * cos_theta_tmp / d_func_d_theta - sin_theta) / (thetad * thetad);
      d_scaling_d_k1 = cos_theta_tmp * theta * theta2 / (d_func_d_theta * thetad);
      d_cos_d_k1 = d_cos_d_thetad * theta * theta2;
    }

    const Tcalc d_res0_d_mx = scaling + mx * d_scaling_d_thetad * d_thetad_d_mx;
    const Tcalc d_res0_d_my = mx * d_scaling_d_thetad * d_thetad_d_my;
    const Tcalc d_res1_d_mx = my * d_scaling_d_thetad * d_thetad_d_mx;
    const Tcalc d_res1_d_my = scaling + my * d_scaling_d_thetad * d_thetad_d_my;
    const Tcalc d_res2_d_mx = d_cos_d_thetad * d_thetad_d_mx;
    const Tcalc d_res2_d_my = d_cos_d_thetad * d_thetad_d_my;
    Vec3calc c0, c1;
    c0(0) = d_res0_d_mx / fx;
    c0(1) = d_res1_d_mx / fx;
    c0(2) = d_res2_d_mx / fx;
    c1(0) = d_res0_d_my / fy;
    c1(1) = d_res1_d_my / fy;
    c1(2) = d_res2_d_my / fy;
    if (d_p3d_d_img) {
      d_p3d_d_img->setZero();
      d_p3d_d_img->col(0) = c0.cast<Tio>();
      d_p3d_d_img->col(1) = c1.cast<Tio>();
    }

    if (d_p3d_d_param) {
      d_p3d_d_param->resize(3, 8);
      d_p3d_d_param->setZero();
      d_p3d_d_param->col(0) = (-c0 * mx).cast<Tio>();
      d_p3d_d_param->col(1) = (-c1 * my).cast<Tio>();
      d_p3d_d_param->col(2) = (-c0).cast<Tio>();
      d_p3d_d_param->col(3) = (-c1).cast<Tio>();
      (*d_p3d_d_param)(0, 4) = (Tio)(mx * d_scaling_d_k1);
      (*d_p3d_d_param)(1, 4) = (Tio)(my * d_scaling_d_k1);
      (*d_p3d_d_param)(2, 4) = (Tio)(d_cos_d_k1);  // old code here should be wrong
      d_p3d_d_param->col(5) = (d_p3d_d_param->col(4) * theta2).cast<Tio>();
      d_p3d_d_param->col(6) = (d_p3d_d_param->col(5) * theta2).cast<Tio>();
      d_p3d_d_param->col(7) = (d_p3d_d_param->col(6) * theta2).cast<Tio>();
    }
  }
}

bool KB8Camera::epipolarConstrain(typename Base::Base *otherCamera, const Vec2data &kp1, const Vec2data &kp2,
                                  const Mat3io &R12, const Vec3io &t12, const float sigmaLevel, const float unc,
                                  bool bkp_distort) const {
  // return Base::epipolarConstrain(otherCamera, kp1, kp2, R12, t12, sigmaLevel, unc, bkp_distort);
  // ORB3 st. is ? then ORB2 st.
  float thresh_cosdisparity = 0.9998;  // 1. - 1e-6;
  aligned_vector<SE3io> Twrs = {
      SE3io(), this->GetTrc().cast<Tio>() * SE3io(Sophus::SO3ex<Tio>(R12), t12) * otherCamera->GetTcr().cast<Tio>()};
  auto zs = this->TriangulateMatches(vector<const typename Base::Base *>(1, otherCamera), {kp1, kp2}, {sigmaLevel, unc},
                                     nullptr, thresh_cosdisparity, nullptr, &Twrs);
  if (zs.empty()) return false;
  for (auto z : zs)
    if (z <= 0.0001f) return false;
  return true;
}

KB8Camera::Tcalc KB8Camera::SolveTheta(const Tcalc &thetad, Tcalc &d_func_d_theta) const {
  const Tdata &k1 = parameters_[4];
  const Tdata &k2 = parameters_[5];
  const Tdata &k3 = parameters_[6];
  const Tdata &k4 = parameters_[7];

  Tcalc theta = thetad;
  for (int i = 0; i < this->num_max_iteration_; ++i) {
    Tcalc theta2 = theta * theta;
    Tcalc func = k4 * theta2;
    func += k3;
    func *= theta2;
    func += k2;
    func *= theta2;
    func += k1;
    func *= theta2;
    func += 1;
    func *= theta;

    d_func_d_theta = 9 * k4 * theta2;
    d_func_d_theta += 7 * k3;
    d_func_d_theta *= theta2;
    d_func_d_theta += 5 * k2;
    d_func_d_theta *= theta2;
    d_func_d_theta += 3 * k1;
    d_func_d_theta *= theta2;
    d_func_d_theta += 1;

    // GN iterative method
    Tcalc theta_fix = (thetad - func) / d_func_d_theta;
    theta += theta_fix;
    if (fabs(theta_fix) < this->precision_) break;
  }
  return theta;
}

}  // namespace camm
}  // namespace VIEO_SLAM