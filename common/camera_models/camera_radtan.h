//
// Created by leavesnight on 5/30/23.
//

#pragma once

#include "camera_pinhole.h"

namespace VIEO_SLAM {
namespace camm {
class COMMON_API RadtanCamera : public PinholeCamera {
  using Base = PinholeCamera;
  using Base::camera_model_;
  using Base::parameters_;
  using Base::Tcalc;
  using Base::unproject_type_;

  using typename Base::CameraModel;
  using typename Base::CamId;
  using typename Base::SE3data;
  using typename Base::Tsize;
  using typename Base::Vec2calc;
  using typename Base::Vec2data;

  using Mat22calc = Eigen::Matrix<Tcalc, 2, 2>;

  int num_k_ = 2;

 public:
  using Ptr = std::shared_ptr<RadtanCamera>;
  using typename Base::Mat23io;
  using typename Base::Mat2Xio;
  using typename Base::Mat32io;
  using typename Base::Mat3Xio;
  using typename Base::Tdata;
  using typename Base::Tio;
  using typename Base::Vec3io;

  // parameters: fx, fy, cx, cy, k1, k2, p1, p2
  RadtanCamera(CamId camera_id, Tsize width, Tsize height, const std::vector<Tdata> &parameters,
               const SE3data &Tbc = SE3data())
      : Base(camera_id, width, height, parameters, Tbc) {
    auto sz_param = parameters_.size();
    if (8 > sz_param)
      assert(0);
    else
      num_k_ = sz_param - 6;
    camera_model_ = CameraModel::kRadtan;
    this->num_max_iteration_ = 5;
  }
  RadtanCamera(const vector<Tdata> &distcoef, cv::FileStorage &fSettings, int id, bool &bmiss_param);
  static bool ParseCamParamFile(cv::FileStorage &fSettings, int id, Camera::Ptr &pCameraInstance);

  inline void Project(const Vec3io &p_3d, Vec2data *p_img, Mat23io *d_img_d_p3d = nullptr,
                      Mat2Xio *d_img_d_param = nullptr) const override;

  inline void UnProject(const Vec2data &p_img, Vec3io *p_3d, Mat32io *d_p3d_d_img = nullptr,
                        Mat3Xio *d_p3d_d_param = nullptr) const override;
};

void RadtanCamera::Project(const Vec3io &p_3d, Vec2data *p_img, Mat23io *d_img_d_p3d, Mat2Xio *d_img_d_param) const {
  const Tdata &fx = parameters_[0];
  const Tdata &fy = parameters_[1];

  const Tdata *k = parameters_.data() + 4;
  const Tdata *p = k + num_k_;

  Tcalc invz = 1 / p_3d[2];
  Tcalc x = (Tcalc)p_3d[0] * invz;
  Tcalc y = (Tcalc)p_3d[1] * invz;

  Tcalc x2 = x * x;
  Tcalc y2 = y * y;
  Tcalc xy = x * y;
  Tcalc r2 = x2 + y2;

  Tcalc fd = 1;  // 1 + rad_dist_u
  Tcalc term_r = 1;
  for (int i = 0; i < num_k_; ++i) {
    term_r *= r2;
    fd += k[i] * term_r;
  }
  if (d_img_d_p3d) {
    Tcalc fd2 = 0, coeff2 = 0;
    term_r = 1;
    for (int i = 2; i < num_k_; ++i) {
      coeff2 += 2;
      fd2 += coeff2 * k[i] * term_r;
      term_r *= r2;
    }
    Tcalc du_dx = fx * invz * (fd + fd2 * x2 + 2 * (p[0] * y + 3 * p[1] * x));
    Tcalc du_dy = fx * invz * (fd2 * xy + 2 * (p[0] * x + p[1] * y));
    Tcalc du_dz = -(x * du_dx + y * du_dy);
    Tcalc dv_dx = du_dy * fy / fx;
    Tcalc dv_dy = fy * invz * (fd + fd2 * y2 + 2 * (p[1] * x + 3 * p[0] * y));
    Tcalc dv_dz = -(x * dv_dx + y * dv_dy);

    (*d_img_d_p3d) << du_dx, du_dy, du_dz, dv_dx, dv_dy, dv_dz;
  }

  if (d_img_d_param) {
    d_img_d_param->resize(2, 6 + num_k_);
    d_img_d_param->setZero();

    (*d_img_d_param)(0, 2) = 1;
    term_r = 1;
    for (int i = 0; i < num_k_; ++i) {
      term_r *= r2;
      (*d_img_d_param)(0, 4 + i) = fx * x * term_r;
      (*d_img_d_param)(1, 4 + i) = fy * y * term_r;
    }
    (*d_img_d_param)(0, 4 + num_k_) = 2 * fx * xy;
    (*d_img_d_param)(0, 5 + num_k_) = fx * (r2 + 2 * x2);
    (*d_img_d_param)(1, 3) = 1;
    (*d_img_d_param)(1, 4 + num_k_) = fy * (r2 + 2 * y2);
    (*d_img_d_param)(1, 5 + num_k_) = 2 * fy * xy;
  }

  // DistortPt
  x = x * fd + 2 * p[0] * xy + p[1] * (r2 + 2 * x2);
  y = y * fd + 2 * p[1] * xy + p[0] * (r2 + 2 * y2);

  if (d_img_d_param) {
    (*d_img_d_param)(0, 0) = x;
    (*d_img_d_param)(1, 1) = y;
  }

  Base::Project(Vec3io(x, y, 1.), p_img);
}

// TODO: unproject jacobi
void RadtanCamera::UnProject(const Vec2data &p_img, Vec3io *p_3d, Mat32io *d_p3d_d_img, Mat3Xio *d_p3d_d_param) const {
  assert(!d_p3d_d_img && !d_p3d_d_param && "Unimplemented UnProjectJac in Radtan Camera Model!");
  const Tdata *k = parameters_.data() + 4;

  //  Vec2calc y = Vec2calc(((Tcalc)p_img(0) - cx) / fx, ((Tcalc)p_img(1) - cy) / fy);
  Vec3io p_3dtmp;
  Base::UnProject(p_img, &p_3dtmp);
  if (this->kUnProject2Plane != unproject_type_) p_3dtmp /= p_3dtmp[2];
  Vec2calc y = p_3dtmp.segment<2>(0).cast<Tcalc>();

  Vec2calc y_bar = y;
  Mat22calc F;  // d(DistortPt(y_tmp)-y)/dy_tmp
  Vec2calc y_tmp;

  const Tcalc precision2 = this->precision_ * this->precision_;
  for (int i = 0; i < this->num_max_iteration_; ++i) {
    y_tmp = y_bar;

    // GN iterative method
    Vec2data y_tmp2;
    Mat23io dimg_dp3d;
    Project((Vec3io() << y_tmp(0), y_tmp(1), 1).finished(), &y_tmp2, &dimg_dp3d);
    Base::UnProject(y_tmp2, &p_3dtmp);
    if (this->kUnProject2Plane != unproject_type_) p_3dtmp /= p_3dtmp[2];
    y_tmp = p_3dtmp.segment<2>(0).cast<Tcalc>();
    const Tdata &fx = parameters_[0];
    const Tdata &fy = parameters_[1];
    F(0, 0) = dimg_dp3d(0, 0) / fx;
    F(0, 1) = dimg_dp3d(0, 1) / fx;
    F(1, 0) = F(0, 1);  // dimg_dp3d(1, 0) / fy;
    F(1, 1) = dimg_dp3d(1, 1) / fy;

    Vec2calc e(y - y_tmp);  // e=-f(x)
    Vec2calc du = (F.transpose() * F).inverse() * F.transpose() * e;
    y_bar += du;

    if (e.dot(e) < precision2) break;
  }

  if (p_3d) {
    auto &pt_3d = *p_3d;
    pt_3d(0) = (Tdata)y_bar(0);
    pt_3d(1) = (Tdata)y_bar(1);
    pt_3d(2) = 1.0;
    if (this->kUnProject2Sphere == unproject_type_) pt_3d.normalize();
  }
}

}  // namespace camm
}  // namespace VIEO_SLAM