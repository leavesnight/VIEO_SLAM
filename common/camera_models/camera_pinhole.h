//
// Created by leavesnight on 5/30/23.
//

#pragma once

#include "camera_base.h"

namespace cv {
class FileStorage;
class Mat;
}  // namespace cv

namespace VIEO_SLAM {
namespace camm {
class COMMON_API PinholeCamera : public GeometricCamera {
 protected:
  using Base = GeometricCamera;
  using Base::camera_model_;
  using Base::parameters_;
  using Base::Tcalc;
  using Base::unproject_type_;

  using typename Base::CameraModel;
  using typename Base::CamId;
  using typename Base::Mat3data;
  using typename Base::SE3data;
  using typename Base::Tsize;
  using typename Base::Vec2data;
  using typename Base::Vec3calc;

 public:
  using Ptr = std::shared_ptr<PinholeCamera>;
  using typename Base::Mat23io;
  using typename Base::Mat2Xio;
  using typename Base::Mat32io;
  using typename Base::Mat3Xio;
  using typename Base::Tdata;
  using typename Base::Tio;
  using typename Base::Vec3io;

  // parameters: fx, fy, cx, cy
  PinholeCamera(CamId camera_id, Tsize width, Tsize height, const std::vector<Tdata> &parameters,
                const SE3data &Tbc = SE3data())
      : Base(camera_id, width, height, parameters, Tbc) {
    assert(4 <= parameters_.size());
    camera_model_ = CameraModel::kPinhole;
  }
  // still has default copy constructor and we call it!
  PinholeCamera(const PinholeCamera *DerivedCamera) : PinholeCamera(*DerivedCamera) {
    assert(4 <= parameters_.size());
    camera_model_ = CameraModel::kPinhole;
  }
  PinholeCamera(cv::FileStorage &fSettings, int id, bool &bmiss_param);
  static bool ParseCamParamFile(cv::FileStorage &fSettings, int id, Camera::Ptr &pCameraInstance);

  inline void Project(const Vec3io &p_3d, Vec2data *p_img, Mat23io *d_img_d_p3d = nullptr,
                      Mat2Xio *d_img_d_param = nullptr) const override;

  inline void UnProject(const Vec2data &p_img, Vec3io *p_3d, Mat32io *d_p3d_d_img = nullptr,
                        Mat3Xio *d_p3d_d_param = nullptr) const override;

  Mat3data toK() const override {
    Mat3data K;
    K << Base::fx(), 0.f, Base::cx(), 0.f, Base::fy(), Base::cy(), 0.f, 0.f, 1.f;
    return K;
  }
};

void PinholeCamera::Project(const Vec3io &p_3d, Vec2data *p_img, Mat23io *d_img_d_p3d, Mat2Xio *d_img_d_param) const {
  const Tcalc x = (Tcalc)p_3d[0];
  const Tcalc y = (Tcalc)p_3d[1];
  const Tcalc z = (Tcalc)p_3d[2];
  const Tcalc invz = 1. / z;

  if (p_img) {
    const Tdata &fx = parameters_[0];
    const Tdata &fy = parameters_[1];
    const Tdata &cx = parameters_[2];
    const Tdata &cy = parameters_[3];
    (*p_img)[0] = (Tdata)((Tcalc)fx * x * invz + cx);
    (*p_img)[1] = (Tdata)((Tcalc)fy * y * invz + cy);
  }

  if (d_img_d_p3d) {
    const Tdata &fx = parameters_[0];
    const Tdata &fy = parameters_[1];
    d_img_d_p3d->setZero();
    const Tcalc invz2 = invz * invz;

    (*d_img_d_p3d)(0, 0) = fx * invz;
    (*d_img_d_p3d)(0, 2) = -fx * x * invz2;
    (*d_img_d_p3d)(1, 1) = fy * invz;
    (*d_img_d_p3d)(1, 2) = -fy * y * invz2;
  }

  if (d_img_d_param) {
    d_img_d_param->resize(2, 4);
    d_img_d_param->setZero();

    (*d_img_d_param)(0, 0) = x * invz;
    (*d_img_d_param)(0, 2) = 1;
    (*d_img_d_param)(1, 1) = y * invz;
    (*d_img_d_param)(1, 3) = 1;
  }
}

void PinholeCamera::UnProject(const Vec2data &p_img, Vec3io *p_3d, Mat32io *d_p3d_d_img, Mat3Xio *d_p3d_d_param) const {
  const Tdata &fx = parameters_[0];
  const Tdata &fy = parameters_[1];
  const Tdata &cx = parameters_[2];
  const Tdata &cy = parameters_[3];

  const Tcalc mx = ((Tcalc)p_img[0] - cx) / fx;
  const Tcalc my = ((Tcalc)p_img[1] - cy) / fy;

  const Tcalc r2 = mx * mx + my * my;

  const Tcalc norm = this->kUnProject2Plane == unproject_type_ ? Tcalc(1.0) : std::sqrt(Tcalc(1.0) + r2);
  const Tcalc norm_inv = Tcalc(1.0) / norm;

  if (p_3d) {
    (*p_3d)[0] = (Tio)(mx * norm_inv);
    (*p_3d)[1] = (Tio)(my * norm_inv);
    (*p_3d)[2] = (Tio)(norm_inv);
  }

  if (d_p3d_d_img || d_p3d_d_param) {
    const Tcalc d_2norm_inv_d_r2 =
        this->kUnProject2Plane == unproject_type_ ? 0 : Tcalc(-norm_inv) * norm_inv * norm_inv;

    Vec3calc c0, c1;
    c0(0) = (norm_inv + mx * mx * d_2norm_inv_d_r2) / fx;
    c0(1) = (my * mx * d_2norm_inv_d_r2) / fx;
    c0(2) = mx * d_2norm_inv_d_r2 / fx;

    c1(0) = (my * mx * d_2norm_inv_d_r2) / fy;
    c1(1) = (norm_inv + my * my * d_2norm_inv_d_r2) / fy;
    c1(2) = my * d_2norm_inv_d_r2 / fy;

    if (d_p3d_d_img) {
      d_p3d_d_img->setZero();

      d_p3d_d_img->col(0) = c0.cast<Tio>();
      d_p3d_d_img->col(1) = c1.cast<Tio>();
    }

    if (d_p3d_d_param) {
      d_p3d_d_param->resize(3, 4);
      d_p3d_d_param->setZero();

      d_p3d_d_param->col(2) = (-c0).cast<Tio>();
      d_p3d_d_param->col(3) = (-c1).cast<Tio>();
      d_p3d_d_param->col(0) = (-c0 * mx).cast<Tio>();
      d_p3d_d_param->col(1) = (-c1 * my).cast<Tio>();
    }
  }
}

}  // namespace camm
}  // namespace VIEO_SLAM