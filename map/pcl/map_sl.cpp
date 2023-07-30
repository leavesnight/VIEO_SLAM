//
// Created by leavesnight on 6/26/23.
//

#include "map_sl.h"
#include "System.h"
#include "KeyFrame.h"
#include "common/mlog/log.h"
#ifdef WINDOWS
// for boost(used by pcl) compile problem on windows
#define BOOST_USE_WINDOWS_H
#endif
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// transformPC
//#include <pcl/common/transforms.h>
// filter
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>//use the z direction filed filter
#include <pcl/filters/statistical_outlier_removal.h>

namespace VIEO_SLAM {
namespace pcl {
void SaveMapPCL(const string &filename, int sensor, Map *pmap, cv::FileStorage &fsettings) {
  if (System::RGBD != sensor) {
    PRINT_INFO_MUTEX("Unsupported sensor to " << __FUNCTION__ << endl);
    return;
  }
  // typedef
  typedef ::pcl::PointXYZRGB PointT;

  typedef ::pcl::PointCloud<PointT> PointCloud;
  PRINT_INFO_MUTEX(endl << "Saving keyframe map to " << filename << " ..." << endl);

  vector<KeyFrame *> vpKFs = pmap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin???here it's at the origin
  // cv::Mat Two = vpKFs[0]->GetPoseInverse();

  PointCloud::Ptr pPC(new PointCloud);
  // vector<Eigen::Isometry3d*> poses;
  double fx = fsettings["Camera.fx"], fy = fsettings["Camera.fy"], cx = fsettings["Camera.cx"],
         cy = fsettings["Camera.cy"];
  double depthScale = fsettings["DepthMapFactor"];
  for (size_t i = 0; i < vpKFs.size(); i += 2) {
    KeyFrame *pKF = vpKFs[i];
    // pKF->SetPose(pKF->GetPose()*Two);
    if (pKF->isBad()) continue;

    // cv::Mat R = pKF->GetRotation().t();
    // vector<float> q = Converter::toQuaternion(R);
    // cv::Mat t = pKF->GetCameraCenter();
    // f << setprecision(6) << pKF->ftimestamp_ << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) <<
    // " " << t.at<float>(2)
    //<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    cv::Mat cvTwc = pKF->GetPoseInverse();
    Eigen::Matrix3d r = Converter::toMatrix3d(cvTwc.colRange(0, 3).rowRange(0, 3));
    Eigen::Isometry3d *Twc = new Eigen::Isometry3d(r);
    (*Twc)(0, 3) = cvTwc.at<float>(0, 3);
    (*Twc)(1, 3) = cvTwc.at<float>(1, 3);
    (*Twc)(2, 3) = cvTwc.at<float>(2, 3);
    // poses.push_back(Twc);]

    PointCloud::Ptr current(new PointCloud);
    if (2 != pKF->imgs_dense_.size()) {
      PRINT_ERR_MUTEX("Some imgs_dense size not right! sz= " << pKF->imgs_dense_.size() << ",id=" << pKF->nid_ << endl);
      return;
    }
    cv::Mat color = pKF->imgs_dense_[0];
    cv::Mat depth = pKF->imgs_dense_[1];
    Eigen::Isometry3d T = *(Twc);
    for (int v = 0; v < color.rows; ++v)
      for (int u = 0; u < color.cols; ++u) {
        float d = depth.ptr<unsigned short>(v)[u] * 1.0 / depthScale;
        if (d == 0 || d > 7) continue;
        Eigen::Vector3d point;
        point[2] = d;
        point[0] = (u - cx) * point[2] / fx;
        point[1] = (v - cy) * point[2] / fy;
        Eigen::Vector3d pointWorld = T * point;

        PointT p;
        p.x = pointWorld[0];
        p.y = pointWorld[1];
        p.z = pointWorld[2];
        p.b = color.data[v * color.step + u * color.channels()];
        p.g = color.data[v * color.step + u * color.channels() + 1];
        p.r = color.data[v * color.step + u * color.channels() + 2];
        current->points.push_back(p);
      }
    // depth filter and statistical removal
    /*
    PointCloud::Ptr pTmp(new PointCloud);
    ::pcl::StatisticalOutlierRemoval<PointT> statis_filter;  // this one costs lots of time!!!
    statis_filter.setMeanK(50);  // the number of the nearest points used to calculate the mean neighbor distance
    // the standart deviation multiplier,here just use 70% for the normaldistri.
    statis_filter.setStddevMulThresh(1.0);
    statis_filter.setInputCloud(current);
    statis_filter.filter(*pTmp);
    *pPC += *pTmp;*/
    *pPC += *current;
  }
  pPC->is_dense = false;  // it contains nan data
  cout << "PC has " << pPC->size() << " points" << endl;

  // voxel filter, to make less volume
  ::pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setLeafSize(0.05, 0.05, 0.05);  // 1cm^3 resolution;now 5cm
  PointCloud::Ptr pTmp(new PointCloud);
  voxel_filter.setInputCloud(pPC);
  voxel_filter.filter(*pTmp);
  // pTmp->swap(*pPC);

  // statistical filter, to eliminate the single points
  ::pcl::StatisticalOutlierRemoval<PointT> statis_filter;  // this one costs lots of time!!!
  statis_filter.setMeanK(50);  // the number of the nearest points used to calculate the mean neighbor distance
  statis_filter.setStddevMulThresh(1.0);  // the standart deviation multiplier,here just use 70% for the normal distri.
  statis_filter.setInputCloud(pTmp);
  statis_filter.filter(*pPC);

  cout << "after downsampling, it has " << pPC->size() << " points" << endl;
  ::pcl::io::savePCDFileBinary(filename, *pPC);

  PRINT_INFO_MUTEX(endl << "Map saved!" << endl);
}

}  // namespace pcl
}  // namespace VIEO_SLAM