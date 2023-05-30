/**
 * This file is part of VIEO_SLAM
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImuGrabber {
 public:
  ImuGrabber(){};
  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

  queue<sensor_msgs::ImuConstPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber {
 public:
  ImageGrabber(VIEO_SLAM::System *pSLAM, ImuGrabber *pImuGb, const bool bClahe)
      : mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe) {}

  void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);
  void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);
  cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
  void SyncWithImu();

  queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;

  VIEO_SLAM::System *mpSLAM;
  ImuGrabber *mpImuGb;

  cv::Mat M1l, M2l, M1r, M2r;

  const bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if (argc < 3 || argc > 4) {
    cerr << endl << "Usage: rosrun VIEO_SLAM Stereo_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  if (argc == 4) {
    std::string sbEqual(argv[3]);
    if (sbEqual == "true") bEqual = true;
  }

  // Read rectification parameters
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  VIEO_SLAM::System SLAM(argv[1], argv[2], VIEO_SLAM::System::STEREO, true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM, &imugb, bEqual);

  if (!fsSettings["LEFT.K"].empty()) {
    // Load settings related to stereo calibration
    if (!fsSettings.isOpened()) {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
        D_r.empty() || rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
      cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
      return -1;
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F,
                                igb.M1l, igb.M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F,
                                igb.M1r, igb.M2r);
  } else
    VIEO_SLAM::System::usedistort_ = true;

  // Maximum delay, 5 seconds
  string topic_tmp = "/camera/imu";
  if (!fsSettings["IMU.topic"].empty()) topic_tmp = string(fsSettings["IMU.topic"]);
  ros::Subscriber sub_imu = n.subscribe(topic_tmp, 1000, &ImuGrabber::GrabImu, &imugb);
  topic_tmp = "/camera/infra1/image_rect_raw";
  if (!fsSettings["Camera.topic"].empty()) topic_tmp = string(fsSettings["Camera.topic"]);
  ros::Subscriber sub_img_left = n.subscribe(topic_tmp, 100, &ImageGrabber::GrabImageLeft, &igb);
  topic_tmp = "/camera/infra2/image_rect_raw";
  if (!fsSettings["Camera2.topic"].empty()) topic_tmp = string(fsSettings["Camera2.topic"]);
  ros::Subscriber sub_img_right = n.subscribe(topic_tmp, 100, &ImageGrabber::GrabImageRight, &igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  ros::spin();

  return 0;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg) {
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty()) imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg) {
  mBufMutexRight.lock();
  if (!imgRightBuf.empty()) imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0) {
    return cv_ptr->image.clone();
  } else {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

#define PRINT_TIME_COST
void ImageGrabber::SyncWithImu() {
  const double maxTimeDiff = 0.01;
  while (1) {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty()) {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec()) continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      mpImuGb->mBufMutex.lock();
      if (!mpImuGb->imuBuf.empty()) {
        // Load imu measurements from buffer
        double imu_data[7];
        while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft) {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          Eigen::Vector3d acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                              mpImuGb->imuBuf.front()->linear_acceleration.y,
                              mpImuGb->imuBuf.front()->linear_acceleration.z);
          Eigen::Vector3d gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y,
                              mpImuGb->imuBuf.front()->angular_velocity.z);
          for (int i = 0; i < 3; ++i) imu_data[i] = acc[i];
          for (int i = 0; i < 3; ++i) imu_data[i + 3] = gyr[i];
          imu_data[6] = t;
          mpSLAM->TrackOdom(imu_data[6], imu_data, (char)VIEO_SLAM::System::IMU);
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();

      if (mbClahe) {
        mClahe->apply(imLeft, imLeft);
        mClahe->apply(imRight, imRight);
      }
      bool do_rectify = !M1l.empty();
      if (do_rectify) {
        cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
      }

#ifdef PRINT_TIME_COST
#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
#endif
      mpSLAM->TrackStereo(vector<cv::Mat>({imLeft, imRight}), tImLeft, do_rectify);
#ifdef PRINT_TIME_COST
#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
      double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      static double ttrack_sum = 0;
      static size_t ttrack_num = 0;
      ++ttrack_num;
      ttrack_sum += ttrack;
      if (1 == ttrack_num % 30) PRINT_INFO("ttrack=" << ttrack << ",avg=" << ttrack_sum / ttrack_num << endl);
#endif

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}
