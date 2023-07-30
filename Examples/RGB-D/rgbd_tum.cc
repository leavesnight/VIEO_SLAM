/**
 * This file is part of VIEO_SLAM
 */
// rectified by zzh in 2017.

#include <iostream>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "System.h"
#include "common/multithread/multithreadbase.h"
#include "common/mlog/log.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

// zzh
VIEO_SLAM::System *g_pSLAM;
double g_simulateTimestamp = -1, gDelayCache;
bool g_brgbdFinished = false;
mutex g_mutex;

// a new thread simulating the odom serial threads
void odomIMURun(ifstream &finOdomdata, int totalNum, const string &settings_path = "") {  // must use &
  // bind to assigned core
#if defined(SET_AFFINITY_LINUX)
  {
    cv::FileStorage fsettings(settings_path, cv::FileStorage::READ);
    VIEO_SLAM::multithread::ThreadPolicyInfo event_info;
    const string thread_type = "ODOM";
    auto node_tmp = fsettings[thread_type + ".processor_ids"];
    size_t num_cores = sysconf(_SC_NPROCESSORS_CONF);
    event_info.affinity_mask_ = node_tmp.empty() ? ((size_t)(0x1 << num_cores) - 1) : (size_t)(int)node_tmp;
    node_tmp = fsettings[thread_type + ".priority"];
    event_info.priority_ = node_tmp.empty() ? 49 : (size_t)(int)node_tmp;
    event_info.thread_type_ = VIEO_SLAM::multithread::THREAD_ODOM;
    int priority_max_rr = sched_get_priority_max(SCHED_RR);
    if (event_info.priority_ > priority_max_rr) {
      PRINT_INFO_FILE_MUTEX("th_name=" << (int)event_info.thread_type_
                                       << ",SCHED_FIFO, priority_min/max_rr=" << sched_get_priority_min(SCHED_RR) << "/"
                                       << priority_max_rr << ",min/max_fifo=" << sched_get_priority_min(SCHED_FIFO)
                                       << "/" << sched_get_priority_max(SCHED_FIFO) << std::endl,
                            VIEO_SLAM::mlog::vieo_slam_debug_path, "alg_event.txt");
      event_info.policy_ = SCHED_FIFO;
      event_info.priority_ -= priority_max_rr;
    } else
      event_info.policy_ = SCHED_RR;
    VIEO_SLAM::multithread::SetAffinity(event_info);
  }
#endif
  PRINT_INFO_MUTEX("OdomIMUThread created!" << endl);
  // read until reading over
  int nTotalNum = 2 + 4 + 3 * 3;
  if (totalNum != 0) nTotalNum = totalNum;
  double *odomdata = new double[nTotalNum];
  double timestamp, tmstpLast = -1;

  while (!g_pSLAM) {  // if it's NULL
    usleep(1e5);      // wait 0.1s
  }
  while (!finOdomdata.eof()) {
    finOdomdata >> timestamp;
    if (finOdomdata.eof()) break;
    while (1) {  // until the image reading time is reached
      {
        unique_lock<mutex> lock(g_mutex);
        if (timestamp <= g_simulateTimestamp + gDelayCache || g_brgbdFinished) break;
      }
      usleep(1.5e4);  // allow 15ms delay
    }
    for (int i = 0; i < nTotalNum; ++i) {
      finOdomdata >> odomdata[i];
    }
    if (timestamp > tmstpLast) {  // avoid == condition
      //       if (nTotalNum>=9&&(odomdata[nTotalNum-5]>-0.5||odomdata[nTotalNum-5]<-1.5)){//ay:2+4+3+2 -1
      // 	cout<<redSTR"Wrong imu data! t: "<<timestamp<<whiteSTR<<endl;
      // 	continue;
      //       }
      // jump vl,vr,quat[4],magnetic data[3] then it's axyz,wxyz for default 15, please ensure the last 6 data is
      // axyz,wxyz
      g_pSLAM->TrackOdom(timestamp, odomdata + (nTotalNum - 6), (char)VIEO_SLAM::System::IMU);
    }
    // cout<<greenSTR<<timestamp<<whiteSTR<<endl;
    tmstpLast = timestamp;
  }
  delete[] odomdata;
  finOdomdata.close();
  cout << greenSTR "Simulation of Odom Data Reading is over." << whiteSTR << endl;
}
void odomEncRun(ifstream &finOdomdata, const string &settings_path = "") {  // must use &
  // bind to assigned core
#if defined(SET_AFFINITY_LINUX)
  {
    cv::FileStorage fsettings(settings_path, cv::FileStorage::READ);
    VIEO_SLAM::multithread::ThreadPolicyInfo event_info;
    const string thread_type = "ODOM";
    auto node_tmp = fsettings[thread_type + ".processor_ids"];
    size_t num_cores = sysconf(_SC_NPROCESSORS_CONF);
    event_info.affinity_mask_ = node_tmp.empty() ? ((size_t)(0x1 << num_cores) - 1) : (size_t)(int)node_tmp;
    node_tmp = fsettings[thread_type + ".priority"];
    event_info.priority_ = node_tmp.empty() ? 49 : (size_t)(int)node_tmp;
    event_info.thread_type_ = VIEO_SLAM::multithread::THREAD_ODOM_Enc;
    int priority_max_rr = sched_get_priority_max(SCHED_RR);
    if (event_info.priority_ > priority_max_rr) {
      PRINT_INFO_FILE_MUTEX("th_name=" << (int)event_info.thread_type_
                                       << ",SCHED_FIFO, priority_min/max_rr=" << sched_get_priority_min(SCHED_RR) << "/"
                                       << priority_max_rr << ",min/max_fifo=" << sched_get_priority_min(SCHED_FIFO)
                                       << "/" << sched_get_priority_max(SCHED_FIFO) << std::endl,
                            VIEO_SLAM::mlog::vieo_slam_debug_path, "alg_event.txt");
      event_info.policy_ = SCHED_FIFO;
      event_info.priority_ -= priority_max_rr;
    } else
      event_info.policy_ = SCHED_RR;
    VIEO_SLAM::multithread::SetAffinity(event_info);
  }
#endif
  PRINT_INFO_MUTEX("OdomEncThread created!" << endl);
  // read until reading over
  int nTotalNum = 2;
  double *odomdata = new double[nTotalNum];
  double timestamp, tmstpLast = -1;
  string strTmp;

  while (!g_pSLAM) {  // if it's NULL
    usleep(1e5);      // wait 0.1s
  }
  while (!finOdomdata.eof()) {
    finOdomdata >> timestamp;
    if (finOdomdata.eof()) break;
    while (1) {  // until the image reading time is reached
      {
        unique_lock<mutex> lock(g_mutex);
        if (timestamp <= g_simulateTimestamp + gDelayCache || g_brgbdFinished) break;
      }
      usleep(1.5e4);  // allow 15ms delay
    }
    for (int i = 0; i < nTotalNum; ++i) {
      finOdomdata >> odomdata[i];
    }
    getline(finOdomdata, strTmp);
    if (timestamp > tmstpLast) {                                                  // avoid == condition
      g_pSLAM->TrackOdom(timestamp, odomdata, (char)VIEO_SLAM::System::ENCODER);  // nTotalNum=2
    }
    // cout<<greenSTR<<timestamp<<whiteSTR<<endl;
    tmstpLast = timestamp;
  }
  delete[] odomdata;
  finOdomdata.close();
  cout << greenSTR "Simulation of Odom Data Reading is over." << whiteSTR << endl;
}
// zzh over

int main(int argc, char **argv) {
  char bMode = 0;                   // 0 for RGBD(pure/VIO/VEO/VIEO), 1 for MonocularVIO, 2 for Monocular
  bool bodoms[2] = {false, false};  // IMU/Enc
  thread *pOdomThread[2] = {nullptr};
  ifstream finOdomdata[2], *pfinOdomdata = &finOdomdata[0];
  int totalNum = 2;
  cout << fixed << setprecision(6) << endl;
  string map_sparse_name = "";

  switch (argc) {
    case 5:
      break;
    case 10:
      // Map Reuse RGBD
      map_sparse_name = argv[9];
    case 9: {
      finOdomdata[1].open(argv[8]);
      if (finOdomdata[1].is_open()) {
        string strTmp;
        getline(finOdomdata[1], strTmp);
        getline(finOdomdata[1], strTmp);
        getline(finOdomdata[1], strTmp);
      }
    }
    case 8:
      bMode = atoi(argv[7]);
      if (bMode == 2) break;
    case 7:
      totalNum = atoi(argv[6]);
    case 6: {
      if (totalNum == 2) {
        bodoms[1] = true;
        pfinOdomdata = &finOdomdata[1];
        if (finOdomdata[1].is_open()) finOdomdata[1].close();
      } else {
        bodoms[0] = true;
        if (finOdomdata[1].is_open()) bodoms[1] = true;
      }
      pfinOdomdata->open(argv[5]);
      if (!pfinOdomdata->is_open()) {
        cerr << redSTR "Please check the last path_to_odometryData" << endl;
        bodoms[0] = bodoms[1] = false;
        break;
      }
      string strTmp;
      getline(*pfinOdomdata, strTmp);
      getline(*pfinOdomdata, strTmp);
      getline(*pfinOdomdata, strTmp);  // odom.txt should have 3 unused lines
    } break;
    default:
      cerr << endl
           << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
      cerr << redSTR
          "Or: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_odometryData"
          " (number of odometryData) (Mode) (path_to_EncData)"
          " (path_to_Map)"
           << endl;
      return 1;
  }

  cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);  // already checked in System() creator
  cv::FileNode node_tmp = fSettings["Camera.delayForPolling"];
  if (node_tmp.empty()) {
    gDelayCache = 0;
  } else {
    gDelayCache = (double)node_tmp;
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenamesRGB;
  vector<string> vstrImageFilenamesD;
  vector<double> vTimestamps;
  string strAssociationFilename = string(argv[4]);
  LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if (vstrImageFilenamesRGB.empty()) {
    cerr << endl << "No images found in provided path." << endl;
    return 1;
  } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
    cerr << endl << "Different number of images for rgb and depth." << endl;
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  VIEO_SLAM::System::eSensor sensor = VIEO_SLAM::System::RGBD;
  if (bMode != 0) sensor = VIEO_SLAM::System::MONOCULAR;
  VIEO_SLAM::System SLAM(argv[1], argv[2], sensor, true, map_sparse_name);
  bool bLoaded = false;
  g_pSLAM = &SLAM;
  if (bodoms[0]) pOdomThread[0] = new thread(&odomIMURun, ref(finOdomdata[0]), totalNum, argv[2]);  // must use ref()
  if (bodoms[1]) pOdomThread[1] = new thread(&odomEncRun, ref(finOdomdata[1]), argv[2]);
  //    cin.get();

  node_tmp = fSettings["Camera.usedistort"];
  if (!node_tmp.empty()) {
    VIEO_SLAM::System::usedistort_ = (bool)(int)node_tmp;
  }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  PRINT_INFO_MUTEX(endl << "-------" << endl);
  PRINT_INFO_MUTEX("Start processing sequence ..." << endl);
  PRINT_INFO_MUTEX("Images in the sequence: " << nImages << endl << endl);

  // Main loop
  cv::Mat imRGB, imD;
  for (int ni = 0; ni < nImages; ni++) {
    // Read image and depthmap from file
    imRGB = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
    imD = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];
    {  // zzh
      unique_lock<mutex> lock(g_mutex);
      g_simulateTimestamp = tframe;  // update g_simulateTimestamp
    }

    if (imRGB.empty()) {
      cerr << endl << "Failed to load image at: " << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
      return 1;
    }

#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the image to the SLAM system
    if (bMode == 0)
      SLAM.TrackStereo(vector<cv::Mat>({imRGB, imD}), tframe);
    else
      SLAM.TrackStereo(vector<cv::Mat>{imRGB}, tframe);

      // double data[9]={0.5,0.4};
      // SLAM.SaveFrame("./test_save/",imRGB,imD,tframe);

#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1)
      T = vTimestamps[ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vTimestamps[ni - 1];

    if (ttrack < T) usleep((T - ttrack) * 1e6);
  }

  // zzh
  {
    unique_lock<mutex> lock(g_mutex);
    g_brgbdFinished = true;
  }
  if (SLAM.MapChanged()) {
    cout << "Map is changing!Please enter s to stop!" << endl;
    while (cin.get() != 's') {
      usleep((__useconds_t)1e6);
    }
  }
  // zzh over

  // Stop all threads
  SLAM.Shutdown();

  // zzh: FinalGBA, this is just the FullBA column in the paper! see "full BA at the end of the execution" in V-B of the
  // VIORBSLAM paper! load if Full BA just after IMU Initialize
  cv::FileNode fnFBA = fSettings["GBA.finalIterations"];
  SLAM.SaveTrajectoryTUM("CameraTrajectory_NO_FULLBA.txt");
  //  SLAM.SaveMap("MapTmp.bin",false,true,true);
  if (!fnFBA.empty() && !bLoaded) {
    if ((int)fnFBA) {
      SLAM.FinalGBA(fnFBA);
      cout << azureSTR "Execute FullBA at the end!" << whiteSTR << endl;
    }
  } else {
    cout << redSTR "No FullBA at the end!" << whiteSTR << endl;
  }
  //  SLAM.LoadMap("MapTmp.bin",false,true);

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  PRINT_INFO_MUTEX("-------" << endl << endl);
  PRINT_INFO_MUTEX("median tracking time: " << vTimesTrack[nImages / 2] << endl);
  PRINT_INFO_MUTEX("mean tracking time: " << totaltime / nImages << endl);

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameTrajectoryIMU.txt");
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
  if (map_sparse_name == "")
    SLAM.SaveMap("Map.pcd", true);  // for PCL Map
  else
    SLAM.SaveMap(map_sparse_name);  // for Reused Sparse Map

  // wait for pOdomThread finished
  for (int i = 0; i < sizeof(pOdomThread) / sizeof(pOdomThread[0]); ++i)
    if (pOdomThread[i]) pOdomThread[i]->join();
  return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps) {
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while (!fAssociation.eof()) {
    string s;
    getline(fAssociation, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t, t2;
      string sRGB, sD;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t2;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
      /*char ch[30];//at least 22+6=28
      sprintf(ch,"depth/%.6f.png",t);
      vstrImageFilenamesD.push_back(ch);*/
    }
  }
}
