/**
 * This file is part of VIEO_SLAM.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>
#include <dirent.h>
#include <libgen.h>

#include <rapidjson/document.h>

using namespace std;
using namespace rapidjson;

//#define DEBUG_STRATEGY

void AlignImgs(vector<vector<double>> &vtmcam, vector<vector<string>> &vstrimg) {
  CV_Assert(vtmcam.size() == vstrimg.size());
  if (!vtmcam.size()) return;
  CV_Assert(vtmcam[0].size() == vstrimg[0].size());
  size_t n_cams = vtmcam.size();
  size_t i = 0;
  const double synch_allow = 0.010;
  while (i < vtmcam[0].size()) {
    bool bplus = true;
    // cout << "tm0:" << vtmcam[0][i] << " ";
    for (size_t j = 1; j < n_cams; ++j) {
      while (vtmcam[j].size() > i && (vtmcam[j][i] < vtmcam[0][i] - synch_allow || vtmcam[0].size() <= i)) {
        vtmcam[j].erase(vtmcam[j].begin() + i);
        vstrimg[j].erase(vstrimg[j].begin() + i);
      }
      if (vtmcam[j].size() <= i) {
        CV_Assert(i == vtmcam[j].size());
        if (vtmcam[0].size() > i) {
          vtmcam[0].resize(i);
          vstrimg[0].resize(i);
        }
        continue;
      }
      if (vtmcam[j][i] > vtmcam[0][i] + synch_allow) {
        vtmcam[0].erase(vtmcam[0].begin() + i);
        vstrimg[0].erase(vstrimg[0].begin() + i);
        bplus = false;
      }
      // cout << "tm" << j << ":" << vtmcam[j][i] << " ";
    }
    // cout << endl;
    if (bplus) ++i;
  }
  auto n0 = vtmcam[0].size();
  for (size_t j = 0; j < n_cams; ++j) {
    CV_Assert(vtmcam[j].size() >= n0);
    CV_Assert(vstrimg[j].size() == vtmcam[j].size());
    vtmcam[j].resize(n0);
    vstrimg[j].resize(n0);
    CV_Assert(vstrimg[j].size() == n0);
  }
  cout << "After align img size=" << vtmcam[0].size() << endl;
}

static void GetFileNames(const string &path, vector<string> &filenames, const string &suffix = ".pgm",
                         const string &prefix = "");

void LoadImages(const string &strImagePath, vector<string> &vstrImages, vector<double> &vTimeStamps,
                const string &suffix = ".pgm");

void LoadIMU(const vector<string> &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc,
             vector<cv::Point3f> &vGyro);

VIEO_SLAM::System *g_pSLAM;
double g_simulateTimestamp = -1, gDelayCache;
bool g_brgbdFinished = false;
mutex g_mutex;

// a new thread simulating the odom serial threads
void odomRun(vector<double> &vTimestampsImu, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro) {  // must use &
  double timestamp, tmstpLast = -1;

  while (!g_pSLAM) {  // if it's NULL
    usleep(15000);    // wait 15ms
  }
  double odomdata[6];  // axyz,wxyz
  for (int line = 0; line < vTimestampsImu.size(); ++line) {
    timestamp = vTimestampsImu[line];
    while (1) {  // until the image reading time is reached
      {
        unique_lock<mutex> lock(g_mutex);
        if (timestamp <= g_simulateTimestamp + gDelayCache || g_brgbdFinished) break;
      }
      usleep(1000);  // allow 1ms delay
    }
    odomdata[0] = vAcc[line].x;
    odomdata[1] = vAcc[line].y;
    odomdata[2] = vAcc[line].z;
    odomdata[3] = vGyro[line].x;
    odomdata[4] = vGyro[line].y;
    odomdata[5] = vGyro[line].z;
    // for (int i=0;i<6;++i) cout<<odomdata[i]<<" ";cout<<endl;
    if (timestamp > tmstpLast)                                                // avoid == condition
      g_pSLAM->TrackOdom(timestamp, odomdata, (char)VIEO_SLAM::System::IMU);  // for EuRoC dataset
    // cout<<green<<timestamp<<whiteSTR<<endl;
    tmstpLast = timestamp;
  }
  PRINT_INFO_MUTEX(greenSTR "Simulation of Odom Data Reading is over." << whiteSTR << endl);
}

void AutoFillParamsFromDS(cv::FileStorage &fSettings, const string& pathCalib, string &tmp_configfile_path,
                          bool mute = true);

int main(int argc, char **argv) {
  thread *pOdomThread = NULL;
  int totalNum = 0;
  PRINT_INFO_MUTEX(fixed << setprecision(6) << endl);

  const int num_seq = argc < 3 ? 1 : argc - 3;
  int seq = 0;
  string pathSeq(argv[(seq) + 3]);
  vector<vector<vector<cv::Point3f>>> vAcc, vGyro;
  vector<vector<vector<double>>> vTimestampsImu;
  vAcc.resize(num_seq, vector<vector<cv::Point3f>>(1));
  vGyro.resize(num_seq, vector<vector<cv::Point3f>>(1));
  vTimestampsImu.resize(num_seq, vector<vector<double>>(1));

  int dataset_type = 0;
  string mode = "VIO";
  switch (argc) {
    case 5:
      mode = argv[4];
      if ("VIO" != mode) break;
    case 4: {
      vector<string> pathImu = {pathSeq + "/Sensors/gyroscope.xml", pathSeq + "/Sensors/accelerometer.xml"};
      auto &vacc = vAcc[seq], &vgyr = vGyro[seq];
      auto &vtmimu = vTimestampsImu[seq];
      LoadIMU(pathImu, vtmimu[0], vacc[0], vgyr[0]);
      if (vtmimu[0].empty()) {
        int n_imu_max = 3;
        vtmimu.resize(n_imu_max);
        vacc.resize(n_imu_max);
        vgyr.resize(n_imu_max);
        pathImu.resize(1);
        for (int i = 0; i < n_imu_max; ++i) {
          pathImu[0] = pathSeq + "/IMU" + to_string(i) + "/data.json";
          LoadIMU(pathImu, vtmimu[i], vacc[i], vgyr[i]);
          if (vtmimu[i].empty()) {
            vtmimu.resize(i);
            vacc.resize(i);
            vgyr.resize(i);
            break;
          }
        }
      }
      PRINT_INFO_MUTEX("IMU size=" << vtmimu[0].size() << endl);
      if (!vtmimu[0].size()) {
        cerr << redSTR "Please check the last path_to_odometryFolder" << endl;
        return -1;
      }
      string strTmp;
      pOdomThread = new thread(&odomRun, ref(vtmimu[0]), ref(vacc[0]), ref(vgyr[0]));
      PRINT_INFO_MUTEX("OdomThread created!" << endl);
    } break;
    default:
      cerr << endl << "Usage: ./stereo_vr path_to_vocabulary path_to_settings path_to_folder (VIO)" << endl;
      return 1;
  }
  if (mode != "VIO" && mode != "VO") {
    cerr << "unsupported mode " << mode << "now" << endl;
    CV_Assert(0);
  }

  cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);  // already checked in System() creator
  if (!fSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }
  cv::FileNode fnDelay = fSettings["Camera.delayForPolling"];
  if ("VIO" == mode) {
    cv::FileNode fnimumode = fSettings["IMU.mode"];
    int mode_imu = fnimumode.empty() ? 0 : (int)fnimumode;
    if (1 == (int)mode_imu) {  // 1 menas -gy,gxgz/-ay,axaz
      for (int seq = 0; seq < num_seq; ++seq) {
        auto &vacc = vAcc[seq], &vgyr = vGyro[seq];
        for (int i = 0; i < vacc.size(); ++i)
          for (int j = 0; j < vacc[i].size(); ++j) {
            swap(vacc[i][j].x, vacc[i][j].y);
            vacc[i][j].y = -vacc[i][j].y;
          }
        for (int i = 0; i < vgyr.size(); ++i)
          for (int j = 0; j < vgyr[i].size(); ++j) {
            swap(vgyr[i][j].x, vgyr[i][j].y);
            vgyr[i][j].y = -vgyr[i][j].y;
          }
      }
    }
  }
  if (fnDelay.empty()) {
    gDelayCache = 0;
  } else {
    gDelayCache = (double)fnDelay;
  }
  string tmp_configfile_path = argv[2];
  auto fncalib_type = fSettings["Calib.type"];
  if (!fncalib_type.empty()) {
    if ((int)fncalib_type) {
      AutoFillParamsFromDS(fSettings, pathSeq + "/device_calibration_imu.xml", tmp_configfile_path);
    } else {
      AutoFillParamsFromDS(fSettings, pathSeq + "/device_calibration.xml", tmp_configfile_path);
    }
  }

  // Retrieve paths to images
  vector<vector<vector<string>>> vstrImages;
  vector<vector<vector<double>>> vTimestampsCam;
  vstrImages.resize(num_seq, vector<vector<string>>(1));
  vTimestampsCam.resize(num_seq, vector<vector<double>>(1));
  string pathCam0 = pathSeq + "/Camera8";
  auto &vstrimg = vstrImages[seq];
  auto &vtmcam = vTimestampsCam[seq];
  LoadImages(pathCam0, vstrimg[0], vtmcam[0]);
  if (vtmcam[0].empty()) {
    dataset_type = 1;
    int n_cams_max = 4;
    vstrimg.resize(n_cams_max);
    vtmcam.resize(n_cams_max);
    for (int i = 0; i < n_cams_max; ++i) {
      pathCam0 = pathSeq + "/Camera" + to_string(i) + "/images";
      LoadImages(pathCam0, vstrimg[i], vtmcam[i], ".bmp");
      if (vtmcam[i].empty()) {
        vtmcam.resize(i);
        vstrimg.resize(i);
        break;
      }
    }
  }
  AlignImgs(vtmcam, vstrimg);
  PRINT_INFO_MUTEX("Img size=" << vtmcam[0].size() << endl);

  if (vstrImages[0].empty()) {
    cerr << "ERROR: No images in provided path." << endl;
    return 1;
  }

  int nImages = vstrimg[0].size(), nImagesUsed = 0;

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  VIEO_SLAM::System SLAM(argv[1], tmp_configfile_path, VIEO_SLAM::System::STEREO, true);
  g_pSLAM = &SLAM;

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages, 0);

  PRINT_INFO_MUTEX(endl << "-------" << endl);
  PRINT_INFO_MUTEX("Start processing sequence ..." << endl);
  PRINT_INFO_MUTEX("Images in the sequence: " << nImages << endl << endl);

  // Main loop
  vector<cv::Mat> ims(2);
  cv::FileNode fnfps = fSettings["Camera.fps"];
  int fpsrat = 1;
  if (!fnfps.empty() && nImages > 1) {
    double fps = (double)fnfps;
    double fpsreal = vtmcam[0].size() / (vtmcam[0].back() - vtmcam[0].front());
    fpsrat = (int)(fpsreal / fps + 0.5);
    if (fpsrat < 1) fpsrat = 1;
    PRINT_INFO_MUTEX("fps ratio: " << fpsrat << endl);
  }
  auto &vtmimu = vTimestampsImu[seq];
  if (vtmimu[0].size()) {
    while (vtmcam[0][nImages - 1] > vtmimu[0].back()) {
      --nImages;
    }
  }
  for (int ni = 0; ni < nImages; ni += fpsrat) {
    // Read left and right images from file
    ims[0] = cv::imread(vstrimg[0][ni], cv::IMREAD_GRAYSCALE);
    if (!dataset_type) {
      ims[1] = ims[0].colRange(ims[0].cols / 2, ims[0].cols);
      ims[0] = ims[0].colRange(0, ims[0].cols / 2);
    } else {
      cv::FileNode fncam3 = fSettings["Camera4.fx"];
      // if (vtmcam[3][ni] != vtmcam[0][ni]) break;
      //      CV_Assert(vtmcam[3][ni] == vtmcam[0][ni]);
      if (fncam3.empty()) {
        ims[1] = cv::imread(vstrimg[3][ni], cv::IMREAD_GRAYSCALE);
      } else {
        ims.resize(vstrimg.size());
        for (int i = 1; i < ims.size(); ++i) ims[i] = cv::imread(vstrimg[i][ni], cv::IMREAD_GRAYSCALE);
      }
    }

    for (int i = 0; i < ims.size(); ++i) {
      if (ims[i].empty()) {
        cerr << endl << "Failed to load image at: " << string(vstrimg[i][ni]) << endl;
        return 1;
      }
    }

    double tframe = vtmcam[0][ni];
    {  // zzh
      unique_lock<mutex> lock(g_mutex);
      g_simulateTimestamp = tframe;  // update g_simulateTimestamp
    }

#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the images to the SLAM system
    SLAM.TrackStereo(ims, tframe);

#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1)
      T = vtmcam[0][ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vtmcam[0][ni - 1];
    T *= fpsrat;

    if (ttrack < T) usleep((T - ttrack) * 1e6);
    ++nImagesUsed;
  }

  // zzh
  {
    unique_lock<mutex> lock(g_mutex);
    g_brgbdFinished = true;
  }
  if (SLAM.MapChanged()) {
    PRINT_INFO_MUTEX("Map is changing!Please enter s to stop!" << endl);
#ifdef DEBUG_STRATEGY
    while (cin.get() != 's') {
      sleep(1);
    }
#endif
    sleep(5);
  }
  // zzh over

  // Stop all threads
  SLAM.Shutdown();

  // zzh: FinalGBA, this is just the FullBA column in the paper! see "full BA at the end of the execution" in V-B of the
  // VIORBSLAM paper! load if Full BA just after IMU Initialized
  cv::FileNode fnFBA = fSettings["GBA.finalIterations"];
  SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameTrajectoryIMU_NO_FULLBA.txt");
  //     SLAM.SaveMap("KeyFrameTrajectoryMap.bin",false);
  if (!fnFBA.empty()) {
    if ((int)fnFBA) {
      SLAM.FinalGBA(fnFBA);
      PRINT_INFO_MUTEX(azureSTR "Execute FullBA at the end!" << whiteSTR << endl);
    }
  } else {
    PRINT_INFO_MUTEX(redSTR "No FullBA at the end!" << whiteSTR << endl);
  }

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  PRINT_INFO_MUTEX("-------" << endl << endl);
  PRINT_INFO_MUTEX("mean tracking time: " << totaltime / nImagesUsed << endl);
  PRINT_INFO_MUTEX("max tracking time: " << vTimesTrack.back() << endl);

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameTrajectoryIMU.txt");
  const bool bgravity_as_w = fSettings["Output.gravity_as_w"].empty() ? false : (int)fSettings["Output.gravity_as_w"];
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt", bgravity_as_w);
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt", 0, bgravity_as_w);
  SLAM.SaveTrajectoryTUM("CameraTrajectoryCamPoseIMUBias.txt", 1);

  return 0;
}

static void GetFileNames(const string &path, vector<string> &filenames, const string &suffix, const string &prefix) {
  DIR *pDir;
  struct dirent *ptr;
  if (!(pDir = opendir(path.c_str()))) {
    cerr << path << " opendir failed" << endl;
    return;
  }

  while ((ptr = readdir(pDir)) != nullptr) {
    if (strcmp(ptr->d_name, ".") && strcmp(ptr->d_name, "..")) {
      string d_name = string(ptr->d_name);
      size_t pos_suffix = d_name.rfind(suffix), pos_prefix = 0;
      if (!prefix.empty()) pos_prefix = d_name.find(prefix);
      if (string::npos != pos_suffix && pos_suffix + suffix.length() == d_name.length()) {
        if (!pos_prefix) filenames.push_back(path + "/" + ptr->d_name);
      }
    }
  }
  closedir(pDir);
}

void LoadImages(const string &strImagePath, vector<string> &vstrImages, vector<double> &vTimeStamps,
                const string &suffix) {
  ifstream fTimes;
  GetFileNames(strImagePath, vstrImages, suffix);
  constexpr double CoeffNanoSecond = 1e-9;
  sort(vstrImages.begin(), vstrImages.end(), [](const string &im1, const string &im2) {
    string dir_path = dirname(strdup(im1.c_str()));
    int offset = dir_path.length();
    if (dir_path[offset - 1] != '/' and dir_path[offset - 1] != '\\') ++offset;
    double ftmp[2] = {strtod(im1.substr(offset).c_str(), 0) * CoeffNanoSecond,
                      strtod(im2.substr(offset).c_str(), 0) * CoeffNanoSecond};
    return ftmp[0] < ftmp[1];
  });
  for (int i = 0; i < vstrImages.size(); ++i) {
    string dir_path = dirname(strdup(vstrImages[i].c_str()));
    int offset = dir_path.length();
    if (dir_path[offset - 1] != '/' and dir_path[offset - 1] != '\\') ++offset;
    double ftmp = strtod(vstrImages[i].substr(offset).c_str(), 0) * CoeffNanoSecond;
    vTimeStamps.push_back(ftmp);
  }
}

typedef enum KeyStrType { kStrStart, kStrEnd, kStrDivide, kNumKeyStrType };

static int GetFloatArray(const string &str_tmp, const string *keystr, size_t &last_pos, vector<double> &ret_vals) {
  size_t pos_keystr = str_tmp.find(keystr[kStrStart], last_pos);
  if (keystr[kStrStart] == "") pos_keystr = last_pos;
  if (string::npos != pos_keystr) {
    last_pos = pos_keystr + keystr[kStrStart].length();
    if (keystr[kStrEnd] == "")
      pos_keystr = str_tmp.length();
    else {
      pos_keystr = str_tmp.find(keystr[kStrEnd], last_pos);
    }
    string str_data = str_tmp.substr(last_pos, pos_keystr - last_pos);
    char *endptr = 0;
    last_pos = 0;
    while (last_pos < str_data.length()) {
      string str_data_tmp = str_data.substr(last_pos);
      ret_vals.push_back(strtod(str_data_tmp.c_str(), &endptr));
      last_pos += endptr - str_data_tmp.c_str() + keystr[kStrDivide].length();
    }
    last_pos = pos_keystr + keystr[kStrEnd].length();

    return 0;
  }

  return -1;
}

void LoadIMU(const vector<string> &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc,
             vector<cv::Point3f> &vGyro) {
  ifstream fImu, fAcc;
  fImu.open(strImuPath[0].c_str());
  if (!fImu.is_open()) return;
  int mode = 0;
  if (strImuPath.size() > 1) {
    fAcc.open(strImuPath[1].c_str());
    if (!fAcc.is_open()) return;
  } else
    mode = 1;
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);
  vector<double> ret_vals;

  while (!fImu.eof()) {
    string s;
    getline(fImu, s);

    if (!mode) {
      string keystr[kNumKeyStrType] = {"Data x='", "' time", "' *='"};
      size_t last_pos = 0;
      if (!GetFloatArray(s, keystr, last_pos, ret_vals)) {
        CV_Assert(3 == ret_vals.size());
        vGyro.push_back(cv::Point3f(ret_vals[0], ret_vals[1], ret_vals[2]));
        ret_vals.clear();
      }

      keystr[kStrStart] = "stamp='";
      keystr[kStrEnd] = "' index";
      if (!GetFloatArray(s, keystr, last_pos, ret_vals)) {
        CV_Assert(1 == ret_vals.size());
        vTimeStamps.push_back(ret_vals[0] / 1e9);
        ret_vals.clear();
      }
    } else {
      rapidjson::Document imu_doc;
      imu_doc.Parse<0>(s.c_str());
      Value &IMUData = imu_doc["Sequence"]["Dataset"]["Data"];
      for (std::size_t i = 0; i < IMUData.Size(); i++) {
        vTimeStamps.push_back(IMUData[i]["timestamp"].GetUint64() / 1.e9);
        vGyro.push_back(
            cv::Point3f(IMUData[i]["g_x"].GetFloat(), IMUData[i]["g_y"].GetFloat(), IMUData[i]["g_z"].GetFloat()));
        vAcc.push_back(
            cv::Point3f(IMUData[i]["a_x"].GetFloat(), IMUData[i]["a_y"].GetFloat(), IMUData[i]["a_z"].GetFloat()));
        // cout << "check
        // tm="<<vTimeStamps.back()<<",ga="<<vGyro.back().x<<","<<vGyro.back().y<<","<<vGyro.back().z<<"/"<<vAcc.back().x<<","<<vAcc.back().y<<","<<vAcc.back().z<<endl;
      }
    }
  }
  if (mode) return;

  int id_acc = 0;
  while (!fAcc.eof() && !fAcc.fail()) {
    string s;
    getline(fAcc, s);

    string keystr[kNumKeyStrType] = {"Data x='", "' time", "' *='"};
    size_t last_pos = 0;
    if (!GetFloatArray(s, keystr, last_pos, ret_vals)) {
      CV_Assert(3 == ret_vals.size());
      vAcc.push_back(cv::Point3f(ret_vals[0], ret_vals[1], ret_vals[2]));
      ret_vals.clear();
    }

    keystr[kStrStart] = "stamp='";
    keystr[kStrEnd] = "' index";
    if (!GetFloatArray(s, keystr, last_pos, ret_vals)) {
      CV_Assert(1 == ret_vals.size());
      if (vTimeStamps.size() <= id_acc) {
        break;
      }
      CV_Assert(vTimeStamps[id_acc++] == ret_vals[0] / 1e9);
      ret_vals.clear();
    }
    if (vTimeStamps.size() <= id_acc) {
      vAcc.resize(vTimeStamps.size());
      break;
    }
  }
  if (vAcc.size() < vTimeStamps.size()) {
    vTimeStamps.resize(vAcc.size());
    vGyro.resize(vAcc.size());
  }
}

class exfstream : public fstream {
  using Base = fstream;

 public:
  exfstream(string path, ios::openmode mode) : Base(path, mode) { (*this) << "%YAML:1.0" << endl; }
  template <class type>
  void write_yaml(const string &key, const type &data) {
    auto &fs = *this;
    fs << key << ": ";
    if (typeid(double) == typeid(type)) {
      // auto oldflags = fs.flags();
      fs.setf(ios::showpoint);
      fs.precision(10);
      fs << data << endl;
      // fs.unsetf(ios::showpoint);
      // fs.flags(oldflags);
    } else if (typeid(int) == typeid(type)) {
      fs << data << endl;
    } else if (typeid(string) == typeid(type)) {
      fs << "\"" << data << "\"" << endl;
    } else {
      cerr << "Unknown type: " << typeid(data).name() << endl;
      return;
    }
  }
};
template<>
void exfstream::write_yaml(const string &key, const Eigen::VectorXd &data) {
  auto &fs = *this;
  CV_Assert(data.size());
  fs << key << ": ";
  fs << "[";
  size_t ilast = data.size() - 1;
  fs.setf(ios::showpoint);
  fs.precision(9);
  for (size_t i = 0; i < ilast; ++i) {
    fs << data[i] << ", ";
  }
  fs << data[ilast];
  fs << "]" << endl;
}
template<>
void exfstream::write_yaml(const string &key, const cv::Mat &data) {
  auto &fs = *this;
  CV_Assert(!data.empty());
  fs << key << ": !!opencv-matrix" << endl;
  fs << "  rows: " << data.rows << endl;
  fs << "  cols: " << data.cols << endl;
  fs << "  dt: " << (CV_32F == data.type() ? "f" : CV_64F == data.type() ? "f" : "unknown") << endl;
  fs << "  data: [ ";
  fs.setf(ios::showpoint);
  fs.precision(9);
  for (size_t row = 0; row < data.rows; ++row) {
    for (size_t col = 0; col < data.cols; ++col) {
      if (CV_32F == data.type()) {
        fs << data.at<float>(row, col);
      } else if (CV_64F == data.type()) {
        fs << data.at<double>(row, col);
      }
      if (col < data.cols - 1)
        fs << ", ";
      else if (row < data.rows - 1)
        fs << "," << endl << "          ";
    }
  }
  fs << "]" << endl;
}
void AutoFillParamsFromDS(cv::FileStorage &fSettings, const string& pathCalib, string &tmp_configfile_path,
                          bool mute) {
  typedef struct SetType {
    int type = 0;  // 0 is vec, 1 is double, 2 is cv::Mat, 3 is int, 4 is string
    struct DataType {
      Eigen::VectorXd vec;
      cv::Mat mat;
      double f;
      int i;
      string s;
    } data;
    SetType() {}
    SetType(const Eigen::VectorXd &vec_in) { data.vec = vec_in; }
    SetType(const double &f_in) : type(1) { data.f = f_in; }
    SetType(const cv::Mat &mat_in) : type(2) { data.mat = mat_in; }
    SetType(const int &i_in) : type(3) { data.i = i_in; }
    SetType(const string &s_in) : type(4) { data.s = s_in; }
  };
  map<string, SetType> all_settings;

  // copy fSettings to all_settings
  cv::FileNode fn_tmp;
  int i = 0;
  while (1) {
    fn_tmp = fSettings.root(i++);
    if (fn_tmp.empty()) break;
    const auto &keys = fn_tmp.keys();
    string last_key0;
    for (auto key : keys) {
      if (!mute) cout << key << endl;

      if (fSettings[key].isReal()) {
        all_settings.emplace(key, fSettings[key].real());
      } else if (fSettings[key].isInt()) {
        all_settings.emplace(key, (int)fSettings[key]);
      } else if (fSettings[key].isString()) {
        all_settings.emplace(key, fSettings[key].string());
      } else if (fSettings[key].isSeq()) {
        Eigen::VectorXd vec;
        auto iter_key = fSettings[key].begin();
        while (fSettings[key].end() != iter_key) {
          auto ind_last = vec.size();
          vec.conservativeResize(ind_last + 1);
          vec[ind_last] = *iter_key;
          ++iter_key;
        }
        all_settings.emplace(key, vec);
      }
    }
  }
  // add settings to all_settings
  cout << pathCalib << endl;
  ifstream input(pathCalib);
  string str_tmp;
  int id_cam = -1;
  while (!input.eof() && !input.fail()) {
    getline(input, str_tmp);

    string keystr[kNumKeyStrType] = {"ombc=\"", " \"", " "};
    size_t last_pos = 0;
    vector<double> ret_vals;
    Eigen::VectorXd T(4 * 4);
    T.setZero();
    cv::Mat Tcv = cv::Mat::zeros(3, 4, CV_64F);
    if (!mute) cout << str_tmp << endl;
    if (!GetFloatArray(str_tmp, keystr, last_pos, ret_vals)) {
      CV_Assert(3 == ret_vals.size());
      Eigen::Vector3d r(ret_vals.data());
      Eigen::Matrix3d R = Sophus::SO3exd::exp(r).matrix();
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) {
          T(row * 4 + col) = R(row, col);
        }
      T(3 * 4 + 3) = 1;
      ret_vals.clear();

      keystr[kStrStart] = "tbc=\"";
      GetFloatArray(str_tmp, keystr, last_pos, ret_vals);
      CV_Assert(3 == ret_vals.size());
      for (int row = 0; row < 3; ++row) {
        T(row * 4 + 3) = ret_vals[row];
      }
      if (!mute) cout << "Camera.Tbc:" << T.transpose() << endl;
      all_settings.emplace("Camera.Tbc", SetType(T));
      ret_vals.clear();
    }

    keystr[kStrStart] = " id=\"";
    keystr[kStrEnd] = "\"";
    if (!GetFloatArray(str_tmp, keystr, last_pos, ret_vals)) {
      CV_Assert(1 == ret_vals.size());
      id_cam = ret_vals[0];
    }
    if (0 <= id_cam) {
      string cam_name = "Camera" + (id_cam ? to_string(id_cam + 1) : "");
      keystr[kStrStart] = "Calibration size=\"";
      keystr[kStrEnd] = " \"";
      if (!GetFloatArray(str_tmp, keystr, last_pos, ret_vals)) {
        CV_Assert(2 == ret_vals.size());
        if (!mute) cout << "size = " << ret_vals[0] << "x" << ret_vals[1] << endl;
        all_settings.emplace(cam_name + ".width", (int)(ret_vals[0]));
        all_settings.emplace(cam_name + ".height", (int)(ret_vals[1]));
        all_settings.emplace(cam_name + ".lappingBegin", 0);
        all_settings.emplace(cam_name + ".lappingEnd", (int)(ret_vals[0] - 1));
        ret_vals.clear();
      }

      keystr[kStrStart] = "principal_point=\"";
      keystr[kStrEnd] = "\"";
      if (!GetFloatArray(str_tmp, keystr, last_pos, ret_vals)) {
        CV_Assert(2 == ret_vals.size());
        if (!mute) cout << cam_name << ".cx,cy:" << ret_vals[0] << "," << ret_vals[1] << endl;
        all_settings.emplace(cam_name + ".cx", ret_vals[0]);
        all_settings.emplace(cam_name + ".cy", ret_vals[1]);
        ret_vals.clear();

        keystr[kStrStart] = "focal_length=\"";
        if (!GetFloatArray(str_tmp, keystr, last_pos, ret_vals)) {
          CV_Assert(2 == ret_vals.size());
          if (!mute) cout << cam_name << ".fx,fy:" << ret_vals[0] << "," << ret_vals[1] << endl;
          auto iter_as = all_settings.find(cam_name + ".fx");
          if (all_settings.end() == iter_as)
            all_settings.emplace(cam_name + ".fx", ret_vals[0]);
          else
            iter_as->second = ret_vals[0];
          all_settings.emplace(cam_name + ".fy", ret_vals[1]);
          ret_vals.clear();

          keystr[kStrStart] = "model=\"";
          size_t model_pos = str_tmp.find(keystr[kStrStart], last_pos);
          if (string::npos != model_pos) {
            model_pos += keystr[kStrStart].length();
            last_pos = str_tmp.find(keystr[kStrEnd], model_pos);
            string model_str = str_tmp.substr(model_pos, last_pos - model_pos);
            last_pos += keystr[kStrEnd].length();
            if (!mute) cout << model_str << endl;
            if ("FISHEYE_4_PARAMETERS" == model_str) all_settings.emplace("Camera.type", "KannalaBrandt8");
          }
        }
      }

      keystr[kStrStart] = "radial_distortion=\"";
      if (!GetFloatArray(str_tmp, keystr, last_pos, ret_vals)) {
        CV_Assert(6 == ret_vals.size());
        if (!mute)
          cout << cam_name << ".k1,k2,k3,k4:" << ret_vals[0] << "," << ret_vals[1] << "," << ret_vals[2] << ","
               << ret_vals[3] << endl;
        for (int j = 0; j < 4; ++j) all_settings.emplace(cam_name + ".k" + to_string(j + 1), ret_vals[j]);
        ret_vals.clear();
      }

      if (0 < id_cam) {
        keystr[kStrStart] = "Rig translation=\"";
        if (!GetFloatArray(str_tmp, keystr, last_pos, ret_vals)) {
          CV_Assert(3 == ret_vals.size());
          Sophus::SE3d Ttmp;
          for (int row = 0; row < 3; ++row) {
            Ttmp.translation()[row] = ret_vals[row];
          }
          ret_vals.clear();

          keystr[kStrStart] = "rowMajorRotationMat=\"";
          GetFloatArray(str_tmp, keystr, last_pos, ret_vals);
          CV_Assert(9 == ret_vals.size());
          Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R(ret_vals.data());
          Ttmp.so3() = Sophus::SO3exd(R);
          Ttmp = Ttmp.inverse();
          for (int row = 0; row < 3; ++row) {
            Tcv.at<double>(row, 3)  = Ttmp.translation()[row];
          }
          R = Ttmp.so3().matrix();
          for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col) {
              Tcv.at<double>(row, col) = R(row, col);
            }
          if (!mute) cout << cam_name + ".Trc:" << T.transpose() << endl;
          all_settings.emplace(cam_name + ".Trc", SetType(Tcv));
          ret_vals.clear();
        }
      }
    }
  }

  // output all_settings
  const string input_config = tmp_configfile_path;
  size_t pos_dir = tmp_configfile_path.rfind("/") + 1;
  tmp_configfile_path = tmp_configfile_path.substr(0, pos_dir) + "VR_tmp.yaml";
  exfstream fs_tmp(tmp_configfile_path, ios::out);
  if (!fs_tmp.is_open()) {
    cerr << "ERROR: Wrong path to tmp_config" << endl;
    return;
  }
  map<string, int> dic_keys;
  string last_key0;
  for (auto iter_mp = all_settings.begin(); all_settings.end() != iter_mp; ++iter_mp) {
    const auto &key = iter_mp->first;
    auto key0 = key.substr(0, key.find_first_of('.'));
    auto dic_iter = dic_keys.find(key0);
    if (dic_iter == dic_keys.end()) {
      dic_keys.emplace(key0, 1);
      if (!mute) cout << endl;
      fs_tmp << endl;
    } else {
      ++dic_iter->second;
      if (last_key0 != key0) {
        if (!mute) cout << endl;
        fs_tmp << endl;
      }
    }
    last_key0 = key0;
    if (!mute) cout << key << endl;

    auto &type_tmp = iter_mp->second.type;
    auto &data_tmp = iter_mp->second.data;
    switch (type_tmp) {
      case 0:
        fs_tmp.write_yaml(key, data_tmp.vec);
        break;
      case 1:
        fs_tmp.write_yaml(key, data_tmp.f);
        break;
      case 2:
        fs_tmp.write_yaml(key, data_tmp.mat);
        break;
      case 3:
        fs_tmp.write_yaml(key, data_tmp.i);
        break;
      case 4:
        fs_tmp.write_yaml(key, data_tmp.s);
        break;
    }
  }
  // CV_Assert(0);
}
