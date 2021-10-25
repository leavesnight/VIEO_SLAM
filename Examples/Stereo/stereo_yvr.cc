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

using namespace std;

static void GetFileNames(const string& path, vector<string>& filenames, const string& suffix=".pgm", const string& prefix="");

void LoadImages(const string &strImagePath, vector<string> &vstrImages, vector<double> &vTimeStamps);

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
  cout << greenSTR "Simulation of Odom Data Reading is over." << whiteSTR << endl;
}

int main(int argc, char **argv) {
  thread *pOdomThread = NULL;
  int totalNum = 0;
  cout << fixed << setprecision(6) << endl;

  const int num_seq = argc < 3 ? 1 : argc - 3;
  int seq = 0;
  string pathSeq(argv[(seq) + 3]);
  vector<vector<double>> vTimestampsImu;
  vector<vector<cv::Point3f>> vAcc, vGyro;
  vTimestampsImu.resize(num_seq);
  vAcc.resize(num_seq);
  vGyro.resize(num_seq);

  string mode = "VIO";
  switch (argc) {
    case 5:
      mode = argv[4];
    case 4: {
      vector<string> pathImu = {pathSeq + "/Sensors/gyroscope.xml", pathSeq + "/Sensors/accelerometer.xml"};
      LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
      cout << "IMU size="<<vTimestampsImu[seq].size()<<endl;
      if (!vTimestampsImu[seq].size()) {
        cerr << redSTR "Please check the last path_to_odometryFolder" << endl;
        return -1;
      }
      string strTmp;
      pOdomThread = new thread(&odomRun, ref(vTimestampsImu[seq]), ref(vAcc[seq]), ref(vGyro[seq]));
      cout << "OdomThread created!" << endl;
    } break;
    default:
      cerr << endl << "Usage: ./stereo_yvr path_to_vocabulary path_to_settings path_to_folder (VIO)" << endl;
      return 1;
  }
  if (mode != "VIO") {
    cerr << "unsupported mode now" << endl;
  }

  cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);  // already checked in System() creator
  cv::FileNode fnDelay = fSettings["Camera.delayForPolling"];
  if (fnDelay.empty()) {
    gDelayCache = 0;
  } else {
    gDelayCache = (double)fnDelay;
  }

  // Retrieve paths to images
  vector<vector<string>> vstrImages(2);  // TODO: 4
  vector<vector<double>> vTimestampsCam(2);
  string pathCam0 = pathSeq + "/Camera8";
  LoadImages(pathCam0, vstrImages[seq], vTimestampsCam[seq]);
  cout << "Img size="<<vTimestampsCam[seq].size()<<endl;

  if (vstrImages[0].empty()) {
    cerr << "ERROR: No images in provided path." << endl;
    return 1;
  }

  // Read rectification parameters
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    return -1;
  }

  const int nImages = vstrImages[0].size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  VIEO_SLAM::System SLAM(argv[1], argv[2], VIEO_SLAM::System::STEREO, true);
  g_pSLAM = &SLAM;

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat imLeft, imRight;
  for (int ni = 0; ni < nImages; ni++) {
    // Read left and right images from file
    imLeft = cv::imread(vstrImages[seq][ni],cv::IMREAD_GRAYSCALE);
    imRight = imLeft.colRange(imLeft.cols / 2, imLeft.cols);
    imLeft = imLeft.colRange(0, imLeft.cols / 2);

    if (imLeft.empty()) {
      cerr << endl << "Failed to load image at: " << string(vstrImages[seq][ni]) << endl;
      return 1;
    }

    if (imRight.empty()) {
      cerr << endl << "Failed to load image at: " << string(vstrImages[seq][ni]) << endl;
      return 1;
    }

    double tframe = vTimestampsCam[seq][ni];
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
    SLAM.TrackStereo(imLeft, imRight, tframe, false);

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
      T = vTimestampsCam[seq][ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vTimestampsCam[seq][ni - 1];

    if (ttrack < T) usleep((T - ttrack) * 1e6);
  }

  // zzh
  {
    unique_lock<mutex> lock(g_mutex);
    g_brgbdFinished = true;
  }
  if (SLAM.MapChanged()) {
    cout << "Map is changing!Please enter s to stop!" << endl;
    //       while (cin.get()!='s') {sleep(1);}
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
      cout << azureSTR "Execute FullBA at the end!" << whiteSTR << endl;
    }
  } else {
    cout << redSTR "No FullBA at the end!" << whiteSTR << endl;
  }

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameTrajectoryIMU.txt");
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

  return 0;
}

static void GetFileNames(const string& path, vector<string>& filenames, const string& suffix, const string& prefix) {
  DIR *pDir;
  struct dirent* ptr;
  if (!(pDir = opendir(path.c_str()))) {
    cerr<<path<<" opendir failed"<<endl;
    return;
  }

  while ((ptr = readdir(pDir)) != nullptr) {
    if (strcmp(ptr->d_name, ".") && strcmp(ptr->d_name, "..")) {
      string d_name = string(ptr->d_name);
      size_t pos_suffix = d_name.rfind(suffix), pos_prefix = 0;
      if (!prefix.empty())
        pos_prefix = d_name.find(prefix);
      if (string::npos != pos_suffix && pos_suffix + suffix.length() == d_name.length()) {
        if (!pos_prefix)
          filenames.push_back(path + "/" + ptr->d_name);
      }
    }
  }
  closedir(pDir);
}

void LoadImages(const string &strImagePath, vector<string> &vstrImages, vector<double> &vTimeStamps)
{
  ifstream fTimes;
  GetFileNames(strImagePath, vstrImages);
  sort(vstrImages.begin(),vstrImages.end());
  for (int i = 0; i < vstrImages.size(); ++i) {
    string dir_path = dirname(strdup(vstrImages[i].c_str()));
    int offset = dir_path.length();
    if (dir_path[offset - 1] != '/' and dir_path[offset - 1] != '\\')
      ++offset;
    double ftmp = strtod(vstrImages[i].substr(offset).c_str(), 0) * 1e-9;
    vTimeStamps.push_back(ftmp);
  }
}

typedef enum KeyStrType { kStrStart, kStrEnd, kStrDivide, kNumKeyStrType };

static int GetFloatArray(const string &str_tmp, const string *keystr, size_t &last_pos, vector<double> &ret_vals) {
  size_t pos_keystr = str_tmp.find(keystr[kStrStart], last_pos);
  if (keystr[kStrStart] == "") pos_keystr = 0;
  if (string::npos != pos_keystr) {
    last_pos = pos_keystr + keystr[kStrStart].length();
    if (keystr[kStrEnd] == "")
      pos_keystr = str_tmp.length();
    else
      pos_keystr = str_tmp.find(keystr[kStrEnd], last_pos);
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
  if (strImuPath.size() > 1) {
    fAcc.open(strImuPath[1].c_str());
  }
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);
  vector<double> ret_vals;

  while (!fImu.eof()) {
    string s;
    getline(fImu, s);

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
  }

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