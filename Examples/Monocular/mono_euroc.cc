/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/leavesnight/VIEO_SLAM>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

//zzh
VIEO_SLAM::System* g_pSLAM;
double g_simulateTimestamp=-1,gDelayCache;
bool g_brgbdFinished=false;
mutex g_mutex;

//a new thread simulating the odom serial threads
void odomRun(ifstream &finOdomdata,int totalNum){//must use &
  //read until reading over
  int nTotalNum=6;//wx~z,ax~z
  if (totalNum!=0) nTotalNum=totalNum;
  double* odomdata=new double[nTotalNum];
  double timestamp,tmstpLast=-1;
  
  while (!g_pSLAM){//if it's NULL
    usleep(15000);//wait 15ms
  }
  while (!finOdomdata.eof()){
    string strTmp;
    getline(finOdomdata,strTmp);
    int posLast=strTmp.find(',');
    timestamp=atof(strTmp.substr(0,posLast).c_str())/1e9;
    ++posLast;
    while (1){//until the image reading time is reached
      {
      unique_lock<mutex> lock(g_mutex);
      if (timestamp<=g_simulateTimestamp+gDelayCache||g_brgbdFinished)
	break;
      }
      usleep(1000);//allow 1ms delay
    }
    for (int i=0;i<nTotalNum;++i){//we should change wxyz,axyz to the order of axyz,wxyz in odomdata
      int pos=strTmp.find(',',posLast);
      string::size_type posNum;if (pos!=string::npos) posNum=pos-posLast;else posNum=string::npos;
      double dtmp=atof(strTmp.substr(posLast,posNum).c_str());
      if (i<nTotalNum/2)
	odomdata[nTotalNum/2+i]=dtmp;
      else
	odomdata[i-nTotalNum/2]=dtmp;
      posLast=pos+1;
    }
    //for (int i=0;i<6;++i) cout<<odomdata[i]<<" ";cout<<endl;
    if (timestamp>tmstpLast)//avoid == condition
      g_pSLAM->TrackOdom(timestamp,odomdata,(char)VIEO_SLAM::System::IMU);//for EuRoC dataset
    //cout<<green<<timestamp<<whiteSTR<<endl;
    tmstpLast=timestamp;
  }
  delete []odomdata;
  finOdomdata.close();
  cout<<greenSTR"Simulation of Odom Data Reading is over."<<whiteSTR<<endl;
}
//zzh over

int main(int argc, char **argv)
{
    thread* pOdomThread=NULL;
    ifstream finOdomdata;
    int totalNum=0;
    cout<<fixed<<setprecision(6)<<endl;
  
    switch (argc){
      case 5:
	break;
      case 7:
	totalNum=atoi(argv[6]);
      case 6:
	{
	finOdomdata.open(argv[5]);
	if (!finOdomdata.is_open()){
	  cerr<< redSTR"Please check the last path_to_odometryData"<<endl;
	  return -1;
	}
        string strTmp;
	getline(finOdomdata,strTmp);//EuRoC's data.csv only has one unused line
        pOdomThread=new thread(&odomRun,ref(finOdomdata),totalNum);//must use ref()
        cout<<"OdomThread created!"<<endl;
	}
	break;
      default:
	cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
	cerr << redSTR"Or: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file path_to_odometryData (number of odometryData)"<<endl;
        return 1;
    }

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);//already checked in System() creator
    cv::FileNode fnDelay=fSettings["Camera.delayForPolling"];
    if (fnDelay.empty()){
      gDelayCache=0;
    }else{
      gDelayCache=(double)fnDelay;
    }
    
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    VIEO_SLAM::System SLAM(argv[1],argv[2],VIEO_SLAM::System::MONOCULAR,true);
    g_pSLAM=&SLAM;//zzh

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
	{//zzh
	unique_lock<mutex> lock(g_mutex);
	g_simulateTimestamp=tframe;//update g_simulateTimestamp
	}

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    
    //zzh
    {
    unique_lock<mutex> lock(g_mutex);
    g_brgbdFinished=true;
    }
    if (SLAM.MapChanged()){
      cout<<"Map is changing!Please enter s to stop!"<<endl;
//       while (cin.get()!='s') {sleep(1);}
      sleep(5);
    }
    //zzh over

    // Stop all threads
    SLAM.Shutdown();
    
    //zzh: FinalGBA, this is just the FullBA column in the paper! see "full BA at the end of the execution" in V-B of the VIORBSLAM paper!
    //load if Full BA just after IMU Initialized
    cv::FileNode fnFBA=fSettings["GBA.finalIterations"];
    SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameTrajectoryIMU_NO_FULLBA.txt");
//     SLAM.SaveMap("KeyFrameTrajectoryMap.bin",false);
    if (!fnFBA.empty()){
      if((int)fnFBA){
        SLAM.FinalGBA(fnFBA);
        cout<<azureSTR"Execute FullBA at the end!"<<whiteSTR<<endl;
      }
    }else{
      cout<<redSTR"No FullBA at the end!"<<whiteSTR<<endl;
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameTrajectoryIMU.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    //SLAM.SaveMap("Map.pcd");//zzh
    //wait for pOdomThread finished
    if (pOdomThread!=NULL)//zzh
      pOdomThread->join();

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
