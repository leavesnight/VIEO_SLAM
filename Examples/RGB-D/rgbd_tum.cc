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
// rectified by zzh in 2017.

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

//zzh
VIEO_SLAM::System* g_pSLAM;
double g_simulateTimestamp=-1,gDelayCache;
bool g_brgbdFinished=false;
mutex g_mutex;

//a new thread simulating the odom serial threads
void odomIMURun(ifstream &finOdomdata,int totalNum){//must use &
    //read until reading over
    int nTotalNum=2+4+3*3;
    if (totalNum!=0) nTotalNum=totalNum;
    double* odomdata=new double[nTotalNum];
    double timestamp,tmstpLast=-1;

    while (!g_pSLAM){//if it's NULL
        usleep(1e5);//wait 0.1s
    }
    while (!finOdomdata.eof()){
        finOdomdata>>timestamp;
        if (finOdomdata.eof())
            break;
        while (1){//until the image reading time is reached
            {
              unique_lock<mutex> lock(g_mutex);
              if (timestamp<=g_simulateTimestamp+gDelayCache||g_brgbdFinished)
                  break;
            }
            usleep(1.5e4);//allow 15ms delay
        }
        for (int i=0;i<nTotalNum;++i){
            finOdomdata>>odomdata[i];
        }
        if (timestamp>tmstpLast){//avoid == condition
        //       if (nTotalNum>=9&&(odomdata[nTotalNum-5]>-0.5||odomdata[nTotalNum-5]<-1.5)){//ay:2+4+3+2 -1
        // 	cout<<redSTR"Wrong imu data! t: "<<timestamp<<whiteSTR<<endl;
        // 	continue;
        //       }
#ifndef TRACK_WITH_IMU
            g_pSLAM->TrackOdom(timestamp,odomdata,(char)VIEO_SLAM::System::ENCODER);
#else
            g_pSLAM->TrackOdom(timestamp,odomdata+(nTotalNum-6),(char)VIEO_SLAM::System::IMU);//jump vl,vr,quat[4],magnetic data[3] then it's axyz,wxyz for default 15, please ensure the last 6 data is axyz,wxyz
            //g_pSLAM->TrackOdom(timestamp,odomdata,(char)VIEO_SLAM::System::ENCODER);//nTotalNum=2
#endif
        }
        //cout<<greenSTR<<timestamp<<whiteSTR<<endl;
        tmstpLast=timestamp;
    }
    delete []odomdata;
    finOdomdata.close();
    cout<<greenSTR"Simulation of Odom Data Reading is over."<<whiteSTR<<endl;
}
void odomEncRun(ifstream &finOdomdata){//must use &
    //read until reading over
    int nTotalNum=2;
    double* odomdata=new double[nTotalNum];
    double timestamp,tmstpLast=-1;
    string strTmp;

    while (!g_pSLAM){//if it's NULL
        usleep(1e5);//wait 0.1s
    }
    while (!finOdomdata.eof()){
        finOdomdata>>timestamp;
        if (finOdomdata.eof())
            break;
        while (1){//until the image reading time is reached
            {
                unique_lock<mutex> lock(g_mutex);
                if (timestamp<=g_simulateTimestamp+gDelayCache||g_brgbdFinished)
                    break;
            }
            usleep(1.5e4);//allow 15ms delay
        }
        for (int i=0;i<nTotalNum;++i){
            finOdomdata>>odomdata[i];
        }
        getline(finOdomdata,strTmp);
        if (timestamp>tmstpLast){//avoid == condition
#ifndef TRACK_WITH_IMU
            g_pSLAM->TrackOdom(timestamp,odomdata,(char)VIEO_SLAM::System::ENCODER);
#else
            g_pSLAM->TrackOdom(timestamp,odomdata,(char)VIEO_SLAM::System::ENCODER);//nTotalNum=2
#endif
        }
        //cout<<greenSTR<<timestamp<<whiteSTR<<endl;
        tmstpLast=timestamp;
    }
    delete []odomdata;
    finOdomdata.close();
    cout<<greenSTR"Simulation of Odom Data Reading is over."<<whiteSTR<<endl;
}
//zzh over

int main(int argc, char **argv)
{
    char bMode=0;//0 for RGBD(pure/VIO/VEO/VIEO), 1 for MonocularVIO, 2 for Monocular, 3 for Map Reuse RGBD
    thread* pOdomThread=NULL,*pEncThread=NULL;
    ifstream finOdomdata,finEncdata;
    int totalNum=2;
    cout<<fixed<<setprecision(6)<<endl;
    string strMapname="";
  
    switch (argc){
    case 5:
        break;
    case 10:
        //Map Reuse RGBD
        strMapname=argv[9];
    case 9:
    {
        finEncdata.open(argv[8]);
        string strTmp;
        getline(finEncdata,strTmp);getline(finEncdata,strTmp);getline(finEncdata,strTmp);
    }
    case 8:
        bMode=atoi(argv[7]);if (bMode==2) break;
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
        getline(finOdomdata,strTmp);getline(finOdomdata,strTmp);getline(finOdomdata,strTmp);//odom.txt should have 3 unused lines
        if (totalNum==2)
            pOdomThread=new thread(&odomEncRun,ref(finOdomdata));//must use ref()
        else{
            pOdomThread=new thread(&odomIMURun,ref(finOdomdata),totalNum);//must use ref()
            if (finEncdata.is_open()) pEncThread=new thread(&odomEncRun,ref(finEncdata));
        }
    }
        break;
    default:
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        cerr << redSTR"Or: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_odometryData"
        " (number of odometryData) (Mode) (path_to_EncData)"" (path_to_Map)"<<endl;
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
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    VIEO_SLAM::System::eSensor sensor=VIEO_SLAM::System::RGBD;
    if (bMode!=0) sensor=VIEO_SLAM::System::MONOCULAR;
    VIEO_SLAM::System SLAM(argv[1],argv[2],sensor,true);
    bool bLoaded=false;
    if (strMapname!="") bLoaded=SLAM.LoadMap(strMapname,false);//for Map Reuse
    g_pSLAM=&SLAM;
//    cin.get();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        {//zzh
            unique_lock<mutex> lock(g_mutex);
            g_simulateTimestamp=tframe;//update g_simulateTimestamp
        }

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                    << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#if (defined(COMPILEDWITHC11) || defined(COMPILEDWITHC17))
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        if (bMode==0)
            SLAM.TrackRGBD(imRGB,imD,tframe);
        else
            SLAM.TrackMonocular(imRGB,tframe);

        //double data[9]={0.5,0.4};
        //SLAM.SaveFrame("./test_save/",imRGB,imD,tframe);

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
        while (cin.get()!='s') {sleep(1);}
    }
    //zzh over

    // Stop all threads
    SLAM.Shutdown();
    
    //zzh: FinalGBA, this is just the FullBA column in the paper! see "full BA at the end of the execution" in V-B of the VIORBSLAM paper!
    //load if Full BA just after IMU Initialize
    cv::FileNode fnFBA=fSettings["GBA.finalIterations"];
//     SLAM.SaveKeyFrameTrajectoryNavState("KeyFrameTrajectoryIMU_NO_FULLBA.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory_NO_FULLBA.txt");
//     SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory2.txt"); 
//     SLAM.SaveMap("MapTmp.bin",false,true,true);
    if (!fnFBA.empty()&&!bLoaded){
      if((int)fnFBA){
        SLAM.FinalGBA(fnFBA);
        cout<<azureSTR"Execute FullBA at the end!"<<whiteSTR<<endl;
      }
    }else{
        cout<<redSTR"No FullBA at the end!"<<whiteSTR<<endl;
    }/*
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory1.txt"); 
    SLAM.SaveTrajectoryTUM("CameraTrajectory1.txt");
    SLAM.LoadMap("MapTmp.bin",false,true);
    if (!fnFBA.empty()){
      if((int)fnFBA){
	SLAM.FinalGBA(fnFBA);
	cout<<azureSTR"Execute FullBA2 at the end!"<<whiteSTR<<endl;
      }
    }else{
      cout<<redSTR"No FullBA2 at the end!"<<whiteSTR<<endl;
    }*/

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
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    if (strMapname=="")
      SLAM.SaveMap("Map.pcd");//for PCL Map
    else
      SLAM.SaveMap(strMapname,false);//for Reused Sparse Map
    
    //wait for pOdomThread finished
    if (pOdomThread!=NULL)
      pOdomThread->join();
    if (pEncThread!=NULL)
      pEncThread->join();

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t,t2;
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
