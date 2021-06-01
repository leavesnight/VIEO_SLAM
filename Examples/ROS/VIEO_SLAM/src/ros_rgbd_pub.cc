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
#include <time.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

//! parameters
bool read_from_topic = false, read_from_camera = false;
std::string image_topic = "/camera/image_raw";
int all_pts_pub_gap = 0;
bool show_viewer = true;

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

bool pub_all_pts = false;
int pub_count = 0;

inline bool isInteger(const std::string & s);
void publish(VIEO_SLAM::System &SLAM, ros::Publisher &pub_pts_and_pose,
	ros::Publisher &pub_all_kf_and_pts, int frame_id);

class ImageGrabber{
public:
	ImageGrabber(VIEO_SLAM::System &_SLAM, ros::Publisher &_pub_pts_and_pose,
		ros::Publisher &_pub_all_kf_and_pts) :
		SLAM(_SLAM), pub_pts_and_pose(_pub_pts_and_pose),
		pub_all_kf_and_pts(_pub_all_kf_and_pts), frame_id(0){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	VIEO_SLAM::System &SLAM;
	ros::Publisher &pub_pts_and_pose;
	ros::Publisher &pub_all_kf_and_pts;
	int frame_id;
};

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

int main(int argc, char **argv){
	ros::init(argc, argv, "Monopub");
	ros::start();
	
	char bMode=0;//0 for RGBD(pure/VIO/VEO/VIEO), 1 for MonocularVIO, 2 for Monocular, 3 for Map Reuse RGBD
	thread* pOdomThread=NULL,*pEncThread=NULL;
	ifstream finOdomdata,finEncdata;
	int totalNum=2;
	cout<<fixed<<setprecision(6)<<endl;
	string strMapname="";
      
	switch (argc){
	  case 5:
	    break;
	  case 12:
	    show_viewer=atoi(argv[11]);
	  case 11:
	    all_pts_pub_gap=atoi(argv[10]);
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
	    cerr << redSTR"Usage: rosrun VIEO_SLAM RGBDpub path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_odometryData"
		" (number of odometryData) (Mode) (path_to_EncData)"" (path_to_Map)"" (all_pts_pub_gap show_viewer)"<<endl;
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
	VIEO_SLAM::System SLAM(argv[1],argv[2],sensor,show_viewer);
	bool bLoaded=false;
	if (strMapname!="") bLoaded=SLAM.LoadMap(strMapname,false);//for Map Reuse
	g_pSLAM=&SLAM;
	cin.get();
	
	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;
	
	ros::NodeHandle nodeHandler;
	//ros::Publisher pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
	ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("/kinect2_odom_publisher/pts_and_pose", 1000);
	ros::Publisher pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("/kinect2_odom_publisher/all_kf_and_pts", 1000);
// 	ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
// 	ros::Publisher pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	if (read_from_topic) {
		assert("Wrong"&&0);
		ImageGrabber igb(SLAM, pub_pts_and_pose, pub_all_kf_and_pts);
		ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage, &igb);
		ros::spin();
	}
	else{
// 		ros::Rate loop_rate(5);
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

	    #ifdef COMPILEDWITHC11
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

	    #ifdef COMPILEDWITHC11
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
		    
		    publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, ni);
		    ros::spinOnce();
// 		    loop_rate.sleep();
		    if (!ros::ok()) break;
		}
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
	//geometry_msgs::PoseArray pt_array;
	//pt_array.header.seq = 0;
	//pub_pts_and_pose.publish(pt_array);
	ros::shutdown();
	
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
	
	//wait for pOdomThread finished
	if (pOdomThread!=NULL)
	  pOdomThread->join();
	if (pEncThread!=NULL)
	  pEncThread->join();
    
	return 0;
}

void publish(VIEO_SLAM::System &SLAM, ros::Publisher &pub_pts_and_pose,
	ros::Publisher &pub_all_kf_and_pts, int frame_id) {
	if (all_pts_pub_gap > 0 && pub_count >= all_pts_pub_gap) {
		pub_all_pts = true;
		pub_count = 0;
	}
	if (pub_all_pts || SLAM.GetLoopDetected()) {
		pub_all_pts = false;SLAM.SetLoopDetected(false);
		geometry_msgs::PoseArray kf_pt_array;
		vector<VIEO_SLAM::KeyFrame*> key_frames = SLAM.GetAllKeyFrames();
		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), VIEO_SLAM::KeyFrame::lId);
		unsigned int n_kf = 0;
		unsigned int n_pts_id = 0;
		for (auto key_frame : key_frames) {
			// pKF->SetPose(pKF->GetPose()*Two);

			if (!key_frame || key_frame->isBad()) {
				continue;
			}

			cv::Mat R = key_frame->GetRotation().t();
			vector<float> q = VIEO_SLAM::Converter::toQuaternion(R);
			cv::Mat twc = key_frame->GetCameraCenter();
			geometry_msgs::Pose kf_pose;

			kf_pose.position.x = twc.at<float>(0);
			kf_pose.position.y = twc.at<float>(1);
			kf_pose.position.z = twc.at<float>(2);
			kf_pose.orientation.x = q[0];
			kf_pose.orientation.y = q[1];
			kf_pose.orientation.z = q[2];
			kf_pose.orientation.w = q[3];
			kf_pt_array.poses.push_back(kf_pose);

			n_pts_id = kf_pt_array.poses.size();
			//! placeholder for number of points
			kf_pt_array.poses.push_back(geometry_msgs::Pose());
			std::set<VIEO_SLAM::MapPoint*> map_points = key_frame->GetMapPoints();
			unsigned int n_pts = 0;
			for (auto map_pt : map_points) {
				if (!map_pt || map_pt->isBad()) {
					//printf("Point %d is bad\n", pt_id);
					continue;
				}
				cv::Mat pt_pose = map_pt->GetWorldPos();
				if (pt_pose.empty()) {
					//printf("World position for point %d is empty\n", pt_id);
					continue;
				}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = pt_pose.at<float>(0);
				curr_pt.position.y = pt_pose.at<float>(1);
				curr_pt.position.z = pt_pose.at<float>(2);
				kf_pt_array.poses.push_back(curr_pt);
				++n_pts;
			}
			kf_pt_array.poses[n_pts_id].position.x = (double)n_pts;
			kf_pt_array.poses[n_pts_id].position.y = (double)n_pts;
			kf_pt_array.poses[n_pts_id].position.z = (double)n_pts;
			++n_kf;
		}
		kf_pt_array.poses[0].position.x = (double)n_kf;
		kf_pt_array.poses[0].position.y = (double)n_kf;
		kf_pt_array.poses[0].position.z = (double)n_kf;
		kf_pt_array.header.frame_id = "1";
		kf_pt_array.header.seq = frame_id + 1;
		printf("Publishing data for %u keyfranmes\n", n_kf);
		pub_all_kf_and_pts.publish(kf_pt_array);
	}
	else if (SLAM.GetKeyFrameCreated()) {
		++pub_count;
		SLAM.SetKeyFrameCreated(false);

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
		//while (pKF->isBad())
		//{
		//	Trw = Trw*pKF->mTcp;
		//	pKF = pKF->GetParent();
		//}

// 		vector<VIEO_SLAM::KeyFrame*> vpKFs = SLAM.GetAllKeyFrames();
// 		sort(vpKFs.begin(), vpKFs.end(), VIEO_SLAM::KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin. but it this is rectified, we should rectify Maps' position as well!
// 		cv::Mat Two = vpKFs[0]->GetPoseInverse();

// 		Trw = Trw*pKF->GetPose()*Two;
// 		cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
// 		cv::Mat Tcw = lit*Trw;
		cv::Mat Tcw=SLAM.GetKeyFramePose();
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = VIEO_SLAM::Converter::toQuaternion(Rwc);
		//geometry_msgs::Pose camera_pose;
		//std::vector<VIEO_SLAM::MapPoint*> map_points = SLAM.getMap()->GetAllMapPoints();
		std::vector<VIEO_SLAM::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
		int n_map_pts = map_points.size();

		//printf("n_map_pts: %d\n", n_map_pts);

		//pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		geometry_msgs::PoseArray pt_array;
		//pt_array.poses.resize(n_map_pts + 1);

		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);

		//printf("Done getting camera pose\n");

		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
				//printf("Point %d is bad\n", pt_id);
				continue;
			}
			cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

			if (wp.empty()) {
				//printf("World position for point %d is empty\n", pt_id);
				continue;
			}
			geometry_msgs::Pose curr_pt;
			//printf("wp size: %d, %d\n", wp.rows, wp.cols);
			//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
			curr_pt.position.x = wp.at<float>(0);
			curr_pt.position.y = wp.at<float>(1);
			curr_pt.position.z = wp.at<float>(2);
			pt_array.poses.push_back(curr_pt);
			//printf("Done getting map point %d\n", pt_id);
		}
		//sensor_msgs::PointCloud2 ros_cloud;
		//pcl::toROSMsg(*pcl_cloud, ros_cloud);
		//ros_cloud.header.frame_id = "1";
		//ros_cloud.header.seq = ni;

		//printf("valid map pts: %lu\n", pt_array.poses.size()-1);

		//printf("ros_cloud size: %d x %d\n", ros_cloud.height, ros_cloud.width);
		//pub_cloud.publish(ros_cloud);
		pt_array.header.frame_id = "1";
		pt_array.header.seq = frame_id + 1;
		pub_pts_and_pose.publish(pt_array);
		//pub_kf.publish(camera_pose);
	}
}

inline bool isInteger(const std::string & s){
	if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

	char * p;
	strtol(s.c_str(), &p, 10);

	return (*p == 0);
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

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg){
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
	publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, frame_id);
	++frame_id;
}




