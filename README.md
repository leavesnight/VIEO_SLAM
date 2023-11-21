# VIEO_SLAM
Now this project is improved by zzh, based on ORBSLAM2 && ORBSLAM3

1. More Annotations and Less Codes(By Template Class Tech.)
2. Real-Time Code With Some Subtle Changes(default is to be faster):
   1. Able to run full ba at last(slower but map should be more accurate)
   2. Loading uses .bin vocabulary
   3. PoseOptimization uses PVR vertex while localBA/GlobalBA use PR-V
   4. Some other time cost related changes
   5. Future work can be:
      1. inverse depth parameterization
      2. optical flow method
   
3. More Camera+Sensor Modes are supported:
   1. undistorted Monocular + (IMU/Encoder) mode: Monocular V(I)(E)O is implemented like the VIORBSLAM paper[Visual-Inertial Monocular SLAM with Map Reuse](https://arxiv.org/pdf/1610.05949.pdf), especially thanks to OnManifold Preintegration paper[On-Manifold Preintegration for Real-Time](https://arxiv.org/pdf/1512.02363.pdf) and [JingWang's initial version](https://github.com/jingpang/LearnVIORB) and our VIEOS2
   2. rectified Stereo(max 2 cams) + (IMU/Encoder) mode: Stereo V(I)(E)O is implemented like our VIEOS2
   3. undistorted RGBD(max 1 cam) + (IMU/Encoder) mode: RGBD V(I)(E)O is implemented like our [VIEOS2](https://github.com/leavesnight/ORB_SLAM2)
   4. distorted Stereo(max 4 cams) + (IMU/Encoder) mode: dStereo V(I)(E)O is implemented like [ORBSLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) and our VIEOS2
   5. distorted RGBD(max 1 cam) + (IMU/Encoder) mode: dRGBD V(I)(E)O is implemented like dStereo VIEO
      
4. We named the total system as VIEO_SLAM, where O should finally means the other odometers instead of ORB descriptor. 
   We hope to provide a general visual tightly-coupled with any odometer SLAM system in the end.

PS:
   1. IMU now only support max 1 IMU
   2. If more cams on distorted Stereo, easily fix the seg bug by yourself
   3. For Monocular V(I)EO: pure encoder edges insertion strategy now is simple like +IMU insertion strategy,
      but scale initialization for mono is still lacked, so this mode is not fully completed;
      And we still consider Initialization of Monocular SLAM is not suitable for a Differential Wheeled Robot based on our experience

## Test

### EuRoC Dataset

*Without Full BA Monocular/(d)Stereo VIO:*

**ATE Min Of Random 3 tests MonocularVIO Res(cm,Leica);feat num 1000:**
```
(With FBA)
V101(1.4cm),V102(1.8),V103(4.4),V201(1.2),V202(1.3),V203(2.6),MH01(6.7),MH02(5.7),MH03(6.3),MH04(5.9),MH05(9.4)
PS: 2017/12/26
```
**ATE Avg/Median of N tests StereoVIO Res(m,Leica|Est\*);
Features Number Per Image 1200(default)->375(with full ba);
XXX_distXXX.yaml(default)--XXX.yaml(undist as ref)**
```
(With FBA)
V101 0.019/?|0.037/? 1->0.018/?|0.036/? 1--0.019/?|0.036/? 1
V102 0.023/?|0.018/? 1->0.021/?|0.012/? 1--0.023/?|0.018/? 1
V103 0.039/?|0.023/? 1->0.038/?|0.017/? 1--0.040/?|0.021/? 1
V201 0.014/?|0.014/? 1->0.019/?|0.017/? 1--0.030/?|0.030/? 1
V202 0.016/?|0.014/? 1->0.012/?|0.009/? 1--0.014/?|0.011/? 1
V203 0.017/?|0.016/? 1->0.020/?|0.015/? 1--0.026/?|0.024/? 1
MH01 0.060/?|0.016/? 1->0.062/?|0.011/? 1--0.069/?|0.024/? 1
MH02 0.047/?|0.015/? 1->0.052/?|0.016/? 1--0.051/?|0.017/? 1
MH03 0.078/?|0.025/? 1->0.081/?|0.026/? 1--0.076/?|0.024/? 1
MH04 0.081/?|0.044/? 1->0.086/?|0.058/? 1--0.093/?|0.054/? 1
MH05 0.100/?|0.039/? 1->0.137/?|0.080/? 1--0.117/?|0.057/? 1
Final MH05 mean time cost per frame of frontend(ms): 41.X -> 20.X -- 30.X
PS:2024/6/27;
Script(./EvaluateEuRoC_Ntimes.sh StereoVIO 10) On i7-12700H(futrue on i9-14900HX);
+ meaning accuracy up(both median diff>=5mm)/ times cost down(>=1ms) compared with before
```

### TUM_VI Dataset

*Without Full BA dStereo VIO:*

**ATE Random tests (1|2|...) StereoVIO Res(m);feat num 1000(default)->375:**

```
corridor1   0.031/? 1->0.032
corridor2   0.024/? 1->0.068
corridor3   0.016/? 1->0.044
corridor4   0.261/? 1->0.214
corridor5   0.040/? 1->0.097
magistrale1 0.107/? 1->0.818
magistrale2 1.134/? 1->2.114
magistrale3 1.437/? 1->1.422
magistrale4 0.194/? 1->1.430
magistrale5 0.643/? 1->1.696
magistrale6 0.198/? 1->3.531
outdoors1   8.752/? 1->21.74
outdoors2   4.319/? 1->8.438
outdoors3   2.373/? 1->32.84
outdoors4   2.805/? 1->14.84
outdoors5   5.017/? 1->18.42
outdoors6   15.13/? 1->23.07
outdoors7   1.338/? 1->5.894
outdoors8   3.533/? 1->7.606
room1       0.010/? 1->0.010
room2       0.010/? 1->0.009
room3       0.009/? 1->0.009
room4       0.006/? 1->0.008
room5       0.010/? 1->0.007
room6       0.008/? 1->0.007
slides1     0.126/? 1->0.304
slides2     0.418/? 1->0.875
slides3     0.413/? 1->0.818
Final slides3(current parallel, future single) mean time cost per frame of frontend(ms): ~46 ->21
PS:2024/6/27;
Script(./EvaluateEuRoC_Ntimes.sh StereoVIO 3 TUM_VI) On i7-12700H;
```

## Usage

1. Build with build.sh. Notice the version requirement of Eigen3 & OpenCV in CMakeLists.txt.
2. Modify the path in Examples/Monocular/EuRoC_VIO.yaml
3. Normally use it like:

   Monocular: `cd ~/zzh/VIEO_SLAM/Examples/Monocular`

   Stereo: `cd ~/zzh/VIEO_SLAM/Examples/Stereo`

   VIO:(LocalWindowSize<=0 for Monocular ORB-SLAM in that paper: ORB-SLAM+IMU Initialization)
   ```
    ./mono_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/V202medium/mav0/cam0/data ./EuRoC_TimeStamps/V202.txt ~/dataset/EuRoC/V202medium/mav0/imu0/data.csv 
    ./stereo_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml  ~/dataset/EuRoC/V103difficult/mav0/cam0/data ~/dataset/EuRoC/V103difficult/mav0/cam1/data ./EuRoC_TimeStamps/V103.txt ~/dataset/EuRoC/V103difficult/mav0/imu0/data.csv
   ```
   ORBSLAM2:(you can also use ./EuRoC.yaml with Tbc is default I)
   ```
    ./mono_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/V202medium/mav0/cam0/data ./EuRoC_TimeStamps/V202.txt 
    ./stereo_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml  ~/dataset/EuRoC/V203difficult/mav0/cam0/data ~/dataset/EuRoC/V203difficult/mav0/cam1/data ./EuRoC_TimeStamps/V203.txt
   ```
   VEORBSLAM2 or VIEORBSLAM2: (you may refer to the kinect2_qhd.yaml, later we may publish our dataset containing encoders & IMU data)
   ```
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/EncSensor.txt
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_qhd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt(path to IMU data file) 9(number of IMU data: (Accelerator,Magnetic,Gyroscope) for our dataset) 0(RGBD SLAM) $OURFILE/EncSensor.txt(path to encoder data file)
   ```
4. Fastly use it on EuRoC/TUM-VI Benchmark like:

   VIEO_SLAM:
   ```
   ./SetEuRoCVIO_Once.sh StereoVIO MH05difficult MH05 0.2 EuRoC EuRoC_VIO
   ./SetEuRoCVIO_Once.sh StereoVIO dataset-room1_512_16 dataset-room1_512 0. TUM_VI TUM_VI_512_VIO
   ```
*PS: Please contact zhuzhanghao9331@yahoo.co.jp for more details.*

## Related Publications:

[IMU, Encoders and Stereo/RGB-D] Zhanghao Zhu, Yutaka Kaizu, Kenichi Furuhashi and Kenji Imou. **Visual-Inertial RGB-D SLAM with Encoders for a Differential Wheeled Robot**. *IEEE Sensors Journal,* vol. 22, pp. 5360-5371, 2022. **(https://ieeexplore.ieee.org/document/9502133)**

Below most part is the primary README of ORBSLAM2
----------------------------------------
# ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**13 Jan 2017**: OpenCV 3 and Eigen 3.3 are now supported.

**22 Dec 2016**: Added AR demo (see section 7).

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular. We also provide a ROS node to process live monocular, stereo or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### Related Publications:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *ArXiv preprint arXiv:1610.06475* **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).
VIEO_SLAM is under review and when the paper is published, we may also apply for GPLv3 release or just use the one from ORB-SLAM2

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

For a closed-source version of VIEO_SLAM for commercial purposes, please contact the authors: zhuzhanghao9331@yahoo.co.jp

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={arXiv preprint arXiv:1610.06475},
      year={2016}
     }

If you use VIEOS2 (Stereo or RGB-D) in an academic work, please cite:

    @article{zhu2021visual,
      title={Visual-Inertial RGB-D SLAM with Encoders for a Differential Wheeled Robot},
      author={Zhu, Zhanghao and Kaizu, Yutaka and Furuhashi, Kenichi and Imou, Kenji},
      journal={IEEE Sensors Journal},
      year={2021},
      publisher={IEEE}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 20.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin+Sophus
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin. **Tested with commit id 86eb4975fc4f.**

We use [Sophus](https://github.com/strasdat/Sophus) for more compact translation and rotation operation. **Tested with commit id 780cd0fce405(old code with Eigen 3.3.7).**

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 4.5/3.2.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0. Tested with Eigen 3.3.7/3.2.10**.

## DBoW2 (Included in loop folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition

## g2o (Included in optimizer folder)
We use modified versions of the [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations.

But we can choose newest g2o by set(USE_G2O_NEWEST 1) in CMakeLists.txt

## cholmod
ubuntu: sudo apt-get install libsuitesparse-dev

## PCL (optional)
We use [pcl](https://github.com/PointCloudLibrary/pcl) for RGBD mode. **Tested with 1.10.0/1.9.0**.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed. **Tested with Neotic.**

# 3. Building VIEO_SLAM library and examples

Clone the repository:
```
git clone https://github.com/leavesnight/VIEO_SLAM.git VIEO_SLAM
```

We provide a script `build.sh` to build the submodule libraries and *VIEO_SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd VIEO_SLAM
chmod +x build.sh
./build.sh
```

This will create **libVIEO_SLAM.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

# 4. Monocular Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.bin Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.bin Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

# 5. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.bin Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.bin Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.bin Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

# 6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.bin Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 7. ROS Examples

### Building the nodes for Stereo/RGB-D + (IMU)
1. Add the path including *Examples/ROS/VIEO_SLAM* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned VIEO_SLAM:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/VIEO_SLAM/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Stereo_Inertial Node
For a stereo/RGB-D input from topic config `Camera.topic` and `Camera.topic2` + IMU data input from `IMU.topic`(refering to a non-existent topic can run VO),
run node VIEO_SLAM/Stereo_Inertial.
You will need to provide the vocabulary file and a settings file.

PS: If you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images may be pre-rectified**.

  ```
  rosrun VIEO_SLAM Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE IF_ONLINE_CLACHE SPARSE_MAP_NAME
  ```
  
**Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```
  
  ```
  rosrun VIEO_SLAM Stereo_Inertial Vocabulary/ORBvoc.bin Examples/Stereo/EuRoC.yaml false
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
Once VIEO_SLAM has loaded the vocabulary and Sensor topic has data, system will run. Enjoy it!.

Note: a powerful computer is required to run the most exigent sequences of these datasets.
  
# 8. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras.
We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the VIEO_SLAM library and how to pass images to the SLAM system.
Stereo input may be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

