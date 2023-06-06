# VIEO_SLAM
Now this project is improved by zzh

1.Some annotations and elimination are done

2.VIORBSLAM is implemented like the VIORBSLAM paper[Visual-Inertial Monocular SLAM with Map Reuse](https://arxiv.org/pdf/1610.05949.pdf), especially thanks to OnManifold Preintegration paper[On-Manifold Preintegration for Real-Time](https://arxiv.org/pdf/1512.02363.pdf) and [JingWang's initial version](https://github.com/jingpang/LearnVIORB)

3.It's a real-time code with full ba at last, PoseOptimization uses PVR vertex while localBA/GlobalBA use PR-V, No inverse depth parameterization, Loading uses .bin vocabulary, Some other related changes

4.Stereo/RGBD+IMU version is completed, thought it's slow, it doesn't have unstable initialization problem on V103 & may succeed totally on V203 if PC is nice & it usually has better non-GT scaled accuracy. We call it VIORBSLAM2 for it supports three kinds of cameras.

5.An RGBD/Stereo version with Encoder+IMU, Encoder has already come, we call it VIEORBSLAM2, VEORBSLAM2 respectively for it's based on ORBSLAM2 & VIORBSLAM.
Though the code now cannot be applied for Monocular+Encoder due to the lack of pure encoder edges insertion strategy and scale initialization,
it's not too difficult to implement and we have considered that the Initialization of Monocular SLAM is not suitable for a Differential Wheeled Robot based on our experience.

6.We named the total system as VIEO_SLAM, where O should finally means the other odometers instead of ORB descriptor currently.
We hope to provide a general visual tightly-coupled with any odometer SLAM system in the end.

## Test

### EuRoC Dataset

*WithFull BA VIORBSLAM:*

**ATE Min Of Random 3 tests MonocularVIO Res(cm,Leica);feat num 1000:**
```
V101(1.4cm),V102(1.8),V103(4.4),V201(1.2),V202(1.3),V203(2.6),MH01(6.7),MH02(5.7),MH03(6.3),MH04(5.9),MH05(9.4)
PS: 2017/12/26
```
**ATE Random 1 test StereoVIO Res(m,Leica|Est\*);feat num 1000->375(default):**

**With XXX_distXXX.yaml--With XXX.yaml(undist as ref)**
```
V101 0.019|0.037 ->0.018|0.036 --0.019|0.036
V102 0.023|0.018 ->0.023|0.014 --0.023|0.018
V103 0.039|0.023 ->0.038|0.017 --0.040|0.021+
V201 0.014|0.014 ->0.016|0.015 --0.030|0.030
V202 0.016|0.014 ->0.012|0.010 --0.014|0.011
V203 0.017|0.016 ->0.021|0.018 --0.026|0.024-
MH01 0.060|0.016+->0.065|0.018 --0.069|0.024
MH02 0.047|0.015 ->0.055|0.016 --0.051|0.017
MH03 0.078|0.025 ->0.084|0.025 --0.076|0.024
MH04 0.081|0.044 ->0.094|0.053 --0.093|0.054-
MH05 0.100|0.039 ->0.144|0.087---0.117|0.057+
Final parallel MH01~3 mean time cost per frame of frontend(ms): ~86 ->43 --62
PS:2023/5/26;Script(SetEuRoCFilesVIO.sh) On i7-12700H;+ meaning accuracy up(>=5mm) compared with before
Single MH05 test: 0.092|0.038 ->0.137|0.077---0.106|0.048+
tm cost(ms): 41 ->20 --30
```

### TUM Dataset

*Without Full BA VIORBSLAM:*

**ATE Random tests (1|2|...) StereoVIO Res(m);feat num 1000(default)->375:**

**--With Full ORB3 St. as ref**
```
corridor1   0.018-->0.032---0.011|0.012
corridor2   0.059-->0.068---0.012|0.016
corridor3   0.013 ->0.044---0.009|0.010
corridor4   0.176-->0.214---0.039|0.098
corridor5   0.025-->0.097---0.010|0.010
magistrale1 0.093-->0.818---0.219|0.350
magistrale2 0.506+->2.114---0.339|1.390
magistrale3 1.381+->1.422+--2.268|2.404
magistrale4 0.197-->1.430---1.276|0.170
magistrale5 0.664+->1.696---1.562|1.549
magistrale6 0.584-->3.531---1.000|0.906
outdoors1   12.40-->21.74---10.77|14.72
outdoors2   3.120+->8.438---11.09|13.73
outdoors3   3.465+->32.84---12.99|5.942
outdoors4   3.367+->14.84---3.568|4.345
outdoors5   5.903-->18.42---10.56|11.31
outdoors6   13.92-->23.07---24.06|46.62
outdoors7   1.710-->5.894---1.085|1.156
outdoors8   2.770+->7.606---10.37|10.03
room1       0.011 ->0.010 --0.010|0.010
room2       0.012 ->0.009 --0.007|0.008
room3       0.009 ->0.009 --0.008|0.007
room4       0.007 ->0.008 --0.008|0.007
room5       0.006 ->0.007 --0.010|0.008
room6       0.007 ->0.007 --0.006|0.007
slides1     0.685-->0.304---0.199|0.378
slides2     0.114+->0.875---0.638|0.627
slides3     0.605-->0.818---1.025|1.135
Final parallel slides3 mean time cost per frame of frontend(ms): ~44 ->21 --31
PS:2023/6/2;Script(SetEuRoCFilesVIO.sh) On i7-12700H;+ meaning accuracy up(>=5mm) compared with before
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
For a closed-source version of VIEO_SLAM for commercial purposes, please contact the authors: zhuzhanghao9331@yahoo.co.jp; orbslam (at) unizar (dot) es.

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

We use [Sophus](https://github.com/strasdat/Sophus) for more compact translation and rotation operation. **Tested with commit id 780cd0fce405.**

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 4.5/3.2.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0. Tested with Eigen 3.3.7/3.2.10**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## cholmod
ubuntu: sudo apt-get install libsuitesparse-dev

## PCL (optional)
We use [pcl](https://github.com/PointCloudLibrary/pcl) for VEO/VIEO mode. **Tested with 1.10.0/1.9.0**.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

# 3. Building VIEO_SLAM library and examples

Clone the repository:
```
git clone https://github.com/leavesnight/VIEO_SLAM.git VIEO_SLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *VIEO_SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
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
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

# 5. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
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
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 7. ROS Examples

### Building the nodes for mono, monoAR, stereo and RGB-D
1. Add the path including *Examples/ROS/VIEO_SLAM* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned VIEO_SLAM:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/VIEO_SLAM/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node VIEO_SLAM/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun VIEO_SLAM Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Monocular Augmented Reality Demo
This is a demo of augmented reality where you can use an interface to insert virtual cubes in planar regions of the scene.
The node reads images from topic `/camera/image_raw`.

  ```
  rosrun VIEO_SLAM MonoAR PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node VIEO_SLAM/Stereo. You will need to provide the vocabulary file and a settings file. If you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**.

  ```
  rosrun VIEO_SLAM Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
**Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```
  
  ```
  rosrun VIEO_SLAM Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
Once VIEO_SLAM has loaded the vocabulary, press space in the rosbag tab. Enjoy!. Note: a powerful computer is required to run the most exigent sequences of this dataset.

### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node VIEO_SLAM/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun VIEO_SLAM RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
# 8. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the VIEO_SLAM library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

