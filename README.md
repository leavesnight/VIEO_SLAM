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

*WithFull BA Monocular/(d)Stereo VIO:*

**ATE Min Of Random 3 tests MonocularVIO Res(cm,Leica);feat num 1000:**
```
V101(1.4cm),V102(1.8),V103(4.4),V201(1.2),V202(1.3),V203(2.6),MH01(6.7),MH02(5.7),MH03(6.3),MH04(5.9),MH05(9.4)
PS: 2017/12/26
```
**ATE Random 1 test StereoVIO Res(m,Leica|Est\*);feat num 1000->375(default):**

**With XXX_distXXX.yaml--With XXX.yaml(undist as ref)**
```
V101 0.019|0.037 ->0.018|0.036 --0.019|0.036
V102 0.023|0.018 ->0.021|0.012 --0.023|0.018
V103 0.039|0.023 ->0.038|0.017 --0.040|0.021+
V201 0.014|0.014 ->0.019|0.017 --0.030|0.030
V202 0.016|0.014 ->0.012|0.009 --0.014|0.011
V203 0.017|0.016 ->0.020|0.015 --0.026|0.024-
MH01 0.060|0.016+->0.062|0.011 --0.069|0.024
MH02 0.047|0.015 ->0.052|0.016 --0.051|0.017
MH03 0.078|0.025 ->0.081|0.026 --0.076|0.024
MH04 0.081|0.044 ->0.086|0.058 --0.093|0.054-
MH05 0.100|0.039 ->0.137|0.080+--0.117|0.057+
Final parallel MH01~3 mean time cost per frame of frontend(ms): ~86 ->23 --62
PS:2023/5/26;Script(SetEuRoCFilesVIO.sh) On i7-12700H;+ meaning accuracy up(>=5mm) compared with before
Single MH05 test: 0.092|0.038 ->0.137|0.077---0.106|0.048+
tm cost(ms): 41 ->20 --30
```

### TUM Dataset

*Without Full BA dStereo VIO:*

**ATE Random tests (1|2|...) StereoVIO Res(m);feat num 1000(default)->375:**

**--With Full ORB3 St. as ref**
```
corridor1   0.031-->0.032---0.011|0.012
corridor2   0.024+->0.068---0.012|0.016
corridor3   0.016 ->0.044---0.009|0.010
corridor4   0.261-->0.214---0.039|0.098
corridor5   0.040-->0.097---0.010|0.010
magistrale1 0.107-->0.818---0.219|0.350
magistrale2 1.134-->2.114---0.339|1.390
magistrale3 1.437-->1.422+--2.268|2.404
magistrale4 0.194 ->1.430---1.276|0.170
magistrale5 0.643+->1.696---1.562|1.549
magistrale6 0.198+->3.531---1.000|0.906
outdoors1   8.752+->21.74---10.77|14.72
outdoors2   4.319-->8.438---11.09|13.73
outdoors3   2.373+->32.84---12.99|5.942
outdoors4   2.805+->14.84---3.568|4.345
outdoors5   5.017+->18.42---10.56|11.31
outdoors6   15.13-->23.07---24.06|46.62
outdoors7   1.338+->5.894---1.085|1.156
outdoors8   3.533-->7.606---10.37|10.03
room1       0.010 ->0.010 --0.010|0.010
room2       0.010 ->0.009 --0.007|0.008
room3       0.009 ->0.009 --0.008|0.007
room4       0.006 ->0.008 --0.008|0.007
room5       0.010 ->0.007 --0.010|0.008
room6       0.008 ->0.007 --0.006|0.007
slides1     0.126+->0.304---0.199|0.378
slides2     0.418-->0.875---0.638|0.627
slides3     0.413+->0.818---1.025|1.135
Final parallel slides3 mean time cost per frame of frontend(ms): ~46 ->21 --31
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
   windows:
   ```
   ./stereo_euroc.exe /c/zzh/4Life/VIEO_SLAM/Vocabulary/ORBvoc.bin /c/zzh/4Life/VIEO_SLAM/Examples/Stereo/EuRoC/EuRoC_VIO_dist_fast.yaml /d/zzh/2Work/Master/Master_Final/dataset/EuRoC/MH05difficult/mav0/cam0/data/ /d/zzh/2Work/Master/Master_Final/dataset/EuRoC/MH05difficult/mav0/cam1/data/ /c/zzh/4Life/VIEO_SLAM/Examples/Stereo/EuRoC/EuRoC_TimeStamps/MH05.txt /d/zzh/2Work/Master/Master_Final/dataset/EuRoC/MH05difficult/mav0/imu0/data.csv
   ```
   VO:(you can also use ./EuRoC.yaml with Tbc is default I)
   ```
    ./mono_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/V202medium/mav0/cam0/data ./EuRoC_TimeStamps/V202.txt 
    ./stereo_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml  ~/dataset/EuRoC/V203difficult/mav0/cam0/data ~/dataset/EuRoC/V203difficult/mav0/cam1/data ./EuRoC_TimeStamps/V203.txt
   ```
   VEO or VIEO: (you may refer to the kinect2_qhd.yaml, later we may publish our dataset containing encoders & IMU data)
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
We have tested the library in **Ubuntu 20.04**, but it should be easy to compile in other platforms.
A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface.
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.
**Tested with commit id 86eb4975fc4f.**

Build it from source (ensure you installed cmake before):
1. linux:
   ```
   // 1. download the source code
   git clone git@github.com:stevenlovegrove/Pangolin.git
   // PS: you may need to add your ssh key (.pub) to git website
   cd Pangolin
   git reset --hard 86eb4975fc4f
   // 2. compile
   mkdir build
   cd build && cmake ..
   make -j8 && sudo make install -j8
   ```
2. windows:
   1. download the source code (by git bash)
   ```
   git clone git@github.com:stevenlovegrove/Pangolin.git
   cd Pangolin
   git reset --hard 86eb4975fc4f
   ```
   2. then use installed windows CMake's cmake-gui to compile it:
      1. source code: C:/zzh/4Life/Pangolin
         build the binaries: C:/zzh/4Life/Pangolin/build
      2. Run cmake-gui as Administrator -> press Configure button (choose installed VS2022)
      PS: if no admin, doxygen may not able to be found!
      -> check BUILD_SHARED_LIBS
      -> config CMAKE_INSTALL_PREFIX: C:/zzh/4Life/Pangolin/build/x64/Release/Pangolin
      3. Configure -> press Generate button -> press Open Project
      4. Select Release -> Build/Build Solution
      PS: pangolin/ModelViewer subprojects may enter error, we right click ALL_BUILD and each FAILED one
      and choose Properties, pick c++17 and then build it again
      Still 3 failed but pangolin.lib&dll ready!
      5. Add C:\zzh\4Life\Pangolin\build\src\Release to environment variable PATH
      Add %OPENNI2_REDIST64% or C:\Program Files\OpenNI2\Redist or C:\Program Files\OpenNI2\Tools to environment variable PATH

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features.
Dowload and install instructions can be found at: http://opencv.org.
**Required at least 2.4.3. Tested with OpenCV 4.5/3.2.0; OpenCV 4.5(Windows)**.

Refer to the official install guide: https://docs.opencv.org/4.x/df/d65/tutorial_table_of_content_introduction.html

Or Build it from source (ensure you installed cmake before):
1. linux:
   ```
   // 1. download the source code
   // if you wanna to use 3.2.0, u'd better use our compile-fixed version: https://github.com/leavesnight/opencv/tree/320
   git clone git@github.com:opencv/opencv.git
   cd opencv
   git reset --hard 4.5.0
   // 2. compile is the same as before
   ```
2. windows:
   1. download the source code (the same as linux)
   2. then use cmake-gui to compile:
      1. source: C:\zzh\4Life\opencv; binaries: C:\zzh\4Life\opencv\build
      2. Configure -> Generate -> Open Project
      3. Release -> Build Solution
      4. Run VS2022 as Administrator, then choose INSTALL subproject -> Build Solution
      5. Add OpenCV_DIR=C:/zzh/4Life/opencv/build/install/lib to envrionment variables
      6. Add C:\zzh\4Life\opencv\build\install\bin to environment variable PATH

## Eigen3
Required by our project and g2o (see below). Test shows it's faster than opencv cv::Mat.
Download and install instructions can be found at: http://eigen.tuxfamily.org.
**Required at least 3.1.0. Tested with Eigen 3.3.7/3.2.10; Eigen 3.4(Windows)**.

Refer to the official install guide: https://eigen.tuxfamily.org/dox-3.3/GettingStarted.html
https://gitlab.com/libeigen/eigen/-/releases

Or Build it from source (ensure you installed cmake before):
1. linux:
   ```
   // 1. download the source code
   git clone git@gitlab.com:libeigen/eigen.git
   cd eigen
   git reset --hard 3.3.7
   // 2. compile is the same as before
   ```
2. windows:
   1. download the source code (the same as linux except origin/3.4 instead of 3.3.7)
   2. then use cmake-gui to compile:
      1. source: C:\zzh\4Life\eigen; binaries: C:\zzh\4Life\eigen\build
      2. Configure
      -> config CMAKE_INSTALL_PREFIX: C:/zzh/4Life/eigen/build/x64/Release/Eigen3
      -> Configure -> Generate( -> Open Project
      3. Release -> Build Solution
      PS: some subprojects may enter internal compile error, we right click each
      and choose Properties/C++/Optimization, pick /Od and then build it again,
      but maybe there's a better solution, you can provide it to me through email~)
      4. Choose INSTALL subproject -> Build Solution
      5. Add Eigen3_DIR=C:/zzh/4Life/eigen/build/x64/Release/Eigen3/share/eigen3/cmake to environment variables,
         can be found by our changed FindEigen3.cmake
      PS: for some windows may still compile error, please use set(EIGEN3_INCLUDE_DIR "C:/zzh/4Life/eigen")
      after find_package(Eigen3 3.3.7 MODULE REQUIRED)

## Ceres (optional, now unused)
For Sophus can depend on ceres, we provide the ceres install process and version here (linux/windows git bash):
1. linux: refer to the offical install guide: http://ceres-solver.org/installation.html
2. windows:
   1. download the ceres source code
   ```
   git clone https://ceres-solver.googlesource.com/ceres-solver
   cd ceres-solver/
   rm -r BUILD
   git clone git@github.com:google/glog.git
   ```
   2. then use installed windows CMake's cmake-gui to config it:
      1. dependency glog compile by installed VS2022
         1. source: C:/zzh/4Life/ceres-solver/glog; binaries: C:/zzh/4Life/ceres-solver/glog/build
         2. Configure -> ... (the same as before)
      2. ceres compile:
         1. source: C:/zzh/4Life/ceres-solver; binaries: C:/zzh/4Life/ceres-solver/build
         2. Configure -> config glog_DIR: C:\zzh\4Life\ceres-solver\glog\build
         3. Configure -> ... (the same as before)

## Sophus
We use [Sophus](https://github.com/strasdat/Sophus) for more compact translation and rotation operation.
**Tested with commit id 780cd0fce405.**

Then Build it from source:
1. linux:
   ```
   // 1. download the source code
   git clone git@github.com:strasdat/Sophus.git
   cd Sophus
   git reset --hard 780cd0fce405
   // 2. compile is the same as before
   ```
2. windows:
   1. download the source code (the same as linux)
   2. then use cmake-gui to compile:
      1. source: C:\zzh\4Life\Sophus; binaries: C:\zzh\4Life\Sophus\build
      2. Configure -> config EIGEN3_INCLUDE_DIR: C:\zzh\4Life\eigen
      -> config CMAKE_INSTALL_PREFIX: C:/zzh/4Life/Sophus/build/x64/Sophus
      (-> config Ceres_DIR: C:\zzh\4Life\ceres-solver\build)
      3. Configure -> Generate -> Open Project
      4. Release -> Build Solution
      5. Choose INSTALL subproject -> Build Solution
      6. Change CMakeList.txt: `set(Sophus_INCLUDE_DIR "C:/zzh/4Life/Sophus/build/x64/Sophus/include")`

## DBoW2 (Included in loop folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition

## g2o (Included in optimizer folder)
We use modified versions of the [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations.

But we can choose newest g2o by set(USE_G2O_NEWEST 1) in CMakeLists.txt

## PCL
We use [pcl](https://github.com/PointCloudLibrary/pcl) for RGBD mode. **Tested with 1.10.0/1.9.0**.

PS: Not required when not build rgbd_tum，please change the code by yourself

Refer to the official install guide: https://pointclouds.org/downloads/
1. linux: simply follow the guide
2. windows:
   1. press [Release](https://github.com/PointCloudLibrary/pcl/releases) and find 1.10.0,
   then download Assets/PCL-1.10.0-AllInOne-msvc2019-win64.exe and install it
   2. change C:\zzh\4Life\pcl\build_all_in_one\PCL 1.10.0\3rdParty\Boost\include\boost-1_72\boost\signals2\detail\auto_buffer.hpp:
   //typedef typename Allocator::pointer              allocator_pointer;
   using allocator_pointer = std::allocator_traits<std::allocator<T>>::pointer; // for c++20 compile

   PS: It's recommended to pick adding path to environment variables, our final result is PCL_ROOT=
   C:\zzh\4Life\pcl\build_all_in_one\PCL 1.10.0

Or Build it from source (ensure you installed cmake before):
1. linux:
   ```
   // 1. download the source code
   git clone git@github.com:PointCloudLibrary/pcl.git
   cd pcl
   git reset --hard 1.10.0
   // 2. compile by forcing Release
   mkdir build
   cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j8 && sudo make install
   ```
2. windows: **unTested**

## cholmod (optional, now unused)
ubuntu: sudo apt-get install libsuitesparse-dev

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org).
Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.
**Tested with Neotic.**

# 3. Building VIEO_SLAM library and examples

Clone the repository:
```
git clone https://github.com/leavesnight/VIEO_SLAM.git VIEO_SLAM
```

Please make sure you have installed all required dependencies (see section 2).

1. linux: We provide a script `build.sh` to build the submodule libraries and *VIEO_SLAM*.

   Execute:
   ```
   cd VIEO_SLAM
   chmod +x build.sh
   ./build.sh
   ```

This will create **libVIEO_SLAM.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

2. windows: We use CMakeSettings.json of Visual Studio 2022 to compile and link this code.
   1. Select x64-Release -> Build/Build All
   2. Add C:\zzh\4Life\VIEO_SLAM\bin\Release to environment variable PATH

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

