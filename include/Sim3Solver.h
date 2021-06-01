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


#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"

namespace VIEO_SLAM
{

//3D-2D solver, similar to PnPSolver
class Sim3Solver
{
public:

    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    cv::Mat GetEstimatedRotation();
    cv::Mat GetEstimatedTranslation();
    float GetEstimatedScale();


protected:

    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    void CheckInliers();

    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);//Project vP3Dc(vector<cv::Mat(3,1,CV_32F)>) camera (3D) coordinate to \
    vP2D(vector<cv::Mat_<float>(2,1)>) image (2D) coordinate through camera intrinsic matrix K


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1;//mpCurrentKF
    KeyFrame* mpKF2;//loop candidate KFs

    //all the following vec has the same size<=mN1
    std::vector<cv::Mat> mvX3Dc1;//matched MPs' Xc1
    std::vector<cv::Mat> mvX3Dc2;//matched MPs' Xc2
    std::vector<MapPoint*> mvpMapPoints1;//matched MPs in pKF1->mvpMapPoints
    std::vector<MapPoint*> mvpMapPoints2;//matched MPs in pKF2->mvpMapPoints, notice the matched MPs mean 2 temporary (may)different MPs
    std::vector<MapPoint*> mvpMatches12;//mvpMatches12[i] matched to mpKF1->mvpMapPoints[i]
    std::vector<size_t> mvnIndices1;//matched MPs' index in pKF1
    //std::vector<size_t> mvSigmaSquare1;//unused
    //std::vector<size_t> mvSigmaSquare2;//unused
    std::vector<size_t> mvnMaxError1;//element is chi2(0.01,2)*sigma2 for pKF1->mvpMapPoints[i](matched ones have MaxError1)
    std::vector<size_t> mvnMaxError2;//for pKF2->mvpMapPoints[j]/mvpMatches12[i](matched ones have MaxError2)

    int N;
    int mN1;//number/size of the mvpMatches12/mpKF1->mvpMapPoints

    // Current Estimation
    cv::Mat mR12i;
    cv::Mat mt12i;
    float ms12i;
    cv::Mat mT12i;
    cv::Mat mT21i;
    std::vector<bool> mvbInliersi;
    int mnInliersi;

    // Current Ransac State
    int mnIterations;
    std::vector<bool> mvbBestInliers;
    int mnBestInliers;
    cv::Mat mBestT12;
    cv::Mat mBestRotation;
    cv::Mat mBestTranslation;
    float mBestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale;

    // Indices for random selection
    std::vector<size_t> mvAllIndices;//size is the number of matched 2 MPs <=mN1, recording the index of entering vector(mvX3Dc1...)

    // Projections, size is the same as mvX3Dc1.../mvAllIndices
    std::vector<cv::Mat> mvP1im1;//image coordinate of matched MPs in pKF1
    std::vector<cv::Mat> mvP2im2;//vec<matched MPs' image coordinate in pKF2>

    // RANSAC probability
    double mRansacProb;

    // RANSAC min inliers
    int mRansacMinInliers;

    // RANSAC max iterations
    int mRansacMaxIts;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // Calibration
    cv::Mat mK1;//for pKF1
    cv::Mat mK2;//for pKF2

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
