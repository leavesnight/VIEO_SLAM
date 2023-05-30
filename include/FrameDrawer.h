/**
* This file is part of VIEO_SLAM
*/

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"
#include "common/common.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace VIEO_SLAM
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame(int cami);

    size_t n_cams_ = 1;
    bool showallimages_ = false;

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    vector<cv::Mat> mIms;
    int N;
    std::vector<std::pair<size_t, size_t>> mapn2in_;
    vector<cv::KeyPoint> mvCurrentKeys;
#ifdef DRAW_KP2MP_LINE
    typedef struct _KptDraw {
      cv::Point2f pt;
      bool valid = false;
    } KptDraw;
    vector<KptDraw> kpts_proj_;
#endif
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;

    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
