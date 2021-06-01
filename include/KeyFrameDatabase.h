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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace VIEO_SLAM
{

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);//mvInvertedFile[pKF->mBowVec[i].first].push_back(pKF)

   void erase(KeyFrame* pKF);//erase pKF from the list of mvInvertedFile[pKF->mBowVec[i].WordID]

   void clear();

   // Loop Detection
   //use pKF->mBowVec&&(mvInvertedFile/KFDatabase) to quickly find the KFs(not in pKF's covisible neighbors,enough CommonWorlds) \
   whose 10 best covisible neighbors have enough accScore(sigma(score(CovNeighKF->mBowVec,F->mBowVec))),\
   but return the best mLoopScore KF in CovNeighKFs(including KFCWs) as the candidate
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   //use F->mBowVec to quickly find the KFs whose covisible neighbors have enough accScore(sigma(score(CovNeighKF->mBowVec,F->mBowVec))), but return the best mRelocScore KF in CovNeighKFs as the candidate
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
