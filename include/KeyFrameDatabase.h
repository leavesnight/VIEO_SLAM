/**
 * This file is part of VIEO_SLAM
 */

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include <mutex>

namespace VIEO_SLAM {

class KeyFrame;
class Frame;

class KeyFrameDatabase {
 public:
  KeyFrameDatabase(const ORBVocabulary& voc);

  void add(KeyFrame* pKF);  // mvInvertedFile[pKF->mBowVec[i].first].push_back(pKF)

  void erase(KeyFrame* pKF);  // erase pKF from the list of mvInvertedFile[pKF->mBowVec[i].WordID]

  void clear();

  // Loop Detection
  // use pKF->mBowVec&&(mvInvertedFile/KFDatabase) to quickly find the KFs(not in pKF's covisible neighbors,enough
  // CommonWorlds) whose 10 best covisible neighbors have enough
  // accScore(sigma(score(CovNeighKF->mBowVec,F->mBowVec))),but return the best mLoopScore KF in CovNeighKFs(including
  // KFCWs) as the candidate
  std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);

  // Relocalization
  // use F->mBowVec to quickly find the KFs whose covisible neighbors have enough
  // accScore(sigma(score(CovNeighKF->mBowVec,F->mBowVec))), but return the best mRelocScore KF in CovNeighKFs as the
  // candidate
  std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

 protected:
  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

}  // namespace VIEO_SLAM

#endif
