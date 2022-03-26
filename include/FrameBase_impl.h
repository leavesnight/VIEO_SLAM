//
// Created by leavesnight on 2022/3/26.
//

#ifndef VIEO_SLAM_FRAMEBASE_IMPL_H
#define VIEO_SLAM_FRAMEBASE_IMPL_H

#include "KeyFrame.h"

namespace VIEO_SLAM {
inline void ErasePairObs(KeyFrame *pFBi, MapPoint *pMPi, size_t idx = -1) {
  if (1) {
    if (-1 == idx)
      pFBi->EraseMapPointMatch(pMPi);
    else
      pFBi->EraseMapPointMatch(idx);
  }
  // here may erase fixed pMP in mpMap through MP::SetBadFlag()
  pMPi->EraseObservation(pFBi, idx);
}
}

#endif  // VIEO_SLAM_FRAMEBASE_IMPL_H
