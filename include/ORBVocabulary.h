/**
* This file is part of VIEO_SLAM
*/


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include"loop/DBoW2/DBoW2/FORB.h"
#include"loop/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace VIEO_SLAM
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
