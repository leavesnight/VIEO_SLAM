//
// Created by leavesnight on 6/14/23.
//

#include "serialize.h"

namespace VIEO_SLAM {

bool Serialize::writeMat(ostream &os, const cv::Mat &mat) {
  for (int i = 0; i < mat.rows; ++i) {
    os.write((char *)mat.ptr(i), mat.cols * mat.elemSize());
  }
  return os.good();
}
bool Serialize::readMat(istream &is, cv::Mat &mat) {
  for (int i = 0; i < mat.rows; ++i) {
    is.read((char *)mat.ptr(i), mat.cols * mat.elemSize());
  }
  return is.good();
}

template <>
bool Serialize::writeVec(ostream &os, const vector<bool> &vec) {
  for (typename vector<bool>::const_iterator iter = vec.begin(); iter != vec.end(); ++iter) {
    bool btmp = *iter;
    os.write((char *)&btmp, sizeof(btmp));
  }
  return os.good();
}
template <>
bool Serialize::readVec(istream &is, vector<bool> &vec) {
  for (typename vector<bool>::iterator iter = vec.begin(); iter != vec.end(); ++iter) {
    bool btmp;
    is.read((char *)&btmp, sizeof(btmp));
    *iter = btmp;
  }
  return is.good();
}

}  // namespace VIEO_SLAM