//
// Created by leavesnight on 6/14/23.
//

#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include "common/interface.h"

namespace VIEO_SLAM {
class COMMON_API Serialize {
 public:
  template <typename _Tp>
  using vector = std::vector<_Tp>;
  using ostream = std::ostream;
  using istream = std::istream;

  // can also be used for set/list
  template <class T>
  static bool writeVec(ostream &os, const T &vec);
  template <class T>
  static bool writeVecwrite(ostream &os, const T &lis);
  template <class T>
  static bool writeVecEigMat(ostream &os, const T &lis);
  static bool writeMat(ostream &os, const cv::Mat &mat);
  // for Eigen::Matrix<_Scalar,_Rows,_Cols>
  template <class T>
  static bool writeEigMat(ostream &os, const T &mat);
  template <class T>
  static bool readVec(istream &is, T &vec);
  template <class T>
  static bool readVecread(istream &is, T &lis);
  template <class T>
  static bool readVecEigMat(istream &is, T &lis);
  static bool readMat(istream &is, cv::Mat &mat);
  template <class T>
  static bool readEigMat(istream &is, T &mat);
};

template <class T>
bool Serialize::writeVec(ostream &os, const T &vec) {
  for (typename T::const_iterator iter = vec.begin(); iter != vec.end(); ++iter) {
    os.write((char *)&(*iter), sizeof(*iter));
  }
  return os.good();
}
template <class T>
bool Serialize::writeVecwrite(ostream &os, const T &lis) {
  for (typename T::const_iterator iter = lis.begin(); iter != lis.end(); ++iter) iter->write(os);
  return os.good();
}
template <class T>
bool Serialize::writeVecEigMat(ostream &os, const T &lis) {
  for (typename T::const_iterator iter = lis.begin(); iter != lis.end(); ++iter) writeEigMat(os, *iter);
  return os.good();
}
template <class T>
bool Serialize::writeEigMat(ostream &os, const T &mat) {
  // mat.size()==mat.rows()*mat.cols(), saved
  os.write((char *)mat.data(), mat.size() * sizeof(typename T::Scalar));
  // acquiescently as the column-major order
  return os.good();
}
template <class T>
bool Serialize::readVec(istream &is, T &vec) {
  for (typename T::iterator iter = vec.begin(); iter != vec.end(); ++iter) {
    is.read((char *)&(*iter), sizeof(*iter));
  }
  return is.good();
}
template <class T>
bool Serialize::readVecread(istream &is, T &lis) {
  for (typename T::iterator iter = lis.begin(); iter != lis.end(); ++iter) iter->read(is);
  return is.good();
}
template <class T>
bool Serialize::readVecEigMat(istream &is, T &lis) {
  for (typename T::iterator iter = lis.begin(); iter != lis.end(); ++iter) readEigMat(is, *iter);
  return is.good();
}
template <class T>
bool Serialize::readEigMat(istream &is, T &mat) {
  is.read((char *)mat.data(), mat.size() * sizeof(typename T::Scalar));
  return is.good();
}

// speicalize
template <>
COMMON_API bool Serialize::writeVec(ostream &os, const vector<bool> &vec);
template <>
COMMON_API bool Serialize::readVec(istream &is, vector<bool> &vec);

}  // namespace VIEO_SLAM