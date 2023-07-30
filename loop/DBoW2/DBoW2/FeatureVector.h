/**
 * File: FeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_FEATURE_VECTOR__
#define __D_T_FEATURE_VECTOR__

#include "BowVector.h"

namespace DBoW2 {

/// Vector of nodes with indexes of local features
class FeatureVector : public std::map<NodeId, std::vector<unsigned int> > {
 public:
  /**
   * Constructor
   */
  FeatureVector(void);

  /**
   * Destructor
   */
  ~FeatureVector(void);

  /**
   * Adds a feature to an existing node, or adds a new node with an initial
   * feature
   * @param id node id to add or to modify
   * @param i_feature index of feature to add to the given node
   */
  void addFeature(NodeId id, unsigned int i_feature);

  /**
   * Sends a string versions of the feature vector through the stream
   * @param out stream
   * @param v feature vector
   */
  friend std::ostream &operator<<(std::ostream &out, const FeatureVector &v);

  // For possible serialize
  /**
   * Serialize of reading words
   * @param is input stream with n_id0, n0, n0_val0, n0_val1..., n0_valn0-1, n_id1... order
   */
  typedef std::vector<unsigned int> TypeSec;
  inline void read(std::istream &is) {
    unsigned int idFeat;
    size_t vecSize;
    size_type nsize;
    is.read((char *)&nsize, sizeof(nsize));
    for (size_t i = 0; i < nsize; ++i) {
      NodeId id;
      is.read((char *)&(id), sizeof(id));
      is.read((char *)&vecSize, sizeof(vecSize));
      for (size_t j = 0; j < vecSize; ++j) {
        is.read((char *)&idFeat, sizeof(idFeat));
        addFeature(id, idFeat);
      }
    }
  }
  /**
   * Serialize of writing words
   * @param os output stream with n_id0, n0, n0_val0, n0_val1..., n0_valn0-1, n_id1... order
   */
  inline void write(std::ostream &os) const {
    size_type nsize = size();
    os.write((char *)&nsize, sizeof(nsize));
    for (const_iterator iter = begin(); iter != end(); ++iter) {
      os.write((char *)&(iter->first), sizeof(iter->first));
      const TypeSec &vec = iter->second;
      size_t vecSize = vec.size();
      os.write((char *)&vecSize, sizeof(vecSize));
      for (TypeSec::const_iterator iter2 = vec.begin(); iter2 != vec.end(); ++iter2) {
        os.write((char *)&(*iter2), sizeof(*iter2));
      }
    }
  }
};

}  // namespace DBoW2

#endif
