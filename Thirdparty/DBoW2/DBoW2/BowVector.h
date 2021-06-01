/**
 * File: BowVector.h
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_BOW_VECTOR__
#define __D_T_BOW_VECTOR__

#include <iostream>
#include <map>
#include <vector>

namespace DBoW2 {

/// Id of words
typedef unsigned int WordId;

/// Value of a word
typedef double WordValue;

/// Id of nodes in the vocabulary treee
typedef unsigned int NodeId;

/// L-norms for normalization
enum LNorm
{
  L1,
  L2
};

/// Weighting type
enum WeightingType
{
  TF_IDF,
  TF,
  IDF,
  BINARY
};

/// Scoring type
enum ScoringType
{
  L1_NORM,
  L2_NORM,
  CHI_SQUARE,
  KL,
  BHATTACHARYYA,
  DOT_PRODUCT,
};

/// Vector of words to represent images
class BowVector: 
	public std::map<WordId, WordValue>
{
public:

	/** 
	 * Constructor
	 */
	BowVector(void);

	/**
	 * Destructor
	 */
	~BowVector(void);
	
	/**
	 * Adds a value to a word value existing in the vector, or creates a new
	 * word with the given value
	 * @param id word id to look for
	 * @param v value to create the word with, or to add to existing word
	 */
	void addWeight(WordId id, WordValue v);
	
	/**
	 * Adds a word with a value to the vector only if this does not exist yet
	 * @param id word id to look for
	 * @param v value to give to the word if this does not exist
	 */
	void addIfNotExist(WordId id, WordValue v);

	/**
	 * L1-Normalizes the values in the vector 
	 * @param norm_type norm used
	 */
	void normalize(LNorm norm_type);
	
	/**
	 * Prints the content of the bow vector
	 * @param out stream
	 * @param v
	 */
	friend std::ostream& operator<<(std::ostream &out, const BowVector &v);
	
	/**
	 * Saves the bow vector as a vector in a matlab file
	 * @param filename
	 * @param W number of words in the vocabulary
	 */
	void saveM(const std::string &filename, size_t W) const;
	
//added by zzh:
	inline void read(std::istream &is){
	  WordId wId;WordValue wVal;
	  size_type nsize;
	  is.read((char*)&nsize,sizeof(nsize));
	  for (int i=0;i<nsize;++i){
	    is.read((char*)&(wId),sizeof(wId));
	    is.read((char*)&(wVal),sizeof(wVal));
	    addWeight(wId,wVal);
	  }
	}
	inline void write(std::ostream &os) const{
	  size_type nsize=size();
	  os.write((char*)&nsize,sizeof(nsize));
	  for (const_iterator iter=begin();iter!=end();++iter){
	    os.write((char*)&(iter->first),sizeof(iter->first));
	    os.write((char*)&(iter->second),sizeof(iter->second));
	  }
	}
};

} // namespace DBoW2

#endif
