//
// Created by leavesnight on 10/18/23.
//

#pragma once

#include "g2otypes.h"

namespace g2o {
class GraphOperator {
 public:
  // here the degree number is related to the residual dimension, not the variable to be optimized
  static constexpr int NumChi2_ = 15 + 1;
  static constexpr float chi2_sig5_[NumChi2_] = {0,      3.841,  5.991,  7.815,  9.488,  11.070, 12.592, 14.067,
                                                 15.507, 16.919, 18.307, 19.675, 21.026, 22.362, 23.685, 24.996};

  // do it before initializeOptimization(=0) for init will change activeEdges in g2o to do solve() in optimize()
  template <class Graph, class Edges>
  static inline void Chi2LargeSetLevel(Graph& optimizer, Edges& es, const int dim_freedom, const float rat_th_chi2 = 1.,
                                       const bool ballocate_jacs = true);
};

template <class Graph, class Edges>
void GraphOperator::Chi2LargeSetLevel(Graph& optimizer, Edges& es, const int dim_freedom, const float rat_th_chi2,
                                      const bool ballocate_jacs) {
  for (auto iter = es.begin(), iterend = es.end(); iter != iterend; ++iter) {
    auto& e = iter->pedge;
    e->computeError();
    if (e->chi2() > rat_th_chi2 * chi2_sig5_[dim_freedom]) {
      e->setLevel(1);
    }
  }
  if (ballocate_jacs) {
    optimizer.initializeOptimization();
    for (auto iter = es.begin(), iterend = es.end(); iter != iterend; ++iter) {
      auto& e = iter->pedge;
      e->linearizeOplus(optimizer.jacobianWorkspace());
    }
  }
}

}  // namespace g2o
