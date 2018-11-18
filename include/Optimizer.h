

#ifndef OPTIMIZER_H
#define OPTIMIZER_H


#include "Global.h"

namespace devo {

class Optimizer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // Optimizer();
    // ~Optimizer();
    std::vector<SE3> T_camera;
    std::vector<SE3> T_vicon;

    int optimizeLoop(SE3 &Tcm, SE3 &Tcv);

};

} // 

#endif // OPTIMIZER_H
