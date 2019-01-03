

#include "Optimizer.h"

#include "thirdparty/g2o/g2o/core/block_solver.h"
#include "thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Frame.h"
#include "CameraModel.h"
#include "Frame.h"
#include <mutex>
#include "Converter.h"
namespace devo {

int
Optimizer::optimizeLoop(SE3 &Tcm, SE3 &Tcv)
{

    T_camera.push_back(Tcm);
    T_vicon.push_back(Tcv);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType* linearSolver;


    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);


    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();

    vSE3->setId(0);
    vSE3->setMarginalized(false);
    vSE3->setEstimate( Converter::toSE3Quat(SE3(Mat33::Identity(),Vec3(0.0,0.0,-0.0))));
    optimizer.addVertex(vSE3);

    int interv = 5;
    std::cout << "data size: " << T_vicon.size() << std::endl;
    for (int i = 0; i < (int(T_vicon.size())-interv); ++i)
    {
        g2o::EdgeSE3Loop* e = new g2o::EdgeSE3Loop();
		e->T1 = Converter::toSE3Quat(T_camera[i+interv]*T_camera[i].inverse());
		e->T2 = Converter::toSE3Quat(T_vicon[i+interv]*T_vicon[i].inverse());
        // std::cout << "x1: " << (T_camera[i+interv]*T_camera[i].inverse()).matrix() << std::endl;
        // std::cout << "x2: " << (T_vicon[i+interv]*T_vicon[i].inverse()).matrix() << std::endl;
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setMeasurement(Vec6::Zero());
        Mat66 Info = Mat66::Identity();
        e->setInformation(Info);
 
        optimizer.addEdge(e);

    }
    optimizer.initializeOptimization(0);
    optimizer.optimize(1000);

    g2o::SE3Quat SE3quat_rest = vSE3->estimate();
    std::cout << "the result: " << SE3quat_rest.to_homogeneous_matrix() << std::endl;

}


} 
