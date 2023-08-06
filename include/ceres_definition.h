#include <iostream>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include "sophus/se3.hpp"

using namespace std;

class PoseParameterizationSE3Sophus : public ceres::LocalParameterization
{
    public:
        virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
        virtual bool ComputeJacobian(const double *x, double *jacobian) const;
        virtual int GlobalSize() const { return 6; };
        virtual int LocalSize() const { return 6; };
};

class PoseCostFactor : public ceres::SizedCostFunction<6, 6, 6>
{
    public:
        PoseCostFactor(double x, double y, double z, double w, double qx, 
                                        double qy, double qz);
        virtual bool Evaluate(double const* const* pose, double *residual, 
                                        double **jacobians) const;
        Sophus::SE3<double> poseSE3_;
        double x_, y_, z_, w_, qx_, qy_, qz_;
};