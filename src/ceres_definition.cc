#include <include/ceres_definition.h>

bool PoseParameterizationSE3Sophus::Plus(const double *x, 
            const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> _x(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> _delta(delta);

    Sophus::SE3d T = Sophus::SE3d::exp(_x);
    Sophus::SE3d delta_T = Sophus::SE3d::exp(_delta);
    Eigen::Matrix<double, 6, 1> x_update = (delta_T * T).log();

    for(int i =0; i<6; i++)
        x_plus_delta[i] = x_update[i];

    return true;
}

bool PoseParameterizationSE3Sophus::ComputeJacobian(
            const double *x, double *jacobian) const
{
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6,6);
    return true;
}

PoseCostFactor::PoseCostFactor(double x, double y, double z, double w, 
            double qx, double qy, double qz)
{
    Eigen::Quaterniond q(w, qx, qy, qz);
    q.normalize();
    poseSE3_ = Sophus::SE3d(q, Eigen::Vector3d(x, y, z));
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
    qx_ = qx;
    qy_ = qy;
    qz_ = qz;
}

bool PoseCostFactor::Evaluate(double const* const* pose, 
                                double *residual, double **jacobians) const
{
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> poseA(pose[0]);
    Sophus::SE3d poseASE3 = Sophus::SE3d::exp(poseA);

    Eigen::Map<const Eigen::Matrix<double, 6, 1>> poseB(pose[1]);
    Sophus::SE3d poseBSE3 = Sophus::SE3d::exp(poseB);

    // cout << x_  << " " << y_ << " " << z_ << " " << w_ << " " << qx_ << " " << qy_ << " " << qz_ << endl;
    // Eigen::Quaterniond q(w_, qx_, qy_, qz_);
    // cout << q.toRotationMatrix() << endl;
    // cout << "Edge: " << "\n" << poseSE3_.matrix() << endl;
    // cout << "vertex 1: " << "\n" << poseASE3.matrix() << endl;
    // cout << "vertex 2: " << "\n" << poseBSE3.matrix() << endl;
    // cout << "vertex 1 to vertex 2: " << "\n" << poseASE3.matrix().inverse() * poseBSE3.matrix() << endl;
    // cout << "\n" << endl;

    Sophus::SE3d errorSE3 = poseSE3_.inverse() * poseASE3.inverse() 
                                                    * poseBSE3;
    // cout << "error: " << errorSE3.matrix() << endl;
    Eigen::Matrix<double, 6, 1> error = errorSE3.log();

    for(int i = 0; i < 6; i++)
        residual[i] = error(i);

    if(!jacobians || (!jacobians[0] && !jacobians[1]))
        return true;

    Eigen::MatrixXd jacobianRInv = Eigen::MatrixXd::Zero(6, 6);
    jacobianRInv.block(0, 0, 3, 3) = Sophus::SO3d::hat(errorSE3.so3().log());
    jacobianRInv.block(0, 3, 3, 3) = Sophus::SO3d::hat(errorSE3.translation());
    jacobianRInv.block(3, 3, 3, 3) = Sophus::SO3d::hat(errorSE3.so3().log());
    
    Eigen::MatrixXd jacobian[2];
    jacobian[0] = - (Eigen::MatrixXd::Identity(6, 6) + 0.5 * jacobianRInv) 
                                * poseBSE3.inverse().Adj();
    jacobian[1] = (Eigen::MatrixXd::Identity(6, 6) + 0.5 * jacobianRInv) 
                                * poseBSE3.inverse().Adj();
    for(int k = 0; k < 2; k++)
    {
        for(int i = 0; i < 6; i ++)
        {
            for (int j = 0; j < 6; j++)
                jacobians[k][i*6 + j] = jacobian[k](i, j);
        }
    }

    return true;
}