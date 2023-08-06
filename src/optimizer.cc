#include <include/optimizer.h>

CeresOptimizer::CeresOptimizer(string poseA_dir , 
                                                                    string poseB_dir ,
                                                                    string edge_dir, 
                                                                    string poseA_opti_dir,
                                                                    string poseB_opti_dir)
{
    //These poses are stored in TUM format.
    poseA_file_.open(poseA_dir, ios::in);
    poseB_file_.open(poseB_dir, ios::in);
    edge_file_.open(edge_dir, ios::in);
    poseA_opti_file_.open(poseA_opti_dir, ios::out);
    poseB_opti_file_.open(poseB_opti_dir, ios::out);

    // options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLOSKY;
    options_.minimizer_progress_to_stdout = true;
}

void CeresOptimizer::AddResidual()
{
    //load vertices
    double timestamp, x, y, z, qx, qy, qz, w;
    while(!poseA_file_.eof())
    {
        poseA_file_ >> timestamp >> x >> y >> z >> qx >> qy >> qz >> w;
        timestampA_.push_back(timestamp);
        Eigen::Quaterniond q(w, qx, qy, qz);
        q.normalize();
        Eigen::Matrix<double, 6, 1> poseVec = Sophus::SE3<double>(q, 
                                                                Eigen::Vector3d(x, y, z)).log();
        double *opti_param = new double[6]{poseVec(0), poseVec(1), poseVec(2),
                                                                                    poseVec(3), poseVec(4), poseVec(5)};
        verticesA_.push_back(opti_param);
    }

    while(!poseB_file_.eof())
    {
        poseB_file_ >> timestamp >> x >> y >> z >>qx >> qy >> qz>> w;
        timestampB_.push_back(timestamp);
        Eigen::Quaterniond q(w, qx, qy, qz);
        q.normalize();
        Eigen::Matrix<double, 6, 1> poseVec = Sophus::SE3<double>(q, 
                                                                Eigen::Vector3d(x, y, z)).log();
        double *opti_param = new double[6]{poseVec(0), poseVec(1), poseVec(2),
                                                                                    poseVec(3), poseVec(4), poseVec(5)};
        verticesB_.push_back(opti_param);
    }

    //load intra-robot and inter-robots constraintions
    int type, id1, id2;
    ceres::LocalParameterization *local_parameterization = 
                                                        new PoseParameterizationSE3Sophus();
    while(!edge_file_.eof())
    {
        edge_file_ >> type >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> w;
        //type=0 ----> inter robot constraintions, id1 for A, id2 for B
        //type=1 ----> intra-robot constraintions for A
        //type=2 ----> intra-robot constraintions for B
        ceres::CostFunction *cost_function = new PoseCostFactor(
                                                                                                x, y, z, w, qx, qy, qz);
        if(type)
        {
            if(type == 1)
            {
                problem_.AddResidualBlock(cost_function,
                                                                        loss_function_,
                                                                        verticesA_[id1],
                                                                        verticesA_[id2]);
                problem_.SetParameterization(verticesA_[id1], 
                                                                                local_parameterization);
                problem_.SetParameterization(verticesA_[id2], 
                                                                                local_parameterization);
            }
            else
            {
                problem_.AddResidualBlock(cost_function,
                                                                        loss_function_,
                                                                        verticesB_[id1],
                                                                        verticesB_[id2]);
                problem_.SetParameterization(verticesB_[id1], 
                                                                                local_parameterization);
                problem_.SetParameterization(verticesB_[id2], 
                                                                                local_parameterization);
            }
        }
        else
        {
            problem_.AddResidualBlock(cost_function,
                                                                        loss_function_,
                                                                        verticesA_[id1],
                                                                        verticesB_[id2]);
            problem_.SetParameterization(verticesA_[id1], 
                                                                            local_parameterization);
            problem_.SetParameterization(verticesB_[id2], 
                                                                            local_parameterization);
        }
    }
}

void CeresOptimizer::SaveOptiResults(vector<double*> &vertices, 
                                                                                ofstream &opti_file,
                                                                                vector<double> &timestamp)
{
    auto seq = timestamp.begin();
    for(auto iter = vertices.begin(); iter != vertices.end(); iter++)
    {
        Eigen::Matrix<double, 6, 1> poseVec(**iter, *(*iter + 1), *(*iter + 2), 
                                                                            *(*iter + 3), *(*iter + 4), *(*iter + 5));
        Eigen::Matrix<double, 4, 4> pose_mat = (Sophus::SE3d::exp(poseVec)).matrix();
        Eigen::Matrix<double, 3, 3> rotation = pose_mat.block(0, 0, 3, 3);
        Eigen::Quaterniond q(rotation);
        opti_file << *seq << " " << pose_mat(0,3) << " "<< pose_mat(1,3) << 
        " " << pose_mat(2,3) << " "<< q.x() << " " << q.y() << " " << q.z() << " " 
        << q.w() << "\n";
        seq ++;
    }
}

void CeresOptimizer::SolveProblem()
{
    ceres::Solve(options_, &problem_, &summary_);
    cout << summary_.FullReport() << endl;
    cout << "\n\n\n" << "End of full report." << endl;
    SaveOptiResults(verticesA_, poseA_opti_file_, timestampA_);
    SaveOptiResults(verticesB_, poseB_opti_file_, timestampB_);
    cout << "\n\n\n" << "Optimization results have been saved." << endl;
}
