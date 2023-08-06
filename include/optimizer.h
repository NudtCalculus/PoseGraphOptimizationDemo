#include <iostream>
#include <fstream>
#include <vector>
#include <include/ceres_definition.h>

using namespace std;

class CeresOptimizer
{
    public:
        CeresOptimizer(string poseA_dir = "../data/poseA_noise.txt", 
                                        string poseB_dir = "../data/poseB_noise.txt",
                                        string edge_dir = "../data/edges.txt", 
                                        string poseA_opti_dir = "../data/poseA_opti.txt",
                                        string poseB_opti_dir = "../data/poseB_opti.txt");

        void AddResidual();

        void SolveProblem();

        void SaveOptiResults(vector<double*> &vertices, ofstream&file,
                                                    vector<double> &timestamp);

        ceres::Problem problem_;

        ceres::Solver::Summary summary_;

        ceres::Solver::Options options_;

        ceres::LossFunction* loss_function_ = nullptr;

        ifstream poseA_file_;

        ifstream poseB_file_;

        ifstream edge_file_;

        ofstream poseA_opti_file_;

        ofstream poseB_opti_file_;

        vector<double*> verticesA_;

        vector<double*> verticesB_;

        vector<double> timestampA_;

        vector<double> timestampB_;
}; 