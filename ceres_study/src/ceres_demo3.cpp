/******* NOTE *******
Purpose: ceres solver study
Project: demo3 <analytic derivation>
Goal:    learn to construct cost-function by numeric derivation.
Author:  KDQ
Date:    2019/6/7
*********************/

#include <iostream>
#include <chrono>
#include "ceres/ceres.h"
using namespace std;
using namespace ceres;

//Analytic Derivatives
class AnalyticDiffCostFunctor :public SizedCostFunction<2,2>{
    virtual ~AnalyticDiffCostFunctor(){};
   // template <typename T>

    virtual bool Evaluate(double const* const* parameter, double* residual,double** jacobians) const {
        double x1 = parameter[0][0];
        double x2 = parameter[0][1];
        residual[0] = 1.0 - sin(x1);
        residual[1] = 4 - x2*x2;
        if(jacobians!=NULL&&jacobians[0]!=NULL)
        {
            jacobians[0][0] = -cos(x1);
            jacobians[0][1] = 0;
            jacobians[1][0] = 0;
            jacobians[1][1] = -2*x2;
        }
         return true;
    }
};



int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value.
    double x[2] = {0.5,1.5};


    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function = new AnalyticDiffCostFunctor;
    problem.AddResidualBlock(cost_function, NULL, x);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;

    chrono::steady_clock::time_point t_begin = chrono::steady_clock::now();
    Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t_end = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_end - t_begin);


    std::cout << summary.BriefReport() << "\n";
    std::cout << "cost time: " << time_used.count() << " seconds!" << std::endl;
    std::cout << "x : " << x[0] << " " << x[1] << endl ;

    return 0;
}


