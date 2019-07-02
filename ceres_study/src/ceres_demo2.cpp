/******* NOTE *******
Purpose: ceres solver study
Project: demo2 <numeric derivation>
Goal:    learn to construct cost-function by numeric derivation.
Author:  KDQ
Date:    2019/6/7
*********************/

#include <iostream>
#include <chrono>
#include "ceres/ceres.h"
using namespace std;
using namespace ceres;

//Numeric Derivatives
struct NumericDiffCostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        residual[1] = T(2.0) - x[1];
         return true;
    }
};



int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value.
    double x[2] = {1,10};


    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
            new NumericDiffCostFunction<NumericDiffCostFunctor, CENTRAL,2,2>(new NumericDiffCostFunctor);
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


