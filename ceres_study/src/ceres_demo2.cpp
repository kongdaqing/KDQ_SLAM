#include <iostream>
#include <chrono>
#include "ceres/ceres.h"
using namespace std;
using namespace ceres;

//Analytic Derivatives
class Demo2Analytic : public SizedCostFunction<3,3> {
public:
    Demo2Analytic() {}
    virtual ~Demo2Analytic() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        const double b1 = parameters[0][0];
        const double b2 = parameters[0][1];
        const double b3 = parameters[0][2];


        residuals[0] = 10.0 - b1;
        residuals[1] = 1.0 - sin(b2);
        //KDQ_Note: x[2]*x[2] is wrong format,must be (x[2])*(x[2]).
        residuals[2] = 4.0 - b3*b3;

        if (!jacobians) return true;
        double* jacobian = jacobians[0];
        if (!jacobian) return true;

        jacobian[0] = -1;
        jacobian[1] = cos(b2);
        jacobian[2] = -2*b3;

        return true;
    }


};

// Automatic Derivatives
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        residual[1] = T(1.0) - sin(x[1]);
        //KDQ_Note: x[2]*x[2] is wrong format,must be (x[2])*(x[2]).
        residual[2] = T(4) - (x[2])*(x[2]);
        return true;
    }
};



int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value.
    double x[3] = {5.0,1,1};


    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
            new AutoDiffCostFunction<CostFunctor, 3, 3>(new CostFunctor);
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
    std::cout << "x : " << 5 << " " << 1 << " " << 0
              << " -> " << x[0] << " " << x[1] << " "<< x[2] << "\n";

    Demo2Analytic* analytic_cost_function = new Demo2Analytic();
    Problem problem1;
    double y[3] = {5.0,1,1};
    problem1.AddResidualBlock(analytic_cost_function, NULL, y);
    Solver::Options options1;
    options1.linear_solver_type = ceres::DENSE_QR;
    options1.minimizer_progress_to_stdout = true;
    Solver::Summary summary1;
    chrono::steady_clock::time_point t_begin1 = chrono::steady_clock::now();
    Solve(options1, &problem1, &summary1);
    chrono::steady_clock::time_point t_end1 = chrono::steady_clock::now();
    chrono::duration<double> time_used1 = chrono::duration_cast<chrono::duration<double>>(t_end1 - t_begin1);

    std::cout << summary1.BriefReport() << "\n";
    std::cout << "cost time: " << time_used1.count() << " seconds!" << std::endl;
    std::cout << "y : " << 5 << " " << 1 << " " << 0
              << " -> " << y[0] << " " << y[1] << " "<< y[2] << "\n";
    return 0;
}


