
#include "apogee_vision/OrientationPredictor.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

std::vector<double> w_list;
std::vector<double> times;

Eigen::Vector3f calculateAngularVelocity(Eigen::Vector4f q_vec, float dt)
{
    static Eigen::Quaternionf prev_q;

    // q = current orientation
    Eigen::Quaternionf q = v4_to_quat(q_vec);

    // A = rotation matrix of current orientation
    Eigen::Matrix3f A(q);
    
    // Change in orientation
    Eigen::Quaternionf q_diff = q * prev_q.inverse();

    // dA = derivative of orientation in matrix form
    Eigen::Matrix3f dA(q_diff);

    dA = dA / dt;

    Eigen::Vector3f angular_velocity = dA.eulerAngles(0, 1, 2);


    //ROS_INFO("angular velocity");
    //print_eigen(angular_velocity);


    prev_q = q;
    return angular_velocity;
}

// y = Asin(B(x + C)) + D
struct OrientationResidual {
    OrientationResidual(double x, double y) : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const A,
                    const T* const B,
                    const T* const C,
                    const T* const D, T* residual) const {
        residual[0] = T(y_) - A[0]*sin(B[0] * T(x_) + C[0]) + D[0];
        //residual[0] = y_ - A[0] + B[0] + C[0] + D[0];
        return true;
    }

    private:
    // Observations for a sample
    const double x_;
    const double y_;
};

Eigen::Quaternionf predict_orientation(float dt, float t)
{
    ROS_INFO("predicting orientation");

    // Create fake data
    int num_points = 30;
    for (int i = 0; i < num_points; i++)
    {
        double t = (double)i/num_points;
        times.push_back(t);
        // y = Asin(B(x + C)) + D
        double a = 4.578;
        double b = 2;
        double c = 1;
        double d = 0;
        double y = a*sin(b * t + c) + d;
        w_list.push_back(y);
        ROS_INFO("t: %f, y: %f", t, y);
    }

    double A = 1;
    double B = 1;
    double C = 1;
    double D = 0;

    // Build problem
    Problem problem;
    ROS_INFO("w_list size %i", (int)w_list.size());
    for (int i = 0; i < (int)w_list.size(); ++i) {
        CostFunction* cost_function =
            new AutoDiffCostFunction<OrientationResidual, 1, 1, 1, 1, 1>(
                new OrientationResidual(times[i], w_list[i]));
        problem.AddResidualBlock(cost_function, nullptr, &A, &B, &C, &D);
    }

    // Run solver
    Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);


    ROS_INFO("summary: %s", summary.BriefReport().c_str());
    ROS_INFO("A: %f, B: %f, C: %f, D: %f", A, B, C, D);
}