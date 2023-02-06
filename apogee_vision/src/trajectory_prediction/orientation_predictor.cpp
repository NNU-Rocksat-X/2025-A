
#include "apogee_vision/OrientationPredictor.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::CauchyLoss;

#define NUM_AXIS (3)

std::vector<Eigen::Vector3f> w_list;
std::vector<double> times;
double curr_time = 0;


///////////////////////////////////////////////////////////////////////////////////////
//                          CALCULATE ORIENTATION VELOCITY
///////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f calculateAngularVelocity(Eigen::Vector4f q_vec, double dt)
{
    static Eigen::Quaternionf prev_q;
    static Eigen::Vector3f prev_vel;

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


    prev_q = q;
    curr_time += dt;
    w_list.push_back(angular_velocity);
    times.push_back(curr_time);

    // Only keep the most recent otherwise it takes to long to predict
    if (w_list.size() > 5000)
    {
        w_list.erase(w_list.begin());
        times.erase(times.begin());
    }


    return angular_velocity;
}


///////////////////////////////////////////////////////////////////////////////////////
//                         PREDICT ORIENTATION
///////////////////////////////////////////////////////////////////////////////////////

// y = Asin(B(x + C)) + D
struct OrientationResidual {
    OrientationResidual(double y) : y_(y) {}

    template <typename T>
    bool operator()(const T* const a, T* residual) const {
        residual[0] = T(y_) - a;
        //residual[0] = y_ - A[0] + B[0] + C[0] + D[0];
        return true;
    }

    private:
    // Observations for a sample
    const double y_;
};

void characterizeW(Eigen::Vector3f *avg_vel)
{
    for (int axis = 0; axis < NUM_AXIS; axis++)
    {
        // Build problem
        Problem problem;

        // Add residual block for each angular velocity point
        for (int i = 0; i < (int)w_list.size(); ++i) {
            CostFunction* cost_function =
                new AutoDiffCostFunction<OrientationResidual, 1, 1>(
                    new OrientationResidual(w_list[i][axis]));
            problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), &avg_vel[axis]);
        }

        // Run solver
        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        //options.minimizer_progress_to_stdout = true;
        Solver::Summary summary;
        Solve(options, &problem, &summary);

        ROS_INFO("summary: %s", summary.BriefReport().c_str());
    }

}

/*
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
        Eigen::Vector3f w(y, y, y);
        w_list.push_back(w);
        ROS_INFO("t: %f, y: %f", t, y);
    }
*/

Eigen::Quaternionf predict_orientation(float dt, float t)
{
    Eigen::Vector3f avg_vel;

    characterizeW(&avg_vel);

    ROS_INFO("curr ang vel:");
    print_eigen(w_list.back());
    
    ROS_INFO("avg vel");
    print_eigen(avg_vel);
}