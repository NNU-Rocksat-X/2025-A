
#include "apogee_vision/OrientationPredictor.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::CauchyLoss;


///////////////////////////////////////////////////////////////////////////////////////
//                         PREDICT ORIENTATION
///////////////////////////////////////////////////////////////////////////////////////

struct OrientationResidual {
    OrientationResidual(double y) : y_(y) {}

    template <typename T>
    bool operator()(const T* const a, T* residual) const {
        residual[0] = T(y_) - a[0];
        //residual[0] = y_ - A[0] + B[0] + C[0] + D[0];
        return true;
    }

    private:
    // Observations for a sample
    const double y_;
};

// Basically calculates average of w_list, but gets rid of outliers
// Uses ceres library which uses doubles, so all floats are converted to doubles
// then back to floats.
void OrientationPredictor::characterizeW(Eigen::Vector3f &avg_vel)
{
    for (int axis = 0; axis < NUM_AXIS; axis++)
    {
        // Build problem
        Problem problem;

        double parameter;

        // Add residual block for each angular velocity point
        for (int i = 0; i < (int)w_list.size(); ++i) {
            CostFunction* cost_function =
                new AutoDiffCostFunction<OrientationResidual, 1, 1>(
                    new OrientationResidual((double)w_list[i][axis]));
            problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), &parameter);
        }

        // Run solver
        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        Solver::Summary summary;
        Solve(options, &problem, &summary);

        avg_vel[axis] = (float)parameter;
        // Should probably add convergence to telemetry
        //ROS_INFO("summary: %s", summary.BriefReport().c_str());
    }

}

Eigen::Quaternionf OrientationPredictor::predict(float t)
{
    Eigen::Vector3f w;
    characterizeW(w);
    Eigen::Vector3f last_w = w_list.back();


    Eigen::Vector3f scaled_w = w * t;

    Eigen::Quaternionf q_dot(0, w[0], w[1], w[2]);

    Eigen::Quaternionf predicted_q = curr_q * q_dot;
    predicted_q.normalize();

    return predicted_q;
    
}
///////////////////////////////////////////////////////////////////////////////////////
//                         CALCULATE ANGULAR VELOCITY
///////////////////////////////////////////////////////////////////////////////////////
void OrientationPredictor::step(Eigen::Vector4f q_vec)
{
    // q = current orientation
    curr_q = v4_to_quat(q_vec);
    
    // Change in orientation
    Eigen::Quaternionf q_diff = curr_q * prev_q.inverse();

    Eigen::Vector3f vector_diff = q_diff.toRotationMatrix().eulerAngles(0,1,2);
    Eigen::Vector3f angular_velocity = 2 *vector_diff / step_period;


    prev_q = curr_q;
    w_list.push_back(angular_velocity);

    // Only keep the most recent otherwise it takes to long to predict
    if (w_list.size() > 75)
    {
        w_list.erase(w_list.begin());
    }

}

///////////////////////////////////////////////////////////////////////////////////////
//                                  GET STATE
///////////////////////////////////////////////////////////////////////////////////////

void OrientationPredictor::get_state(Eigen::Quaternionf &orientation, Eigen::Vector3f &velocity)
{
    orientation = curr_q;
    velocity = w_list.back();
}