
#include "apogee_vision/AngularFilter.h"

using namespace Orientation;

Eigen::Vector4f quat_to_v4(const Eigen::Quaternionf q)
{
    Eigen::Vector4f v;
    v(0) = q.vec()(0);
    v(1) = q.vec()(1);
    v(2) = q.vec()(2);
    v(3) = q.w();
    return v;
}

void print_eigen(const Eigen::Ref<const Eigen::MatrixXf>& mat)
{
    for (int row = 0; row < mat.rows(); row++)
    {
        std::string row_str = "";
        for (int col = 0; col < mat.cols(); col++)
        {
            row_str += std::to_string(mat(row, col));
            row_str += " ";
        }
        ROS_INFO("%s", row_str.c_str());
    }
}

void print_eigen(const Eigen::Quaternionf q)
{
    Eigen::Matrix<float, 4, 1> vector_q;
    vector_q = quat_to_v4(q);
    print_eigen(vector_q);
}

Eigen::Quaternionf v4_to_quat(Eigen::Vector4f v)
{
    Eigen::Quaternionf quat(v(3), v(0), v(1), v(2));
    return quat;
}

// Quaternion to rotation vector (rotation vector is a part of angle axis method)
Eigen::Vector3f quat_to_v3(Eigen::Quaternionf q)
{
    Eigen::Vector3f v;

    v = q.vec();

    float theta = 2*acos(q.w());

    if (theta == 0)
    {
        v = v*theta;
    }
    else {
        v = v*theta / sin(theta/2);
    }
    
    return v;
}

// Rotation vector to quaternion (rotation vector is a part of angle axis method)
// Inverse of quat_to_v3
Eigen::Quaternionf v3_to_quat(Eigen::Vector3f v)
{
    float angle = v.norm(); // The norm of the orientation vector

    if (angle == 0)
    {
        Eigen::Quaternionf quaternion = Eigen::Quaternionf::Identity();
        return quaternion;
    }
    else {
        Eigen::Vector3f axis = v / angle;
        Eigen::Quaternionf quaternion(
                            cos(angle/2),
                            axis(0) * sin(angle/2),
                            axis(1) * sin(angle/2),
                            axis(2) * sin(angle/2)
        );
        return quaternion;
    }
}



//------------------------------- SIGMA POINTS ---------------------------------------------

/* @Brief - Performs the Chollesky-Banachiewicz algorithm
*           (Factorization on a matrix)
*
* @param[in] A - The input matrix to perform factorization on (Square matrix)
*
* @param[in] n - The dimension of the square input matrix
*
* returns L - one of the factors (Transpose it to get the other)
*/
DOFMatrix cholesky(DOFMatrix A, int n)
{
    DOFMatrix L = DOFMatrix::Zero();
    for ( int i = 0; i < n; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            float sum = 0;

            if (i == j)
            {
                for (int k = 0; k < j; k++)
                {
                    sum += pow(L(j,k), 2);
                }
                L(i, j) = sqrt(A(i, i) - sum);
                assert(!isnan(L(i,j)));
            } else {
                for (int k = 0; k < j; k++)
                {
                    sum += L(i, k) * L(j, k);
                }
                L(i, j) = (A(i, j) - sum) / L(j, j);
            }
        }
    }
    return L;
}

/* @Brief - Generates a set of values with a mean of 0 and covariance as the previous estimation uncertainty
*
* @param[in] L - One of the factors of the previous estimation uncertainty
*
* returns Wi - The output matrix with twice as many columns as the input.
*              Each column represents a sigma point distributed around the mean value zero
*/
DOF2Matrix generate_sigma_distribution(DOFMatrix L)
{
    DOF2Matrix Wi;
    Wi.block<DOF, DOF>(0, 0) = L * sqrt(2 * DOF);
    Wi.block<DOF, DOF>(0, DOF) = L * -1 * sqrt(2 * DOF);
    return Wi;
}

/* @Brief - Calculates sigma points which are a set of points that follow the normal distribution
*           described with the state_estimate as the mean and the covariance parameters
*
* @param[in] state_estimate - The current estimate of the state which is used as the mean of the distribution
*
* @param[in] covariance - The covariance of the state estimate distribution
*
* Returns a matrix with 2*DOF columns where each column is a sigma point vector.
*/
DistributionMatrix calculate_sigma_points(StateVector state_estimate, DOFMatrix covariance)
{
    DistributionMatrix sigma_points;

    // Perform cholesky decomposition to get factor of covariance
    DOFMatrix L = cholesky(covariance, DOF);

    // Wi is a set of 2*DOF vectors distributed around the mean value of 0 with the covariance of the covariance parameter
    DOF2Matrix Wi = generate_sigma_distribution(L);

    // Calculate quaternion sigma points by shifting the mean by the state estimate 
    Eigen::Quaternionf state_quaternion(state_estimate(3), state_estimate(0), state_estimate(1), state_estimate(2));

    // Move sigma orientation distribution to be centered around the orientation of the state estimate
    for (int i = 0; i < 2*DOF; i++)
    {
        // Create quaternion from the rotation vector of Wi
        Eigen::Quaternionf qW = v3_to_quat(Wi.col(i).head(3));

        // Move Wi point to a sigma point 
        Eigen::Quaternionf sigma_point_q = state_quaternion * qW;
        
        sigma_points(3, i) = sigma_point_q.w();
        sigma_points.block<3, 1>(0, i) = sigma_point_q.vec();
        
    }

    // Move angular velocity to be centered around the state estimate
    for (int i = 0; i < 2*DOF; i++)
    {
        Eigen::Vector3f sigma_point_angular = state_estimate.tail<3>() + Wi.col(i).tail<3>();
        sigma_points.block<3, 1>(4, i) = sigma_point_angular;
    }
    return sigma_points;
}

//------------------------------- PROJECT SIGMA POINTS  ---------------------------------------------

/* @Brief - Predicts the next state given the current state and the noise vector
*
* @param[in] current_state - The current state estimate denoted as x_n in the docs
*
* @param[in] noise_vector - A vector with dimension matching the degrees of freedom
*                           The vector follows the noise uncertainty matrix Q.
*                           (Orientation vector, angular velocity vector)
*
* @param[in] add_noise - Allows for process model calculation with sigma points that should not have 
*                        extra noise added to them
* Returns the predicted state
*/
StateVector process_model(StateVector current_state, DOFVector noise_vector, bool add_noise)
{
    Eigen::Quaternionf state_q = v4_to_quat(current_state.head(4));

    // Calculate noise quaternion
    Eigen::Quaternionf noise_q;
    if (add_noise)
    {
        noise_q = v3_to_quat(noise_vector.head(3));
    } else {
        noise_q.setIdentity();
    }

    // Calculate delta q (Change in rotation)
    Eigen::Quaternionf delta_q = v3_to_quat(current_state.tail<3>());


    Eigen::Quaternionf next_state_q;
    next_state_q = state_q * delta_q;


    StateVector next_state;
    next_state.head(4) = quat_to_v4(next_state_q);
    next_state.tail<3>() = current_state.tail<3>() + noise_vector.tail<3>();

    return next_state;
}

/* @Brief - Projects/Transforms sigma points one time step into the future. 
*           Utilizes the process model to project each point.
*
* @param[in] sigma_points - A matrix with 2*DOF columns where each column is a point in the estimate distribution
*                           Each point will be projected one time step
*
* Returns a matrix with 2*DOF columns where each column is a projected point.
*/
DistributionMatrix project_points(DistributionMatrix sigma_points)
{
    DOFVector additional_noise = DOFVector::Zero();

    DistributionMatrix projected_points;

    for (int i = 0; i < 2*DOF; i++)
    {
        projected_points.col(i) = process_model(sigma_points.col(i), additional_noise, false);
    }

    return projected_points;
}

//------------------------------- STATE ESTIMATE  ---------------------------------------------

/* @Brief - Computes the mean of the projected sigma points distribution. 
*           The mean for the angular velocity is calculated as the barycentric mean
*           The mean for the quaternion orientation is calculated with intrinsic gradient descent. (https://hal.inria.fr/inria-00073318/document
*
* @param[in] state_estimate - The mean estimation is seeded with the current state estimate
*
* @param[in] projected_points - The projected points distribution to calculate the mean of.
*
* @param[out] error_vectors - The 4d x 2*DOF final error vectors which are later used to calculate the distribution covariance
*
* Returns a state vector with the estimated mean of the projected sigma points
*/
StateVector calculate_mean_of_distribution(StateVector state_estimate, DistributionMatrix projected_points, QDOFMatrix &error_vectors)
{
    Eigen::Quaternionf mean_estimation = v4_to_quat(state_estimate.head(4));
    Eigen::Vector4f mean_error = Eigen::Vector4f::Constant(10); // Some value larger than epsilon
    int mean_iterations = 0;


    // Move to rosparam
    float epsilon = 0.001; 
    int max_iterations = 5;
    while (mean_iterations < 10 && mean_error.norm() > epsilon)
    {
        error_vectors = QDOFMatrix::Zero();

        // Calculate error vectors
        for (int i = 0; i < 2*DOF; i++)
        {
            Eigen::Quaternionf projected_q = v4_to_quat(projected_points.col(i).head(4));
            Eigen::Quaternionf error_quaternion = projected_q*mean_estimation.inverse();
            error_vectors.col(i) = quat_to_v4(error_quaternion);
        }

        Eigen::Vector4f error_sum = Eigen::Vector4f::Zero();
        for (int i = 0; i < 2*DOF; i++)
        {
            error_sum += error_vectors.col(i);
        }

        mean_error = error_sum / (2*DOF);

        Eigen::Quaternionf mean_error_q = v4_to_quat(mean_error);
        mean_error_q.normalize();

        mean_estimation = mean_error_q * mean_estimation;

        mean_iterations++;
    }

    // Calculate mean of projected angular velocity vectors
    Eigen::Vector3f angular_v_sum = Eigen::Vector3f::Zero();
    for (int i = 0; i < 2*DOF; i++)
    {
        angular_v_sum += projected_points.col(i).tail<3>();
    }
    Eigen::Vector3f mean_angular_v = angular_v_sum / (2*DOF);


    StateVector distribution_mean;
    distribution_mean.head(4) = quat_to_v4(mean_estimation);
    distribution_mean.tail<3>() = mean_angular_v;
    return distribution_mean;
}

//--------------------------- STATE VECTOR COVARIANCE  ---------------------------------------------

/* @Brief - Calculates the covariance of the projected state estimate by calculating the difference between each projected point.
*           The difference for the orientation of the projected points was already calculated with the error vectors.
*
* @param[in] priori_estimate - The estimate of the next state, the mean of the distribution that the covariance is being calculated for
*
* @param[in] projected_points - The projected points in the distribution which the covariance will be calculated from.
*
* @param[in] error_vectors - The 4d x 2*DOF final error vectors which were found while calculating the mean of the distribution
*
* @param[out] point_difference_matrix - A set of 2*DOF differences between the projected state estimate and projected sigma points
*                                       This is used later to calculate the cross correlation matrix.
*
* Returns the covariance matrix
*/
DOFMatrix calculate_state_vector_covariance(StateVector priori_estimate, DistributionMatrix projected_points, QDOFMatrix error_vectors, DOF2Matrix &point_difference_matrix)
{
    DOFMatrix covariance_sum = DOFMatrix::Zero();

    for (int i = 0; i < 2*DOF; i++)
    {
        DOFVector point_difference; // Wi' in paper

        // Calculate angular velocity difference
        point_difference.tail<3>() = projected_points.col(i).tail<3>() - priori_estimate.tail<3>();

        // Calculate quaternion orientation difference
        Eigen::Quaternionf q = v4_to_quat(error_vectors.col(i));
        q.normalize();
        point_difference.head(3) = quat_to_v3(q);
    
        
        // Calculate covariance for this point
        DOFMatrix point_covariance = point_difference * point_difference.transpose();
        covariance_sum += point_covariance;

        // Save point_difference for later
        point_difference_matrix.col(i) = point_difference;
    }

    DOFMatrix covariance = covariance_sum / (2*DOF);

    // Check for negatives. covariance must be positive semidefinite
    // Without this, covariance goes to infinity
    
    
    return covariance;
}

Eigen::Vector3f calculate_measurement(Eigen::Vector4f measurement)
{
    Eigen::Quaternionf quat_measurement = v4_to_quat(measurement);
    return quat_to_v3(quat_measurement);
}

//------------------------------- CROSS CORRELATION MATRIX ---------------------------------------------

/* @Brief - The cross correlation matrix relates the noise in the state vector to the noise in the measurement.
*           This is calculated by multiplying the difference between each projected sigma point and the average of the projection
*           by the transpose of the difference between each measurement point prediction and the average measurement prediction.
*
* @param[in] point_difference_matrix - Contains 2*DOF vectors which contain the difference between each projected sigma point and the average.
*
* @param[in] error_vectors - The 4d x 2*DOF final error vectors which were found while calculating the mean of the distribution.
*                            This is equal to the measurement difference prediction
*
* Returns the cross correlation matrix
*/
DOFMeasureMatrix calculate_cross_correlation(DOF2Matrix point_difference_matrix, QDOFMatrix error_vectors)
{
    DOFMeasureMatrix point_correlation_sum = DOFMeasureMatrix::Zero();

    for (int i = 0; i < 2*DOF; i++)
    {
        Eigen::Quaternionf q = v4_to_quat(error_vectors.col(i));
        q.normalize();
        Eigen::Vector3f measure_difference = quat_to_v3(q);


        DOFMeasureMatrix correlation = point_difference_matrix.col(i) * measure_difference.transpose();

        point_correlation_sum += correlation;
    }

    DOFMeasureMatrix cross_correlation_matrix = point_correlation_sum / (2*DOF);
    return cross_correlation_matrix;
}

//------------------------------- STATE UPDATE ---------------------------------------------

/* @Brief - Calculates the next state by adding the priori estimate to the kalman gain multiplied with the innovation.
*
* @param[in] priori_estimate - The predicted next state
*
* @param[in] kalman_gain - The matrix that is learned to determine how much the measured state vs predicted state should be used
*
* @param[in] innovation - Difference between the measurement and predicted measurement
*
* Returns the next state
*/
StateVector calculate_next_state(StateVector priori_estimate, DOFMeasureMatrix kalman_gain, MeasureVector innovation)
{
    // Note: kalman gain matrix will be 6x3.
    DOFVector kalman_update_vector = kalman_gain * innovation;

    ROS_INFO("kalman update vector");
    print_eigen(kalman_update_vector.transpose());

    // YoYou must multiply by innovation then use v3_to_quat on the first three dimensions before adding to the previous state.u must multiply by innovation then use v3_to_quat on the first three dimensions before adding to the previous state.
    Eigen::Quaternionf updated_measurement = v3_to_quat(kalman_update_vector.head(3));
    Eigen::Quaternionf priori_estimate_q = v4_to_quat(priori_estimate.head(4));


    // Calculate next state by adding updated measurement to priori estimate
    // Quaternions must be multiplied to add them
    StateVector next_state;
    next_state.head(4) = quat_to_v4(priori_estimate_q * updated_measurement);
    next_state.tail<3>() = priori_estimate.tail<3>() + kalman_update_vector.tail<3>();

    return next_state;
}

///////////////////////////////////////////////////////////////////////////////////////
//                         ORIENTATION FILTER PRIVATE
///////////////////////////////////////////////////////////////////////////////////////

void OrientationFilter::predict(void)
{
    // TODO: This may cause uncertainty to go to infinity
    DOFMatrix uncertainty = state_covariance;// + noise_uncertainty;

    DistributionMatrix sigma_points = calculate_sigma_points(state_estimate, uncertainty);
    DistributionMatrix projected_points = project_points(sigma_points);

    ROS_INFO("projected points");
    print_eigen(projected_points);

    // a priori estimate is computed as mean of transformed sigma points
    priori_estimate = calculate_mean_of_distribution(state_estimate, projected_points, error_vectors);

    ROS_INFO("error vectors");
    print_eigen(error_vectors);

    ROS_INFO("priori estimate");
    print_eigen(priori_estimate.transpose());

    priori_state_covariance = calculate_state_vector_covariance(priori_estimate, projected_points, error_vectors, point_difference_matrix);

    ROS_INFO("priori state covariance");
    print_eigen(priori_state_covariance);

    for (int i = 0; i < DOF; i++)
    {
        for (int j = 0; j < DOF; j++)
        {

            assert(priori_state_covariance(i,j)<1000);
        }
    }

}

void OrientationFilter::measurement_update(ObservationVector observation)
{
    ROS_INFO("observation");
    print_eigen(observation);
    // Calculate measurement estimate
    // The measurement estimate is the orientation from the predicted state
    MeasureVector predicted_measurement = calculate_measurement(observation_matrix * priori_estimate);

    ROS_INFO("predicted measurement");
    print_eigen(predicted_measurement);

    MeasureVector measurement = calculate_measurement(observation);

    ROS_INFO("measurement");
    print_eigen(measurement);

    // Innovation is actual measurement - predicted value
    MeasureVector innovation = measurement - predicted_measurement;
    ROS_INFO("innovation");
    print_eigen(innovation);

    // ------------ Calculate innovation covariance ----------------------

    // Predicted measurement uncertainty Pzz is orientation part of estimation uncertainty
    MeasureMatrix measurement_uncertainty = priori_state_covariance.block<3, 3>(0, 0);

    MeasureMatrix innovation_covariance = measurement_uncertainty + measurement_noise;

    ROS_INFO("innovation_covariance");
    print_eigen(innovation_covariance);

    // -------------- Calculate Kalman Gain ----------------

    DOFMeasureMatrix cross_correlation_matrix = calculate_cross_correlation(point_difference_matrix, error_vectors);

    ROS_INFO("cross correlation matrix");
    print_eigen(cross_correlation_matrix);

    // Calculate kalman gain by multiply cross correlation matrix with inverse of innovation covariance.
    DOFMeasureMatrix kalman_gain = cross_correlation_matrix * innovation_covariance.inverse();

    ROS_INFO("kalman gain");
    print_eigen(kalman_gain);

    // -------------- Calculate Next State and State Covariance ----------------

    // Then update the state estimate.
    state_estimate = calculate_next_state(priori_estimate, kalman_gain, innovation);

    ROS_INFO("state estimate");
    print_eigen(state_estimate.transpose());

    // Update state covariance
    state_covariance = priori_state_covariance - kalman_gain*innovation_covariance*kalman_gain.transpose();

    state_covariance = state_covariance.cwiseProduct(covariance_mask);

    for(int i = 0; i < DOF; i++)
    {
        for (int j = 0; j < DOF; j++)
        {
            if (state_covariance(i,j) < 0)
            {
                state_covariance(i,j) *= -1;
            }
        }
    }

    ROS_INFO("next state covariance");
    print_eigen(state_covariance);
}

///////////////////////////////////////////////////////////////////////////////////////
//                         ORIENTATION FILTER PUBLIC
///////////////////////////////////////////////////////////////////////////////////////

OrientationFilter::OrientationFilter(StateVector initial_state_guess, float initial_uncertainty_guess, float dt, float angular_velocity_variance, float measurement_variance)
{   
    float dt2 = pow(dt, 2);

    state_estimate = initial_state_guess;

    state_covariance << dt2, 0,   0,   0, 0, 0,
                        0,   dt2, 0,   0, 0, 0,
                        0,   0,   dt2, 0, 0, 0,
                        0,   0,   0,   1, 0, 0,
                        0,   0,   0,   0, 1, 0,
                        0,   0,   0,   0, 0, 1;

    state_covariance = state_covariance*initial_uncertainty_guess;

    noise_uncertainty << dt2, 0, 0, dt,  0, 0,
                         0,   dt2,  0,   0, dt, 0,
                         0,   0,    dt2, 0, 0,  dt,
                         dt,  0,    0,   1, 0,  0,
                         0,   dt,   0,   0, 1,  0,
                         0,   0,    dt,  0, 0,  1;


    noise_uncertainty = noise_uncertainty*angular_velocity_variance;

    observation_matrix << 1, 0, 0, 0, 0, 0, 0, 
                          0, 1, 0, 0, 0, 0, 0,
                          0, 0, 1, 0, 0, 0, 0,
                          0, 0, 0, 1, 0, 0, 0;

    covariance_mask <<   1, 0, 0, 1, 0, 0,
                         0, 1, 0, 0, 1, 0,
                         0, 0, 1, 0, 0, 1,
                         1, 0, 0, 1, 0, 0,
                         0, 1, 0, 0, 1, 0,
                         0, 0, 1, 0, 0, 1;
    

    measurement_noise << 1, 0, 0,
                         0, 1, 0,
                         0, 0, 1;

    measurement_noise = measurement_noise * measurement_variance;

    ROS_INFO("initial state");
    print_eigen(state_estimate);

    ROS_INFO("state_covariance");
    print_eigen(state_covariance);
}

void OrientationFilter::step(ObservationVector observation)
{
    predict();
    measurement_update(observation);
    for (int i = 0; i < STATEDIM; i++)
    {
        assert(!isnan(state_estimate(i)));
    }
    //assert(state_estimate(5) < 1);
    //assert(state_estimate(5) > -1);
}

void OrientationFilter::get_state(StateVector &state, DOFMatrix &covariance)
{
    state = state_estimate;
    covariance = state_covariance;
}

OrientationFilter::~OrientationFilter() {}

///////////////////////////////////////////////////////////////////////////////////////
//                                      MAIN
///////////////////////////////////////////////////////////////////////////////////////
/*
int main(int argc, char** argv)
{
    ros::init(argc, argv, "angular_filter");
    ros::NodeHandle nh;


    StateVector initial_state_guess;
    initial_state_guess << 0.707, 0, 0, 0.707, 0, 0, 0;

    float initial_uncertainty_guess = 0.1;
    float dt = 0.5;
    float angular_velocity_variance = 0.1;
    float measurement_variance = 0.1;


    OrientationFilter filter(initial_state_guess, initial_uncertainty_guess, dt, angular_velocity_variance, measurement_variance);

    // Get measurement
    ObservationVector observation;
    observation << 0.707, 0, 0, 0.707;

    filter.step(observation);

    StateVector out_state;

    filter.get_state(out_state);

    ROS_INFO("out");
    print_eigen(out_state);

}
*/

// Testing

Eigen::Quaternionf q_log(Eigen::Quaternionf q) {
    double norm = q.vec().norm();
    double w = q.w();
    double angle = atan2(norm, w);

    Eigen::Vector3f log_vec = q.vec() * angle / norm;

    return Eigen::Quaternionf(0, log_vec(0), log_vec(1), log_vec(2));
}

Eigen::Quaternionf mult(Eigen::Quaternionf q, float scalar)
{
    q.w() *= scalar;
    q.x() *= scalar;
    q.y() *= scalar;
    q.z() *= scalar;
    return q;
}

// Testing
static Eigen::Vector3f angular_velocity;
static Eigen::Quaternionf curr_q;
static Eigen::Quaternionf q_prev = Eigen::Quaternionf();



Eigen::Vector3f calculateAngularVelocity(Eigen::Vector4f q_vec, float dt) {
    Eigen::Quaternionf quaternion = v4_to_quat(q_vec);


    if (quaternion.norm() == 0)
    {
        angular_velocity = Eigen::Vector3f(0, 0, 0);
    } else {
        // w = dq(t)/dt * q(t)^-1
        Eigen::Quaternionf q_derivative = q_log(quaternion * q_prev.conjugate());
        angular_velocity = 2 * q_derivative.vec() / dt;

    }

    q_prev = quaternion;

    return angular_velocity;
}

struct CostFunctor {
  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    // Function to be fitted: y = a*sin(b*x + c) + d
    residuals[0] = parameters[0] * sin(parameters[1] * T(1.0) + parameters[2]) + parameters[3] - T(1.0);
    residuals[1] = parameters[0] * sin(parameters[1] * T(2.0) + parameters[2]) + parameters[3] - T(2.0);
    residuals[2] = parameters[0] * sin(parameters[1] * T(3.0) + parameters[2]) + parameters[3] - T(3.0);
    return true;
  }
};

// Fit function: y = a*sin(b*x + c) + d to the points
void characterizeW(double* parameters)
{

}


// q: initial orientation
Eigen::Quaternionf predict_orientation(float dt, float t) {
    float A = angular_velocity.norm();  // Amplitude
    float omega = A; // angular frequency
    float phi = atan2(angular_velocity(1), angular_velocity(0)); // phase shift

    ROS_INFO("A: %f, omega: %f, phi: %f", A, omega, phi);

    Eigen::Quaternionf q = curr_q;
    // Runge-Kutta 4th order method
    float k1, k2, k3, k4;
    for (float i = 0; i < t; i += dt) {
        k1 = A * sin(omega * i + phi) * dt;
        k2 = A * sin(omega * (i + dt / 2) + phi) * dt;
        k3 = A * sin(omega * (i + dt / 2) + phi) * dt;
        k4 = A * sin(omega * (i + dt) + phi) * dt;
        q = q * Eigen::Quaternionf(cos(k1/2), sin(k1/2) * k1/k1, sin(k1/2) * k2/k1, sin(k1/2) * k3/k1);
        print_eigen(q);
    }

    // Quaternionf rotation = Quaternionf(AngleAxisf(0.5f * w.norm() * dt, w.normalized()));
    // Quaternionf q_t = rotation * q0;

    ROS_INFO("Prediction:");
    print_eigen(q);
    return q;
}
