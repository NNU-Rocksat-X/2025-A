#include "apogee_vision/KalmanFilter.h"

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

/////////////////////////////////////////////////////////////
//                      CONSTRUCTORS
/////////////////////////////////////////////////////////////
template <uint8_t StateDim, uint8_t MeasureDim>
KalmanFilter<StateDim, MeasureDim>::KalmanFilter(StateMatrix transition_matrix, StateMatrix process_noise_matrix, StateMatrix estimation_uncertainty_matrix, Eigen::Matrix<float, MeasureDim, StateDim> observation_matrix_,
            const StateVector &initial_state_guess, float initial_uncertainty_guess, float dt_, float acceleration_variance, float measurement_variance)
{
    // Set initial estimates
    state_estimate = initial_state_guess;
    state_transition = transition_matrix;
    process_noise = process_noise_matrix;
    estimation_uncertainty = estimation_uncertainty_matrix;
    observation_matrix = observation_matrix_;


    // Initialize measurement uncertainty matrix
    measurement_uncertainty = MeasureMatrix::Identity() * measurement_variance;
}

/////////////////////////////////////////////////////////////
//                      DESTRUCTOR
/////////////////////////////////////////////////////////////

template <uint8_t StateDim, uint8_t MeasureDim>
KalmanFilter<StateDim, MeasureDim>::~KalmanFilter() {}

/////////////////////////////////////////////////////////////
//                      PRIVATE FUNCTIONS
/////////////////////////////////////////////////////////////

/* @Brief - Updates the kalman gain, state estimate and estimation uncertainty based
*           on the latest measurement.
*
* @param[in] measurement - (x, y, z) position measurement
*                           aka Z in notation
*
* @return N/A
*/
template <uint8_t StateDim, uint8_t MeasureDim>
void KalmanFilter<StateDim, MeasureDim>::measurement_update(const MeasureVector &measurement)
{
    // Kalman gain update equation
    // K = P * H^T(HPH^T + R)^-1
    kalman_gain = estimation_uncertainty * observation_matrix.transpose() * 
            (observation_matrix * estimation_uncertainty * observation_matrix.transpose() +
            measurement_uncertainty).inverse();

    // State update equation
    // X = X + K(z - HX)
    state_estimate = state_estimate + kalman_gain *
            (measurement - observation_matrix*state_estimate); 

    // Covariance update equation
    // P = (I - KH) * P * (I - KH)^T + KRK^T
    StateMatrix I_minus_KH = StateMatrix::Identity() - kalman_gain * observation_matrix;

    estimation_uncertainty = I_minus_KH * estimation_uncertainty * I_minus_KH.transpose() +
            kalman_gain * measurement_uncertainty * kalman_gain.transpose();
}

/* @Brief - Extrapolates state estimate and estimation uncertainty
*
* @return N/A
*/
template <uint8_t StateDim, uint8_t MeasureDim>
void KalmanFilter<StateDim, MeasureDim>::predict(void)
{
    // State extrapolation equation
    // X = FX
    state_estimate = state_transition * state_estimate;

    // Covariance extrapolation equation
    // P = FPF^T + Q
    estimation_uncertainty = state_transition * estimation_uncertainty * state_transition.transpose() +
            process_noise;
}

/////////////////////////////////////////////////////////////
//                      PUBLIC FUNCTIONS
/////////////////////////////////////////////////////////////
template <uint8_t StateDim, uint8_t MeasureDim>
void KalmanFilter<StateDim, MeasureDim>::step(const MeasureVector &measurement)
{
    measurement_update(measurement);

    predict();

    /*
    ROS_INFO("state estimate");
    print_eigen(state_estimate);

    
    ROS_INFO("kalman gain");
    print_eigen(kalman_gain);

    ROS_INFO("estimation uncertainty");
    print_eigen(estimation_uncertainty);
    */
}

template <uint8_t StateDim, uint8_t MeasureDim>
void KalmanFilter<StateDim, MeasureDim>::predict_state(StateMatrix prediction_transition, StateVector &predicted_state)
{
    predicted_state = prediction_transition * state_estimate;
}

template <uint8_t StateDim, uint8_t MeasureDim>
void KalmanFilter<StateDim, MeasureDim>::get_state(StateVector &state)
{
    state = state_estimate;
}
