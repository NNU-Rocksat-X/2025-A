#include <Eigen/Dense>
#include <cmath>
#include <ros/ros.h>

#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

template <uint8_t StateDim, uint8_t MeasureDim>
class KalmanFilter {
    public:
        typedef Eigen::Matrix<float, MeasureDim, MeasureDim> MeasureMatrix;
        typedef Eigen::Matrix<float, MeasureDim, 1> MeasureVector;
        typedef Eigen::Matrix<float, StateDim, StateDim> StateMatrix;
        typedef Eigen::Matrix<float, StateDim, 1> StateVector;
    private:
        // X in notation
        StateVector state_estimate; // (x, dx, ddx, y, dy, ddy, z, dz, ddz)
        
        StateMatrix state_transition; // F in notation

        StateMatrix process_noise; // Q in notation - Variance of the noise

        StateMatrix estimation_uncertainty; // P in notation

        Eigen::Matrix<float, MeasureDim, StateDim> observation_matrix; // H in notation
       
        MeasureMatrix measurement_uncertainty; // R in notation

        Eigen::Matrix<float, StateDim, MeasureDim> kalman_gain; // K in notation

        void set_transition_matrix(float dt, StateMatrix &transition_matrix);
        void measurement_update(const MeasureVector &measurement);
        void predict(void);
        
    public:

        KalmanFilter(StateMatrix transition_matrix, 
                    StateMatrix process_noise_matrix, 
                    StateMatrix estimation_uncertainty_matrix, 
                    Eigen::Matrix<float, MeasureDim, StateDim> observation_matrix_,
                    const StateVector &initial_state_guess, 
                    float initial_uncertainty_guess, 
                    float dt_, 
                    float acceleration_variance, 
                    float measurement_variance);

        ~KalmanFilter();

        void step(const MeasureVector &measurement);

        void predict_state(StateMatrix prediction_transition, StateVector &predicted_state);

        void get_state(StateVector &state);
};

// Fix linker issue for template class
#include "kalman_filter.cpp"

#endif /* _KALMANFILTER_H_ */