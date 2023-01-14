#ifndef _VELOCITY_FILTER_H_
#define _VELOCITY_FILTER_H_


#include <Eigen/Dense>
#include <cmath>
#include <ros/ros.h>
#include <apogee_vision/KalmanFilter.h>

namespace Position
{
    typedef KalmanFilter<9,3>::MeasureMatrix MeasureMatrix;
    typedef KalmanFilter<9,3>::MeasureVector MeasureVector;
    typedef KalmanFilter<9,3>::StateMatrix StateMatrix;
    typedef KalmanFilter<9,3>::StateVector StateVector;



    typedef struct state {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Vector3f acceleration;
    } State;

}

class VelocityEstimator {
    public:
    private:
        KalmanFilter<9, 3>* kalman_filter;

        void set_transition_matrix(float dt, Position::StateMatrix &transition_matrix);
        Position::State vector_to_state(Position::StateVector vector);
    public:
        /* @Brief - Performs initialization of kalman filter with the initial state guess
        *
        * @param[in] state_estimate - Initial state guess (x, y, z)
        *
        * @param[in] initial_uncertainty_guess - guess at the variance of the estimation
        */
        VelocityEstimator(const Position::StateVector &initial_state_guess, float initial_uncertainty_guess, float dt_, float acceleration_variance, float measurement_variance);

        ~VelocityEstimator();

        /* @Brief - Performs one kalman model step
        *
        * @param[in] measurement - (x, y, z) position measurement
        *
        * @return N/A
        */
        void step(const Position::MeasureVector &measurement);

        /* @Brief - Predicts position and velocity at specified time
        *
        * @param[in] secs_until_prediction - number of seconds from now to predict the state
        *
        * @param[out] predicted_state - position and velocity at selected time
        *                               (x, y, z, dx, dy, dz)
        * @return N/A
        */
        void predict_state(float secs_until_prediction, Position::State &predicted_state);

        void get_state(Position::State &state);
        void get_state(Position::StateVector &state);

        void print_state(Position::StateVector &state);
};

#endif /*_VELOCITY_FILTER_H_*/