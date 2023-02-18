#include "apogee_vision/PositionFilter.h"

using namespace Position;

/////////////////////////////////////////////////////////////
//                      CONSTRUCTORS
/////////////////////////////////////////////////////////////
PositionFilter::PositionFilter(const StateVector &state_guess, float initial_uncertainty_guess, float dt, float acceleration_variance, float measurement_variance)
{
    float dt2 = pow(dt, 2);
    float dt3 = pow(dt, 3);
    float dt4 = pow(dt, 4);

    // Initialize state transition matrix
    StateMatrix transition_matrix;
    set_transition_matrix(dt, transition_matrix);


    // Initialize process noise matrix
    StateMatrix process_noise_matrix;
    
    process_noise_matrix << dt4/4, dt3/2, dt2/2, 0,     0,     0,     0,     0,     0,
                            dt3/2, dt2,   dt,    0,     0,     0,     0,     0,     0,
                            dt2/2, dt,    1,     0,     0,     0,     0,     0,     0,
                            0,     0,     0,     dt4/4, dt3/2, dt2/2, 0,     0,     0,
                            0,     0,     0,     dt3/2, dt2,   dt,    0,     0,     0,
                            0,     0,     0,     dt2/2, dt,    1,     0,     0,     0,
                            0,     0,     0,     0,     0,     0,     dt4/4, dt3/2, dt2/2,
                            0,     0,     0,     0,     0,     0,     dt3/2, dt2,   dt,   
                            0,     0,     0,     0,     0,     0,     dt2/2, dt,    1;
    
    process_noise_matrix = process_noise_matrix * acceleration_variance;
                     
    // Initialize estimation uncertainty matrix
    StateMatrix uncertainty_matrix;
    uncertainty_matrix << 1, 1, 1, 0, 0, 0, 0, 0, 0,
                          1, 1, 1, 0, 0, 0, 0, 0, 0,
                          1, 1, 1, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 1, 1, 1, 0, 0, 0,
                          0, 0, 0, 1, 1, 1, 0, 0, 0,
                          0, 0, 0, 1, 1, 1, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 1, 1, 1,
                          0, 0, 0, 0, 0, 0, 1, 1, 1,
                          0, 0, 0, 0, 0, 0, 1, 1, 1;

    uncertainty_matrix = uncertainty_matrix * initial_uncertainty_guess;

    // Initialize observation matrix
    Eigen::Matrix<float, 3, 9> observation_matrix;
    observation_matrix << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 1, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 1, 0, 0;

    kalman_filter = new KalmanFilter<9, 3>(transition_matrix, process_noise_matrix,
                                           uncertainty_matrix, observation_matrix,
                                           state_guess, initial_uncertainty_guess, 
                                           dt, acceleration_variance, measurement_variance);
}

/////////////////////////////////////////////////////////////
//                      DESTRUCTOR
/////////////////////////////////////////////////////////////

PositionFilter::~PositionFilter() {}


/////////////////////////////////////////////////////////////
//                      PRIVATE FUNCTIONS
/////////////////////////////////////////////////////////////
void PositionFilter::set_transition_matrix(float dt, StateMatrix &transition_matrix)
{
    float dt2 = pow(dt, 2);

    // Initialize state transition matrix
    transition_matrix << 1, dt, dt2/2, 0, 0,  0,     0, 0,  0,
                         0, 1,  dt,    0, 0,  0,     0, 0,  0,
                         0, 0,  1,     0, 0,  0,     0, 0,  0,
                         0, 0,  0,     1, dt, dt2/2, 0, 0,  0,
                         0, 0,  0,     0, 1,  dt,    0, 0,  0,
                         0, 0,  0,     0, 0,  1,     0, 0,  0,
                         0, 0,  0,     0, 0,  0,     1, dt, dt2/2,
                         0, 0,  0,     0, 0,  0,     0, 1,  dt,   
                         0, 0,  0,     0, 0,  0,     0, 0,  1;  
}

State PositionFilter::vector_to_state(StateVector vector)
{
    State state;
    state.position(0) = vector(0);
    state.position(1) = vector(3);
    state.position(2) = vector(6);
    state.velocity(0) = vector(1);
    state.velocity(1) = vector(4);
    state.velocity(2) = vector(7);
    state.acceleration(0) = vector(2);
    state.acceleration(1) = vector(5);
    state.acceleration(2) = vector(8);
    return state;
}

/////////////////////////////////////////////////////////////
//                      PUBLIC FUNCTIONS
/////////////////////////////////////////////////////////////

void PositionFilter::step(const MeasureVector &measurement)
{
    kalman_filter->step(measurement);

}

void PositionFilter::predict_state(float secs_until_prediction, State &predicted_state)
{
    StateVector predicted_state_vector;
    StateMatrix prediction_transition;
    set_transition_matrix(secs_until_prediction, prediction_transition);

    kalman_filter->predict_state(prediction_transition, predicted_state_vector);
    predicted_state = vector_to_state(predicted_state_vector);
}

void PositionFilter::get_state(State &state)
{
    StateVector state_vector;
    kalman_filter->get_state(state_vector);
    state = vector_to_state(state_vector);
}

void PositionFilter::get_state(StateVector &state)
{
    kalman_filter->get_state(state);
}

void PositionFilter::print_state(StateVector &state)
{
    ROS_DEBUG("x: %f, dx: %f, ddx: %f, y: %f, dy: %f, ddy: %f, z: %f, dz: %f, ddz: %f", state(0), state(1), state(2), state(3), state(4), state(5), state(6), state(7), state(8));
}