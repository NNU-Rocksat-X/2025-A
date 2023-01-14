#ifndef _ORIENTATION_FILTER_H_
#define _ORIENTATION_FILTER_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <assert.h>

#define STATEDIM (7)
#define DOF (6)
#define MEASUREDIM (3)

namespace Orientation
{
    typedef Eigen::Matrix<float, STATEDIM, STATEDIM> StateMatrix;
    typedef Eigen::Matrix<float, STATEDIM, 1> StateVector;
    typedef Eigen::Matrix<float, DOF, DOF> DOFMatrix;
    typedef Eigen::Matrix<float, DOF, 2*DOF> DOF2Matrix;
    typedef Eigen::Matrix<float, DOF, 1> DOFVector;
    typedef Eigen::Matrix<float, STATEDIM, 2*DOF> DistributionMatrix;
    typedef Eigen::Matrix<float, 4, 2*DOF> QDOFMatrix;
    typedef Eigen::Matrix<float, MEASUREDIM, 1> MeasureVector;
    typedef Eigen::Matrix<float, 4, 1> ObservationVector;
    typedef Eigen::Matrix<float, MEASUREDIM, MEASUREDIM> MeasureMatrix;
    typedef Eigen::Matrix<float, DOF, MEASUREDIM> DOFMeasureMatrix;
}


void print_eigen(const Eigen::Ref<const Eigen::MatrixXf>& mat);


class OrientationFilter {
    private:

        Orientation::StateVector state_estimate;
        Orientation::DOFMatrix state_covariance;

        Orientation::DOFMatrix noise_uncertainty;

        Eigen::Matrix<float, 4, 7> observation_matrix;

        Orientation::MeasureMatrix measurement_noise;

        Orientation::DOFMatrix covariance_mask;

        // Updated during prediction
        Orientation::StateVector priori_estimate;
        Orientation::DOFMatrix priori_state_covariance;
        Orientation::DOF2Matrix point_difference_matrix;
        Orientation::QDOFMatrix error_vectors;

        void predict(void);
        void measurement_update(Orientation::ObservationVector observation);
    public:
        OrientationFilter(Orientation::StateVector initial_state_guess, float initial_uncertainty_guess, float dt, float angular_velocity_variance, float measurement_variance);
        ~OrientationFilter();

        /* @Brief - Performs one kalman model step
        *
        * @param[in] observation - (x, y, z, w) quaternion orientation measurement
        *
        * @return N/A
        */
        void step(Orientation::ObservationVector observation);

        void get_state(Orientation::StateVector &state, Orientation::DOFMatrix &covariance);
};




Eigen::Vector3f calculateAngularVelocity(Eigen::Vector4f q_vec, float dt);
Eigen::Quaternionf predict_orientation(float dt, float t);
#endif /*_ORIENTATION_FILTER_H_*/