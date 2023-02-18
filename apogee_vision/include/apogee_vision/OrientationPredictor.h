#ifndef _ORIENTATION_PREDICTOR_H_
#define _ORIENTATION_PREDICTOR_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "apogee_vision/EigenUtil.h"

#define NUM_AXIS (3)

class OrientationPredictor 
{
    private:
        Eigen::Quaternionf prev_q;
        Eigen::Quaternionf curr_q;
        std::vector<Eigen::Vector3f> w_list;

        float step_period;

        // @Brief Basically averages list of angular velocities (w_list)
        //        Gets rid of outliers with linear least squares algorithm
        void characterizeW(Eigen::Vector3f &avg_vel);

    public:
        OrientationPredictor(float _step_period) : step_period(_step_period) {}
        ~OrientationPredictor(void) {}

        /* @Brief - Predict orientation in 't' seconds from now.
        *
        * @param[in] t - The time to predict the orientation
        *
        * Returns the orientation as a quaternion.
        */
        Eigen::Quaternionf predict(float t);

        /* @Brief - Calculates angular velocity and 
        *           updates the previous orientation and velocity
        *
        * @param[in] q_vec - A quaternion of the current orientation in a 4d vector form
        *
        * Returns N/A
        */
        void step(Eigen::Vector4f q_vec);


        // @Brief - Places current orientation and angular velocity in reference parameters.
        void get_state(Eigen::Quaternionf &orientation, Eigen::Vector3f &velocity);

};


#endif /* _ORIENTATION_PREDICTOR_H_ */