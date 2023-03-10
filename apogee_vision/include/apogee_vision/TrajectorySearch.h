#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>

#define SEARCH_TIMEOUT (1000) 


typedef Eigen::Matrix<float, 3, 1> Vector3f;



Vector3f mult(Vector3f vect1, Vector3f vect2) {
    return Vector3f(vect1[0] * vect2[0], vect1[1] * vect2[1], vect1[2] * vect2[2]);
}

float eigen_dist(Vector3f vectA, Vector3f vectB) {
    Vector3f diff = vectA - vectB;
    return sqrt(pow(diff[0], 2) + pow(diff[1], 2) + pow(diff[2], 2));
}


float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class Searcher {
    private:
        Vector3f x_0;
        Vector3f v_0;
        Vector3f a;
        Vector3f robot_base;
        // Defaults, real values are read from params
        float robot_length = 1;
        float epsilon = 0.001;
        float search_delta = 0.01;
        float min_time_delta = 0.01;
        float max_time_delta = 5;

        float t_0;

        float b;
        float d;

        bool is_linear = false;

        float linear() {
            return (-1 * b) / (2 * d);
        }

        Vector3f find_position(float t)
        {
            return x_0 + v_0*t + 0.5*a*t*t;
        }

        float distance(float t) {
            Vector3f obj_pos = find_position(t);
            ROS_DEBUG("position: [x: %f, y: %f, z: %f]", obj_pos[0], obj_pos[1], obj_pos[2]);
            float dist = eigen_dist(robot_base, obj_pos);
            ROS_DEBUG("dist: %f", dist);
            return dist;
        }

        float binarySearch(float before, float after) {

            float mid = before + (after - before) / 2;
            ROS_DEBUG("search [before: %f, mid: %f, after %f]", before, mid, after);
            float mid_dist = distance(mid);
            float after_dist = distance(after);
            float before_dist = distance(before);

            ROS_DEBUG("dist: [before: %f, mid: %f, after: %f]", before_dist, mid_dist, after_dist);
            // if not much change occurs local minimum has been reached
            if (abs(after_dist - before_dist) < epsilon) {
                return mid;
            }

            if (mid_dist < before_dist and mid_dist < after_dist) {
                if (before_dist < after_dist) {
                    return binarySearch(before, mid);
                }
                else {
                    return binarySearch(mid, after);
                }
            }

            // mid_dist is also less than before_dist
            if (mid_dist > after_dist) {
                return binarySearch(mid, after);
            }
            
            if (mid_dist > before_dist) {
                return binarySearch(before, mid);
            }

            throw;
        }

        float search_before(float closest)
        {
            float check_time = closest;
            while (distance(check_time) < robot_length && check_time > min_time_delta)
            {
                check_time -= search_delta;
            }
            return check_time;
        }

        float search_after(float closest)
        {
            float check_time = closest;
            while (distance(check_time) < robot_length && check_time < max_time_delta)
            {
                check_time += search_delta;
            }
            return check_time;
        }
    public:
        Searcher(Vector3f _x_0, Vector3f _v_0, Vector3f _a, float _t_0=200.0) {
            b = 2*_x_0.sum() * _v_0.sum();
            d = mult(_v_0, _v_0).sum();

            x_0 = _x_0;
            v_0 = _v_0;
            a = _a;
            t_0 = _t_0;

            ROS_INFO("x_0: [x: %f, y: %f, z: %f]", x_0[0], x_0[1], x_0[2]);
            ROS_INFO("v_0: [x: %f, y: %f, z: %f]", v_0[0], v_0[1], v_0[2]);
            ROS_INFO("a:   [x: %f, y: %f, z: %f]", a[0], a[1], a[2]);

            if (ros::param::has("TrajectoryPredictorConfig"))
            {
                ROS_INFO("Getting params");
                ros::param::get("arm_length", robot_length);
                std::vector<float> base = {0, 0, 0};
                ros::param::get("robot_base", base);
                robot_base[0] = base[0];
                robot_base[1] = base[1];
                robot_base[2] = base[2];
                ros::param::get("sufficient_convergence", epsilon);
                ros::param::get("search_delta", search_delta);
                ros::param::get("min_time_delta", min_time_delta);
                ros::param::get("max_time_delta", max_time_delta);
            } else {
                ROS_ERROR("Trajectory Predictor params not loaded.");
            }

            // Use different solving method if there is no acceleration
            if (_a.sum() == 0) {
                is_linear = true;
            }
        }

        // Returns a vector with the position of the object at the chosen trajectory slice
        // Updates norm_time with the duration until the selected point in the trajectory
        // A norm time of -1 indicates the object is out of reach
        Vector3f solve(float trajectory_slice, float &norm_time) {
            float closest_point;
            if (is_linear) {
                closest_point = linear();
            } else {
                closest_point = binarySearch(0, t_0);
            }
            if (closest_point > robot_length)
            {
                norm_time = -1;
                return Vector3f(0, 0, 0);
            }
            ROS_INFO("Closest point: %f", closest_point);
            float furthest_before = search_before(closest_point);
            ROS_INFO("Furthest before: %f", furthest_before);
            float furthest_after = search_after(closest_point);
            ROS_INFO("Furthest after: %f", furthest_after);

            norm_time = map(trajectory_slice, 0, 1, furthest_before, furthest_after);

            return find_position(norm_time);
        }
};