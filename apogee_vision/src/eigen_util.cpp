#include <apogee_vision/EigenUtil.h>

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

/*
Eigen::Vector3f quat_to_euler(Eigen::Quaternionf q)
{
    return q.toRotationMatrix().eulerAngles(0,1,2);
}
*/

Eigen::Vector3f quat_to_euler(Eigen::Vector4f vq)
{
    Eigen::Quaternionf q = v4_to_quat(vq);
    return quat_to_euler(q);
}

Eigen::Vector3f quat_to_euler(Eigen::Quaternionf q)
{
    Eigen::Vector3f euler;
    // roll
    double sinr_cosp = 2 * (q.w() * q.vec()[0] + q.vec()[1] * q.vec()[2]);
    double cosr_cosp = 1 - 2 * (q.vec()[0] * q.vec()[0] + q.vec()[1] * q.vec()[1]);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch
    double sinp = std::sqrt(1 + 2 * (q.w() * q.vec()[1] - q.vec()[0] * q.vec()[2]));
    double cosp = std::sqrt(1 - 2 * (q.w() * q.vec()[1] - q.vec()[0] * q.vec()[2]));
    euler[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw
    double siny_cosp = 2 * (q.w() * q.vec()[2] + q.vec()[0] * q.vec()[1]);
    double cosy_cosp = 1 - 2 * (q.vec()[1] * q.vec()[1] + q.vec()[2] * q.vec()[2]);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
    return euler;
}