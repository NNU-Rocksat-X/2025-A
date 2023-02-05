#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Quaterniond;
using Eigen::Vector3d;

class SpinningObject {
public:
    Vector3d angularVelocity; // rad/s
    Vector3d momentOfInertia; // kg*m^2
    Quaterniond quaternion; // quaternion representation of orientation
    double time; // s

    SpinningObject(Vector3d w, Vector3d I, Quaterniond q) {
        angularVelocity = w;
        momentOfInertia = I;
        quaternion = q;
        time = 0;
    }

    void update(double dt) {
        // update quaternion based on angular velocity
        Quaterniond w_dt;
        w_dt.w() = 0;
        w_dt.vec() = angularVelocity * dt / 2;
        quaternion = quaternion * w_dt;
        quaternion.normalize();
        time += dt;
    }

    void predict(double dt) {
        // predict quaternion based on angular velocity
        Quaterniond w_dt;
        w_dt.w() = 0;
        w_dt.vec() = angularVelocity * dt / 2;
        quaternion = quaternion * w_dt;
        quaternion.normalize();
    }

    Vector3d calculateAngularVelocity(Quaterniond q_prev, double dt) {
        Quaterniond q_derivative = (quaternion - q_prev) / dt;
        Quaterniond q_conj = quaternion.conjugate();
        Vector3d angular_velocity = -2 * (q_conj.vec().transpose() * q_derivative.vec());
        return angular_velocity;
    }

    void print() {
        std::cout << "Orientation at t = " << time << " s: " << quaternion.w() << " + " << quaternion.vec().transpose() << std::endl;
    }
};

int main() {
    Vector3d w(0, 0, 0); // rad/s (initial angular velocity)
    Vector3d I(1, 2, 3); // kg*m^2 (moment of inertia)
    Quaterniond q(1, 0, 0, 0); // initial quaternion
    double dt = 0.1; // s (time step)

    SpinningObject obj(w, I, q);

    for (int i = 0; i < 10; i++) {
        obj.update(dt);
        obj.print();
    }
    Quaterniond q_prev = obj.quaternion;
    for (int i = 0; i < 10; i++) {
        obj.predict(dt);
        Vector3d angular_velocity = obj.calculateAngularVelocity(q_prev, dt);
        q_prev = obj.quaternion;
        std::cout << "Angular velocity at t = " << obj.time << " s: " << angular_velocity.transpose() << " rad/s" << std::endl;
        obj.print();
    }


    return 0;
}

