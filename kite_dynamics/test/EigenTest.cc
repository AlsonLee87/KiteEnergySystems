#include <Eigen/Dense>
#include <iostream>

int main(int argc, char** argv)
{
    Eigen::Quaternion<double> q(1.0, 1.0, 1.0, 1.0);

    std::cout << "The quaternion representaion is w: " << q.w() << " x: " << q.x() << " y: " << q.y() << " z: " << q.z() << std::endl;

    std::cout << "The corresponding rotational matrix is " << q.matrix() << std::endl;
}