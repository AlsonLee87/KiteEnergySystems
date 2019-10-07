#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

namespace kite_system_simulation
{

class KiteState
{
public:
    KiteState();

    KiteState(const KiteState &other);

    KiteState &operator=(const KiteState &other);

    ~KiteState();

    bool operator==(const KiteState &other);

    Eigen::Vector3d cartesianPosition() const; // The Cartesian kite position

    Eigen::Vector3d eulerAngle() const; // The euler angle of the kite orientation

    void eulerAngle(const Eigen::Vector3d val);

    Eigen::Vector3d cartesianVelocity() const; // The Cartesian kite velocity

    Eigen::Vector3d eulerAngleRate() const; // The euler angle rate of the kite

    void eulerAngleRate(const Eigen::Vector3d val);

    Eigen::Vector3d angularVelocity() const; // The angular velocity of kite in body frame

    Eigen::Vector3d sphericalPosition() const; // The spherical coordinate of the kite

    void sphericalPosition(const Eigen::Vector3d val);

    Eigen::Vector3d sphericalVelocity() const; // The spherical rate the kite

    void sphericalVelocity(const Eigen::Vector3d val);

    Eigen::Matrix3d R() const;

    Eigen::Matrix3d Rdot() const; // TODO: need implementation from matlab

    Eigen::Matrix3d P() const;

    Eigen::Matrix3d Pdot() const; // TODO: need implementation form matlab

    Eigen::Matrix3d OmegaCross() const;

    Eigen::Matrix3d LBCdot() const;

    Eigen::Matrix3d LCBdot() const;

protected:
    Eigen::Vector3d q_; // Spherical Representation of the kite position

    Eigen::Vector3d q_dot_; // Spherical Representation of the kite velocity

    Eigen::Vector3d euler_angle_; // Euler angle representation of the kite orientation

    Eigen::Vector3d euler_rate_; // Rate of Euler angle

    class MatrixHelper
    {
    public:
        MatrixHelper();

        Eigen::Matrix3d getR(const Eigen::Vector3d &angle); // get the rotational velocity transformation matrix

        Eigen::Matrix3d getLCB(const Eigen::Vector3d &angle); // get the inverse rotation matrix

        Eigen::Matrix3d getLBC(const Eigen::Vector3d &angle); // get the rotational matrix

        Eigen::Matrix3d getP(const Eigen::Vector3d &spherical_state); // get the velocity transformation matrix

        Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d &omega); // get the cross product matrix

        Eigen::Matrix3d getPdot(const Eigen::Vector3d &q, const Eigen::Vector3d &q_dot); // get the translational velocity transformation matrix derivative

        Eigen::Matrix3d getRdot(const Eigen::Vector3d &euler_angle, const Eigen::Vector3d &euler_rate); // get the time derivative of the rotational transformation matrix
    protected:
        Eigen::Matrix3d L1(const double val);

        Eigen::Matrix3d L2(const double val);

        Eigen::Matrix3d L3(const double val);
    };

    std::shared_ptr<MatrixHelper> helper_;
};

} // namespace kite_system_simulation