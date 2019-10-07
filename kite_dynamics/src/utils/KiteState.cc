#include "KiteState.h"

namespace kite_system_simulation
{

KiteState::KiteState() : 
q_(), q_dot_(), euler_angle_(), euler_rate_()
{
    helper_ = std::make_shared<MatrixHelper>();
}

KiteState::KiteState(const KiteState &other) : 
q_(other.q_), q_dot_(other.q_dot_), euler_angle_(other.euler_angle_), euler_rate_(other.euler_rate_)
{
    helper_ = std::make_shared<MatrixHelper>();
}

KiteState &KiteState::operator=(const KiteState &other)
{
    if (*this == other)
    {
    }
    else
    {
        this->q_ = other.q_;
        this->q_dot_ = other.q_dot_;
        this->euler_angle_ = other.euler_angle_;
        this->euler_rate_ = other.euler_rate_;
    }

    return *this;
}

KiteState::~KiteState()
{
}

bool KiteState::operator==(const KiteState &other)
{
    return (this->q_ == other.q_) && (this->q_dot_ == other.q_dot_) && (this->euler_angle_ == other.euler_angle_) 
    && (this->euler_rate_ == other.euler_rate_);
}

Eigen::Vector3d KiteState::cartesianPosition() const
{
    double r = q_[0];
    double q1 = q_[1];
    double q2 = q_[2];

    Eigen::Vector3d retVec;

    retVec << r * std::cos(q1) * std::sin(q2), r * std::sin(q1), r * std::cos(q1) * std::cos(q2);

    return retVec;
}

Eigen::Vector3d KiteState::cartesianVelocity() const
{
    return helper_->getP(q_) * q_dot_; 
}

Eigen::Vector3d KiteState::eulerAngle() const
{
    return euler_angle_;
}

void KiteState::eulerAngle(const Eigen::Vector3d val)
{
    euler_angle_ = val;
}

Eigen::Vector3d KiteState::eulerAngleRate() const
{
    return euler_rate_;
}

void KiteState::eulerAngleRate(const Eigen::Vector3d val)
{
    euler_rate_ = val;
}

Eigen::Vector3d KiteState::angularVelocity() const
{
    return helper_->getR(euler_angle_) * euler_rate_;
}

Eigen::Vector3d KiteState::sphericalPosition() const 
{
    return q_;
}

void KiteState::sphericalPosition(const Eigen::Vector3d val)
{
    q_ = val;
}

Eigen::Vector3d KiteState::sphericalVelocity() const
{
    return q_dot_;
}

void KiteState::sphericalVelocity(const Eigen::Vector3d val)
{
    q_dot_ = val;
}

Eigen::Matrix3d KiteState::OmegaCross() const
{
    return helper_->crossProductMatrix(angularVelocity());
}

Eigen::Matrix3d KiteState::P() const
{
    return helper_->getP(q_);
}

Eigen::Matrix3d KiteState::Pdot() const
{
    return helper_->getPdot(q_, q_dot_);
}

Eigen::Matrix3d KiteState::R() const
{
    return helper_->getR(euler_angle_);
}

Eigen::Matrix3d KiteState::Rdot() const
{
    return helper_->getRdot(euler_angle_, euler_rate_);
}

Eigen::Matrix3d KiteState::LBCdot() const
{
    return - OmegaCross() * helper_->getLBC(euler_angle_);
}

// Definition of the helper class

KiteState::MatrixHelper::MatrixHelper()
{

}

Eigen::Matrix3d KiteState::MatrixHelper::getP(const Eigen::Vector3d& spherical_pos)
{
    double r = spherical_pos[0];
    double q1 = spherical_pos[1];
    double q2 = spherical_pos[2];

    Eigen::Matrix3d retMat;

    retMat << std::cos(q1) * std::sin(q2), -r * std::sin(q1) * std::sin(q2), r * std::cos(q1) * std::cos(q2),
           std::sin(q1), r * std::cos(q1), 0.0,
           std::cos(q1) * std::cos(q2), -r * std::sin(q1) * std::cos(q2), -r * std::cos(q1) * std::sin(q2);

    return retMat;
} 

Eigen::Matrix3d KiteState::MatrixHelper::getR(const Eigen::Vector3d& angle)
{
    Eigen::Matrix3d ret;

    double phi = angle[0];
    double theta = angle[1];

    ret << 1.0, 0.0, -std::sin(theta),
        0.0, std::cos(phi), std::sin(phi) * std::cos(theta),
        0.0, -std::sin(phi), std::cos(phi) * std::cos(theta);
    return ret;
}

Eigen::Matrix3d KiteState::MatrixHelper::L1(const double val)
{
    Eigen::Matrix3d ret;

    ret << 1.0, 0.0, 0.0,
        0.0, std::cos(val), std::sin(val),
        0.0, -std::sin(val), std::cos(val);

    return ret;
}

Eigen::Matrix3d KiteState::MatrixHelper::L2(const double val)
{
    Eigen::Matrix3d retMat;

    retMat << std::cos(val), 0.0, -std::sin(val),
        0.0, 1.0, 0.0,
        std::sin(val), 0.0, std::cos(val);

    return retMat;
}

Eigen::Matrix3d KiteState::MatrixHelper::L3(const double val)
{
    Eigen::Matrix3d retMat;

    retMat << std::cos(val), std::sin(val), 0.0,
           -std::sin(val), std::cos(val), 0.0,
           0.0, 0.0, 1.0;

    return retMat;
}

Eigen::Matrix3d KiteState::MatrixHelper::getLCB(const Eigen::Vector3d &angle)
{
    return getLBC(angle).transpose();
}

Eigen::Matrix3d KiteState::MatrixHelper::getLBC(const Eigen::Vector3d &angle)
{
    double phi = angle[0];
    double theta = angle[1];
    double psi = angle[2];

    return L1(phi) * L2(theta) * L3(psi);
}

Eigen::Matrix3d KiteState::MatrixHelper::crossProductMatrix(const Eigen::Vector3d& omega)
{
    double omega_x = omega[0];
    double omega_y = omega[1];
    double omega_z = omega[2];

    Eigen::Matrix3d ret;

    ret << 0.0, -omega_z, omega_y,
        omega_z, 0.0, -omega_x,
        -omega_y, omega_x, 0.0;

    return ret;
}

} // namespace kite_system_simulation