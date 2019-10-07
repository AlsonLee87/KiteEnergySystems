#include "Quaternion.h"

namespace kite_system_simulation
{

Quaternion::Quaternion() : 
q0_(0.0), q1_(0.0), q2_(0.0), q3_(0.0), mat_rep_(createMatrixRep(0.0, 0.0, 0.0, 0.0)) {}

Quaternion::~Quaternion() {}

Quaternion::Quaternion(const double q0, const double q1, const double q2, const double q3) : 
q0_(q0), q1_(q1), q2_(q2), q3_(q3), mat_rep_(createMatrixRep(q0, q1, q2, q3)) {}

Quaternion::Quaternion(const double q1, const double q2, const double q3) : 
q0_(0.0), q1_(q1), q2_(q2), q3_(q3), mat_rep_(createMatrixRep(0.0, q1, q2, q3)) {}

Quaternion::Quaternion(const Quaternion &other) : 
q0_(other.q0_), q1_(other.q1_), q2_(other.q2_), q3_(other.q3_), mat_rep_(createMatrixRep(other.q0_, other.q1_, other.q2_, other.q3_))
{
}

Quaternion &Quaternion::operator=(const Quaternion &other)
{
    if (*this == other)
    {
    }
    else
    {
        this->q0_ = other.q0_;
        this->q1_ = other.q1_;
        this->q2_ = other.q2_;
        this->q3_ = other.q3_;
    }

    return *this;
}

bool Quaternion::operator==(const Quaternion &other)
{
    return (this->q0_ == other.q0_) && (this->q1_ == other.q1_) && (this->q2_ == other.q2_) && (this->q3_ == other.q3_);
}

Eigen::Matrix2cd Quaternion::conjugate() const
{
    return mat_rep_.conjugate();
}


Eigen::Matrix2cd Quaternion::createONE()
{
    Eigen::Matrix2cd one;
    one << 1.0, 0.0, 0.0, 1.0;

    return one;
}

Eigen::Matrix2cd Quaternion::createI()
{
    Eigen::Matrix2cd temp_I;
    temp_I << std::complex<double>(0, 1), std::complex<double>(0, 0), std::complex<double>(0, 0), std::complex<double>(0, -1);

    return temp_I;
}

Eigen::Matrix2cd Quaternion::createJ()
{
    Eigen::Matrix2cd temp_J;
    temp_J << 0.0, 1.0, -1.0, 0.0;

    return temp_J;
}

Eigen::Matrix2cd Quaternion::createK()
{
    Eigen::Matrix2cd temp_K;
    temp_K << std::complex<double>(0.0, 0.0), std::complex<double>(0.0, 1.0), std::complex<double>(0.0, 1.0), std::complex(0.0, 0.0);

    return temp_K;
}

Eigen::Matrix2cd Quaternion::createMatrixRep(const double q0, const double q1, const double q2, const double q3)
{
    mat_rep_ = q0 * one_ + q1 * i_ + q2 * j_ + q3 * k_;
}

Eigen::Matrix2cd Quaternion::one_(Quaternion::createONE());

Eigen::Matrix2cd Quaternion::i_(Quaternion::createI());

Eigen::Matrix2cd Quaternion::j_(Quaternion::createJ());

Eigen::Matrix2cd Quaternion::k_(Quaternion::createK());

} // namespace kite_system_simulation