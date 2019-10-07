#include <complex>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kite_system_simulation{


class Quaternion
{
public:
    Quaternion();

    ~Quaternion();

    Quaternion(const double q0, const double q1, const double q2, const double q3);

    Quaternion(const double q1, const double q2, const double q3);

    Quaternion(const Quaternion& other); 

    Quaternion& operator=(const Quaternion& other);

    bool operator==(const Quaternion& other);

    Eigen::Matrix2cd conjugate() const;

    Quaternion& operator*(const Quaternion& other);

    Quaternion& operator*=(const Quaternion& other);


protected:
    double q0_;

    double q1_;

    double q2_;

    double q3_;

    Eigen::Matrix2cd mat_rep_;

    Eigen::Matrix2cd createMatrixRep(const double q0, const double q1, const double q2, const double q3);

    static Eigen::Matrix2cd createONE();

    static Eigen::Matrix2cd one_;

    static Eigen::Matrix2cd createI();

    static Eigen::Matrix2cd i_;

    static Eigen::Matrix2cd createJ();

    static Eigen::Matrix2cd j_;

    static Eigen::Matrix2cd createK();

    static Eigen::Matrix2cd k_; 
};

}