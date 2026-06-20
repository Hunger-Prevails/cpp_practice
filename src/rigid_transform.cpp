# include "rigid_transform.hpp"

Eigen::Vector3d SE3::transformPoint(const Eigen::Vector3d& p) const {
    return R * p + t;
}

SE3 SE3::inverse() const {
    SE3 inv;
    inv.R = R.transpose();
    inv.t = -inv.R * t;
    return inv;
}

SE3 SE3::operator*(const SE3& other) const {
    SE3 result;

    result.R = this->R * other.R;
    result.t = this->R * other.t + this->t;

    return result;
}
