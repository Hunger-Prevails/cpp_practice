# include <Eigen/Dense>
# include <Eigen/Core>

struct SE3 {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    Eigen::Vector3d transformPoint(const Eigen::Vector3d& p) const;
    SE3 inverse() const;
    SE3 operator*(const SE3& other) const;
};
