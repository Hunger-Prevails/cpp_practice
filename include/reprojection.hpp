# include <vector>
# include <Eigen/Core>
# include <Eigen/Dense>

Eigen::Matrix2Xd stack_columns(const std::vector<Eigen::Vector2d>& points);
Eigen::Matrix3Xd stack_columns(const std::vector<Eigen::Vector3d>& points);

double mean_reprojection_error(
    const std::vector<Eigen::Vector3d>& object_points,
    const std::vector<Eigen::Vector2d>& image_points,
    const Eigen::Matrix3d& K,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
);
