# include "reprojection.hpp"

Eigen::Matrix2Xd stack_columns(const std::vector<Eigen::Vector2d>& points) {
    Eigen::Matrix2Xd mat(2, static_cast<Eigen::Index>(points.size()));

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(points.size()); ++i) {
        mat.col(i) = points[static_cast<std::size_t>(i)];
    }

    return mat;
}

Eigen::Matrix3Xd stack_columns(const std::vector<Eigen::Vector3d>& points) {
    Eigen::Matrix3Xd mat(3, static_cast<Eigen::Index>(points.size()));

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(points.size()); ++i) {
        mat.col(i) = points[static_cast<std::size_t>(i)];
    }

    return mat;
}

double mean_reprojection_error(
    const std::vector<Eigen::Vector3d>& object_points,
    const std::vector<Eigen::Vector2d>& image_points,
    const Eigen::Matrix3d& K,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
) {
    if (object_points.empty()) {
        throw std::invalid_argument("Object points cannot be empty.");
    }

    if (object_points.size() != image_points.size()) {
        throw std::invalid_argument("Number of object points and image points must be the same.");
    }

    auto object_points_stacked = stack_columns(object_points);

    auto image_points_stacked = stack_columns(image_points);

    auto camera_points = (R * object_points_stacked).colwise() + t;

    auto projections = (K * camera_points).colwise().hnormalized();

    auto errors = (projections - image_points_stacked).colwise().norm();

    return errors.mean();
}
