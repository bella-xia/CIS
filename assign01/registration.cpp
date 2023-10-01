#include "registration.h"

void Registration::calculate_midpoints()
{
    Eigen::MatrixXf sum_mat_a(3, 1);
    Eigen::MatrixXf sum_mat_b(3, 1);
    sum_mat_a << 0,
        0,
        0;
    sum_mat_b << 0,
        0,
        0;
    for (int i = 0; i < size_a; ++i)
    {
        sum_mat_a += eigens_a[i];
    }
    mid_a = sum_mat_a / size_a;
    for (int j = 0; j < size_b; ++j)
    {
        sum_mat_b += eigens_b[j];
    }
    mid_b = sum_mat_b / size_b;
}

Eigen::MatrixXf Registration::best_plane_from_points()
{
    // copy coordinates to  matrix in Eigen format
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> coord_a(3, size_a);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> coord_b(3, size_b);
    for (size_t i = 0; i < size_a; ++i)
        coord_a.col(i) = eigens_a[i].col(0);
    for (size_t j = 0; j < size_b; ++j)
        coord_b.col(j) = eigens_b[j].col(0);

    calculate_midpoints();

    // calculate centroid
    // Eigen::MatrixXf centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord_a.row(0).array() -= mid_a(0, 0);
    coord_a.row(1).array() -= mid_a(1, 0);
    coord_a.row(2).array() -= mid_a(2, 0);

    coord_b.row(0).array() -= mid_b(0, 0);
    coord_b.row(1).array() -= mid_b(1, 0);
    coord_b.row(2).array() -= mid_b(2, 0);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = (coord_a.transpose()).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf m_t = svd.solve(coord_b.transpose());
    std::cout << coord_a.transpose() * m_t.transpose() << std::endl;
    std::cout << coord_b.transpose() << std::endl;

    return m_t;
}