#include "registration.h"

/**
 * Calculate the midpoints of the matrix set a.
 * Store the midpoint calculated in mid_a.
 */
void Registration::calculate_midpoint_a()
{
    Eigen::MatrixXf sum_mat_a(3, 1);
    sum_mat_a << 0,
        0,
        0;
    for (int i = 0; i < size_a; ++i)
    {
        sum_mat_a += eigens_a[i].get();
    }
    mid_a = sum_mat_a / size_a;
    a_mid_calculated = true;
}

/**
 * Calculate the midpoints of the matrix set b.
 * Store the midpoint calculated in mid_b.
 */
void Registration::calculate_midpoint_b()
{
    Eigen::MatrixXf sum_mat_b(3, 1);
    sum_mat_b << 0,
        0,
        0;
    for (int j = 0; j < size_b; ++j)
    {
        sum_mat_b += eigens_b[j].get();
    }
    mid_b = sum_mat_b / size_b;
}

/**
 * Calculate a set of matrix a by a set of matrix b,
 * where a is at the frame which origins at the midpoint of b.
 */
void Registration::get_matrix_a_from_b()
{
    calculate_midpoint_b();
    Matrix neg_mid_b_mat = Matrix(mid_b) * -1;
    for (int i = 0; i < size_b; i++)
    {
        add_matrix_a(eigens_b[i] + neg_mid_b_mat);
    }
    mid_a << 0,
        0,
        0;
    a_mid_calculated = true;
}

/**
 * perform point cloud registration.
 * Find the frame transformation Fab such that Fab * b = a.
 */
Frame Registration::point_cloud_registration()
{
    // copy coordinates to  matrix in Eigen format
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> coord_a(3, size_a);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> coord_b(3, size_b);
    for (int i = 0; i < size_a; ++i)
        coord_a.col(i) = eigens_a[i].get().col(0);
    for (int j = 0; j < size_b; ++j)
        coord_b.col(j) = eigens_b[j].get().col(0);

    if (!a_mid_calculated)
    {
        calculate_midpoint_a();
    }
    calculate_midpoint_b();

    // subtract centroid
    coord_a.row(0).array() -= mid_a(0, 0);
    coord_a.row(1).array() -= mid_a(1, 0);
    coord_a.row(2).array() -= mid_a(2, 0);

    coord_b.row(0).array() -= mid_b(0, 0);
    coord_b.row(1).array() -= mid_b(1, 0);
    coord_b.row(2).array() -= mid_b(2, 0);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd_1 = (coord_a.transpose()).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf m_t = svd_1.solve(coord_b.transpose());
    Matrix m = Matrix(m_t.transpose());
    auto svd_2 = m.get().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    // we refered to http://www.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche0062.html to orthogonalize
    // result of SVD to r.
    Eigen::MatrixXf r = svd_2.matrixU() * svd_2.matrixV().transpose();
    Eigen::MatrixXf p = mid_b - r * mid_a;

    return Frame(Rotation(r), Position(p));
}

Matrix Registration::pivot_calibration(const std::vector<Frame> &f)
{
    int length = f.size();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> rot_mat(3 * length, 6);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pos_mat(3 * length, 1);
    pass_matrix(rot_mat, pos_mat, f);
    Matrix p_ts = Matrix(rot_mat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(pos_mat));
    return p_ts;
}

void Registration::pass_matrix(Eigen::MatrixXf &rot_mat, Eigen::MatrixXf &pos_mat, const std::vector<Frame> &f)
{
    int length = f.size();
    for (int i = 0; i < length; ++i)
    {
        Rotation rot = f[i].get_rot();
        Position pos = f[i].get_pos();
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                rot_mat(3 * i + j, k) = rot.get_rot().get_pos(j, k);
                rot_mat(3 * i + j, 3 + k) = (j == k) ? -1 : 0;
            }
            pos_mat(3 * i + j, 0) = pos.get_pos().get_pos(j, 0) * -1;
        }
    }
}