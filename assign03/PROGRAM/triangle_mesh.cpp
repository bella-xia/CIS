#include "triangle_mesh.h"
TriangleMesh::TriangleMesh() : m_coords(std::vector<Matrix>()),
                               m_neighbor_index(std::vector<int>()),
                               m_vertex_index(std::vector<int>())
{
}
TriangleMesh::TriangleMesh(Matrix mat1, Matrix mat2, Matrix mat3,
                           int neighbor_idx1, int neighbor_idx2, int neighbor_idx3,
                           int vertex_idx1, int vertex_idx2, int vertex_idx3)
{
    TriangleMesh();
    m_coords.push_back(mat1);
    m_coords.push_back(mat2);
    m_coords.push_back(mat3);

    m_neighbor_index.push_back(neighbor_idx1);
    m_neighbor_index.push_back(neighbor_idx2);
    m_neighbor_index.push_back(neighbor_idx3);

    m_vertex_index.push_back(vertex_idx1);
    m_vertex_index.push_back(vertex_idx2);
    m_vertex_index.push_back(vertex_idx3);
}

std::tuple<float, Matrix> TriangleMesh::find_closest_point_in_triangle(Matrix mat)
{
    // std::cout << "inside find closest point" << std::endl;
    Matrix result = get_bary(mat);
    // std::cout << "finsih bary" << std::endl;
    float lambda = result.get_pos(0, 0);
    float mu = result.get_pos(1, 0);
    float v = result.get_pos(2, 0);
    if (lambda >= 0 && mu >= 0 && v >= 0)
    {
        float dist = (result + mat * -1).magnitude();
        return std::make_tuple(dist, result);
    }
    bool lambda_is_neg = lambda < 0;
    bool mu_is_neg = mu < 0;
    bool v_is_neg = v < 0;

    // vertex 1's opposite side
    if (lambda_is_neg)
    {
        if (mu_is_neg)
        {
            // cross between vertex 1 and 2 --> vertex 3
            float dist = (m_coords.at(2) + mat * -1).magnitude();
            return std::make_tuple(dist, m_coords.at(2));
        }
        if (v_is_neg)
        {
            // cross between vertex 1 and 3 --> vertex 2
            float dist = (m_coords.at(1) + mat * -1).magnitude();
            return std::make_tuple(dist, m_coords.at(1));
        }
        // now this means that the shortest line is on the line between vertex 2 and 3
        return get_project(mat, 1, 2);
    }
    else if (mu_is_neg)
    {
        if (v_is_neg)
        {
            // cross between vertex 2 and 3 --> vertex 1
            float dist = (m_coords.at(0) + mat * -1).magnitude();
            return std::make_tuple(dist, m_coords.at(0));
        }
        // now this means that the shortest line is on the line between vertex 1 and 3
        // std::cout << "reached here" << std::endl;
        return get_project(mat, 0, 2);
    }
    // now this means that the shortest line is on the line between vertex 1 and 2
    return get_project(mat, 0, 1);
}

Matrix TriangleMesh::get_bary(Matrix mat)
{
    Eigen::MatrixXf triangle_vertices = Eigen::MatrixXf::Zero(3, 2);
    // std::cout << "in bary function" << std::endl;
    for (int i = 0; i < 2; ++i)
    {
        Matrix difference = m_coords.at(i) + m_coords.at(2) * -1;
        triangle_vertices(0, i) = difference.get_pos(0, 0);
        triangle_vertices(1, i) = difference.get_pos(1, 0);
        triangle_vertices(2, i) = difference.get_pos(2, 0);
    }
    // std::cout << "added values" << std::endl;
    auto svd_1 = triangle_vertices.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf m_t = svd_1.solve((mat + m_coords.at(2) * -1).get());
    // std::cout << "finish least square" << std::endl;
    // std::cout << "lambda: " << m_t(0, 0) << std::endl;
    // std::cout << "mu: " << m_t(1, 0) << std::endl;
    // std::cout << "v: " << 1 - m_t(0, 0) - m_t(1, 0) << std::endl;
    return Matrix(m_t(0, 0), m_t(1, 0), 1.0 - m_t(0, 0) - m_t(1, 0));
}

std::tuple<float, Matrix> TriangleMesh::get_project(Matrix &target_mat, int vertex_idx1, int vertex_idx2)
{
    // std::cout << "entered get project" << std::endl;
    Matrix example = Matrix(-23.977119, -12.596711, -48.630154);
    // (example * -1).print_str();
    // m_coords.at(0).print_str();
    // m_coords.at(1).print_str();
    // (m_coords.at(vertex_idx2) * -1).print_str();
    Matrix direction_vector = m_coords.at(vertex_idx1) + (m_coords.at(vertex_idx2) * -1);
    // std::cout << "direction gotten" << std::endl;
    direction_vector = direction_vector * (1.0 / direction_vector.magnitude());
    // std::cout << "direction normalized" << std::endl;
    Matrix path = target_mat + (m_coords.at(vertex_idx2) * -1);
    // std::cout << "path gotten" << std::endl;
    Matrix displace = target_mat.transpose() * path;
    assert(displace.get_col() == 1 && displace.get_row() == 1);
    Matrix projected = m_coords.at(vertex_idx2) + (target_mat * displace.get_pos(0, 0));
    // std::cout << "prepare to return" << std::endl;
    return std::make_tuple((projected + (target_mat * -1)).magnitude(), projected);
}