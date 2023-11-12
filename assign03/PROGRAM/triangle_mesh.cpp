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
    
    Matrix result = get_bary(mat);
    float lambda = result.get_pos(0, 0);
    float mu = result.get_pos(1, 0);
    float v = result.get_pos(2, 0);
    if (lambda >= 0 && mu >= 0 && v >= 0)
    {
        Matrix closest = m_coords.at(0) * lambda + m_coords.at(1) * mu + m_coords.at(2) * v;
        float dist = (closest - mat).magnitude();
        return std::make_tuple(dist, closest);
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
            float dist = (m_coords.at(2) - mat).magnitude();
            return std::make_tuple(dist, m_coords.at(2));
        }
        if (v_is_neg)
        {
            // cross between vertex 1 and 3 --> vertex 2
            float dist = (m_coords.at(1) - mat).magnitude();
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
            float dist = (m_coords.at(0) - mat).magnitude();
            return std::make_tuple(dist, m_coords.at(0));
        }
        // now this means that the shortest line is on the line between vertex 1 and 3
        return get_project(mat, 0, 2);
    }
    // now this means that the shortest line is on the line between vertex 1 and 2
    return get_project(mat, 0, 1);
}

Matrix TriangleMesh::get_bary(Matrix mat)
{   
    /*
    Eigen::MatrixXf triangle_vertices = Eigen::MatrixXf::Zero(3, 3);
    Matrix q_p = m_coords.at(0) - m_coords.at(2);
    Matrix r_p = m_coords.at(1) - m_coords.at(2);
    Matrix p = m_coords.at(2);
    for (int i = 0; i < 3; i++) {
        triangle_vertices(i, 0) = q_p.get_pos(i, 0);
        triangle_vertices(i, 1) = r_p.get_pos(i, 0);
        triangle_vertices(i, 2) = p.get_pos(i, 0);
    }
    auto svd_1 = triangle_vertices.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf m_t = svd_1.solve((mat).get());
    return Matrix(m_t(0, 0), m_t(1, 0), 1.0 - m_t(0, 0) - m_t(1, 0));
    */
    
    Eigen::MatrixXf triangle_vertices = Eigen::MatrixXf::Zero(3, 2);
    for (int i = 0; i < 2; ++i)
    {
        Matrix difference = m_coords.at(i) - m_coords.at(2);
        triangle_vertices(0, i) = difference.get_pos(0, 0);
        triangle_vertices(1, i) = difference.get_pos(1, 0);
        triangle_vertices(2, i) = difference.get_pos(2, 0);
    }
    auto svd_1 = triangle_vertices.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf m_t = svd_1.solve((mat + m_coords.at(2) * -1).get());
    return Matrix(m_t(0, 0), m_t(1, 0), 1.0 - m_t(0, 0) - m_t(1, 0));
    
}

std::tuple<float, Matrix> TriangleMesh::get_project(Matrix &target_mat, int vertex_idx1, int vertex_idx2)
{   
    /* Reference: https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d*/
    // let A be the target mat, BC be the edge, P be the projected point of A on BC
    /**
     *      A
     *      |\v
     *      | \
     * C----P--B
     *      <-d-
     * 
     * ||P - B|| = t
     *       
    */
    
    Matrix A = target_mat;
    Matrix B = m_coords.at(vertex_idx1);
    Matrix C = m_coords.at(vertex_idx2);
    // direction vector: d = (C - B) / ||C - B||
    Matrix d = (C - B) * (1.0 / (C - B).magnitude());
    // vector from A to B: v = A - B
    Matrix v = A - B;
    // distance between B and the projection of A on BC: t = v * d
    float t = (v.transpose() * d).get_pos(0,0);
    // projection point P = B + t * d
    Matrix P = B + (d * t);

    float distance = (P - A).magnitude();
    return std::make_tuple(distance, P);
    /*
    Matrix direction_vector = m_coords.at(vertex_idx1) - m_coords.at(vertex_idx2);
    direction_vector = direction_vector * (1.0 / direction_vector.magnitude());
    Matrix path = target_mat - m_coords.at(vertex_idx2);
    Matrix displace = path.transpose() * direction_vector;
    assert(displace.get_col() == 1 && displace.get_row() == 1);
    Matrix projected = m_coords.at(vertex_idx2) + (direction_vector * displace.get_pos(0, 0));
    return std::make_tuple((projected - target_mat).magnitude(), projected);
    */
}