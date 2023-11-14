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

TriangleMesh::~TriangleMesh()
{
}

std::tuple<float, Matrix> TriangleMesh::find_closest_point_in_triangle(Matrix mat)
{

    Matrix result = get_bary(mat);
    float lambda = result.get_pos(0, 0);
    float mu = result.get_pos(1, 0);
    float v = result.get_pos(2, 0);

    bool lambda_is_neg = lambda < 0;
    bool mu_is_neg = mu < 0;
    bool v_is_neg = v < 0;

    // if the point is inside the triangle.
    if (!lambda_is_neg && !mu_is_neg && !v_is_neg)
    {
        Matrix closest = m_coords.at(0) * lambda + m_coords.at(1) * mu + m_coords.at(2) * v;
        float dist = (closest - mat).magnitude();
        return std::make_tuple(dist, closest);
    }

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
    /** According to the Barycentric Form:
     * c = lambda * q + mu * r + v * p, where:
     * 1. lambda + mu + v = 1
     * 2. q, r, p are the three coordinates, c is the closest point
     *    if lambda >= 0, mu >= 0, v >= 0
     *
     * According to 1, v = 1 - lambda - mu.
     * Rearranging the equation, we get:
     * c = lambda * (q - p) + mu * (r - p) + p
     * c - p = lambda * (q - p) + mu * (r - p)
     *
     * Then, we can construct the least square problem:
     * | c_x - p_x |   |q_x - p_x, r_x - p_x|   | lambda |
     * | c_y - p_y | = |q_y - p_y, r_y - p_y| * |   mu   |
     * | c_z - p_z |   |q_z - p_z, r_z - p_z|
     *
     * to solve for lambda and mu.
     */
    Eigen::MatrixXf triangle_vertices = Eigen::MatrixXf::Zero(3, 2);

    // construct the middle matrix
    for (int i = 0; i < 2; ++i)
    {
        Matrix difference = m_coords.at(i) - m_coords.at(2);
        triangle_vertices(0, i) = difference.get_pos(0, 0);
        triangle_vertices(1, i) = difference.get_pos(1, 0);
        triangle_vertices(2, i) = difference.get_pos(2, 0);
    }
    auto svd_1 = triangle_vertices.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    // construct the left matrix and solve for lambda and mu
    Eigen::MatrixXf m_t = svd_1.solve((mat - m_coords.at(2)).get());

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
    float t = (v.transpose() * d).get_pos(0, 0);
    // projection point P = B + t * d
    Matrix P = B + (d * t);

    float distance = (P - A).magnitude();
    return std::make_tuple(distance, P);
}