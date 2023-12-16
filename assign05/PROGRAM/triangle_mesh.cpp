#include "triangle_mesh.h"

TriangleMesh::TriangleMesh() : m_vertices_modes(nullptr), m_neighbor_index(std::vector<int>()),
                               m_vertex_index(std::vector<int>()), m_lambdas(nullptr)
{
}
TriangleMesh::TriangleMesh(std::vector<std::vector<Matrix>> *vertices_modes,
                           int neighbor_idx1, int neighbor_idx2, int neighbor_idx3,
                           int vertex_idx1, int vertex_idx2, int vertex_idx3,
                           std::vector<float> *lambdas)
{
    TriangleMesh();
    m_vertices_modes = vertices_modes;
    m_lambdas = lambdas;

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
bool TriangleMesh::inTriangle(Matrix m)
{
    Matrix result = get_bary(m);
    float lambda = result.get_pos(0, 0);
    float mu = result.get_pos(1, 0);
    float v = result.get_pos(2, 0);

    bool lambda_is_neg = lambda < 0;
    bool mu_is_neg = mu < 0;
    bool v_is_neg = v < 0;
    if (!lambda_is_neg && !mu_is_neg && !v_is_neg)
    {
        return true;
    }
    return false;
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
        Matrix closest = get_coord(0) * lambda + get_coord(1) * mu + get_coord(2) * v;
        // if (!inTriangle(closest))
        // {
        //     std::cout << "it is supposed to be in triangle with three positive but it doesn't" << std::endl;
        // }
        float dist = (closest - mat).magnitude();
        return std::make_tuple(dist, closest);
    }
    if (lambda_is_neg)
    {
        if (mu_is_neg)
        {
            // cross between vertex 1 and 2 --> vertex 3
            float dist = (get_coord(2) - mat).magnitude();
            return std::make_tuple(dist, get_coord(2));
        }
        if (v_is_neg)
        {
            // cross between vertex 1 and 3 --> vertex 2
            float dist = (get_coord(1) - mat).magnitude();
            return std::make_tuple(dist, get_coord(1));
        }
        // now this means that the shortest line is on the line between vertex 2 and 3
        auto result = get_project(mat, 1, 2);
        // if (!inTriangle(std::get<1>(result)))
        // {
        //     std::cout << "lambda being negative but project is being stupid" << std::endl;
        // }
        return result;
    }
    else if (mu_is_neg)
    {
        if (v_is_neg)
        {
            // cross between vertex 2 and 3 --> vertex 1
            float dist = (get_coord(0) - mat).magnitude();
            return std::make_tuple(dist, get_coord(0));
        }
        // now this means that the shortest line is on the line between vertex 1 and 3
        auto result = get_project(mat, 0, 2);
        // if (!inTriangle(std::get<1>(result)))
        // {
        //     std::cout << "mu being negative but project is being stupid" << std::endl;
        // }
        return result;
    }
    // now this means that the shortest line is on the line between vertex 1 and 2
    auto results = get_project(mat, 0, 1);
    // // if (!inTriangle(std::get<1>(results)))
    // // {
    // //     std::cout << "v being negative but project is being stupid" << std::endl;
    // // }
    return results;
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
        Matrix difference = get_coord(i) - get_coord(2);
        triangle_vertices(0, i) = difference.get_pos(0, 0);
        triangle_vertices(1, i) = difference.get_pos(1, 0);
        triangle_vertices(2, i) = difference.get_pos(2, 0);
    }
    auto svd_1 = triangle_vertices.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    // construct the left matrix and solve for lambda and mu
    Eigen::MatrixXf m_t = svd_1.solve((mat - get_coord(2)).get());

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
    Matrix B = get_coord(vertex_idx1);
    Matrix C = get_coord(vertex_idx2);
    // direction vector: d = (C - B) / ||C - B||
    Matrix d = (C - B) * (1.0 / (C - B).magnitude());
    // vector from A to B: v = A - B
    Matrix v = A - B;
    // distance between B and the projection of A on BC: t = v * d
    float t = (v.transpose() * d).get_pos(0, 0);
    Matrix P;
    if (t > (C - B).magnitude())
    {
        P = C;
    }
    else if (t < -1)
    {
        P = B;
    }
    else
    {
        P = B + (d * t);
    }

    // projection point P = B + t * d

    float distance = (P - A).magnitude();
    return std::make_tuple(distance, P);
}

float TriangleMesh::get_area(Matrix v1, Matrix v2, Matrix v3)
{
    Matrix AB = v2 - v1;
    Matrix AC = v3 - v1;
    Matrix cross_product = AB.cross(AC);
    float cross_magnitude = cross_product.magnitude();
    return cross_magnitude / 2.0;
}

Matrix TriangleMesh::get_coord(int idx) const
{
    int coord_idx = m_vertex_index.at(idx);
    // std::cout << "idx of interest: " << idx << " coord index: " << coord_idx << std::endl;
    Matrix m_coord = m_vertices_modes->at(0).at(coord_idx);
    // std::cout << "original coord" << std::endl;
    // m_coord.print_str();
    Matrix offset(3, 1);
    for (int i = 0; i < (int)m_lambdas->size(); ++i)
    {
        offset = offset + m_vertices_modes->at(i + 1).at(coord_idx) * m_lambdas->at(i);
    }
    // std::cout << "added offset coord" << std::endl;
    //(m_coord + offset).print_str();
    return m_coord + offset;
}

std::tuple<float, float, float> TriangleMesh::get_barycentric_coefficient(Matrix p)
{
    Matrix a = get_coord(0);
    Matrix b = get_coord(1);
    Matrix c = get_coord(2);

    float area_abc = get_area(a, b, c);
    float area_abp = get_area(a, b, p);
    float area_acp = get_area(a, c, p);
    float area_bcp = get_area(b, c, p);

    // equation: w * A + u * B + v * C
    float u = area_acp / area_abc;
    float v = area_abp / area_abc;
    float w = area_bcp / area_abc;

    if (u + v + w - 1 > 0.1)
    {
        std::cout << area_acp << " " << area_abp << " " << area_bcp << " " << area_abc << std::endl;
    }

    // std::cout << "bary coefficients" << std::endl;
    // std::cout << w << " " << u << " " << v << std::endl;
    //(a * w + b * u + c * v).print_str();
    // p.print_str();

    return std::make_tuple(w, u, v);
}

std::vector<Matrix> TriangleMesh::get_coords() const
{
    std::vector<Matrix> return_vec;
    return_vec.push_back(get_coord(0));
    return_vec.push_back(get_coord(1));
    return_vec.push_back(get_coord(2));
    return return_vec;
}
