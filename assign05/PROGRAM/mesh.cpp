#include "mesh.h"

// default constructor with empty data structure to input vertices and triangles
Mesh::Mesh() : m_vertices(std::vector<Matrix>()), m_triangles(std::vector<TriangleMesh>()),
               m_num_vertices(0), m_num_triangles(0)
{
    initiate_lambdas();
}

// constructor with passed-in mode values
Mesh::Mesh(int n_m, std::vector<std::vector<Matrix>> modes) : m_vertices(std::vector<Matrix>()), m_modes(modes), m_triangles(std::vector<TriangleMesh>()),
                                                              m_num_vertices(0), m_num_triangles(0)
{
    initiate_lambdas(n_m);
}

Mesh::~Mesh()
{
}

// initiate the data structure for lambda with given mode number with all lambda = 0
void Mesh::initiate_lambdas(int n_m)
{
    m_lambdas = std::vector<float>();
    for (int i = 0; i < n_m; ++i)
    {
        m_lambdas.push_back(0);
    }
}

// insert a given vertex into local vertices pool
void Mesh::insert_vertex(float m1, float m2, float m3)
{
    m_vertices.push_back(Matrix(m1, m2, m3));
    m_num_vertices++;
}

// insert a given triangle into local triangular meshes
// m_k: the triangle's vertices
// n_idx_k: the neighber's index
// v_idx_k: the vertices' index
void Mesh::insert_triangle(Matrix m1, Matrix m2, Matrix m3,
                           int n_idx1, int n_idx2, int n_idx3, int v_idx1, int v_idx2, int v_idx3)
{
    m_triangles.push_back(TriangleMesh(&m_modes, n_idx1, n_idx2, n_idx3, v_idx1, v_idx2, v_idx3, &m_lambdas));
    m_num_triangles++;
}

// return the vertex at a given index, most used for accessing a coordinate of the triangular mesh
Matrix Mesh::get_vertex_at(int idx)
{
    assert((int)m_vertices.size() > idx);
    return m_vertices.at(idx);
}

// function to find the closest point for a matrix inside the triangular meshes
// used for final linear searches after tree-based search has reached its minimum
// threshold
std::tuple<TriangleMesh, Matrix> Mesh::find_closest_point(Matrix mat)
{
    Matrix min_mat;            // closest point
    float min_dist = INFINITY; // bound
    TriangleMesh min_tri;

    // loop through all triangles to find the closest point
    for (int i = 0; i < m_num_triangles; i++)
    {
        TriangleMesh trig = m_triangles.at(i);
        std::tuple<float, Matrix> trig_find = trig.find_closest_point_in_triangle(mat);
        float dist = std::get<0>(trig_find);

        // update the bound and closest point if the current one is closer to the
        // target than the original closest point.
        if (dist < min_dist)
        {
            min_dist = dist;
            min_mat = std::get<1>(trig_find);
            min_tri = trig;
        }
    }

    // return both the triangular mesh and the closest point
    return std::make_tuple(min_tri, min_mat);
}

// use tree search for find the closest point for a given matrix
std::vector<std::tuple<TriangleMesh, Matrix>> Mesh::find_closest_point_advanced(const std::vector<Matrix> &mat,
                                                                                BoundingBoxTreeNode *node) const
{
    // vectors to store the closest point to each target matrix.
    std::vector<std::tuple<TriangleMesh, Matrix>> closest_sets;

    // initialize the closest point for each target matrix.
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        closest_sets.push_back(std::make_tuple(TriangleMesh(), Matrix(3, 1)));
    }

    // find the closest point in each target matrix using the bounding box search
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        float bound = INFINITY;
        node->findClosestPoint(mat.at(i), bound, closest_sets.at(i));
    }
    return closest_sets;
}

// finding the optinum transformation in a deformed context
std::tuple<Frame, std::vector<std::tuple<TriangleMesh, Matrix>>> Mesh::deformed_find_optimum_transformation(const std::vector<Matrix> &mat,
                                                                                                            float threshold, bool advanced)
{
    // make a copy of the c Matrix
    std::vector<Matrix> mat_copy(mat);
    // initiate the previous error and current error values to be sufficiently big
    float pre_err = INFINITY;
    float cur_err = 100000;
    // initiate a frame transformation with identity rotation and zero translation
    Frame f_reg = Frame();

    std::vector<std::tuple<TriangleMesh, Matrix>> closest_set;

    while (1)
    {
        // initiate the matrix data structures to store the values c (closest point sets),
        // q0_k (all the mode 0 vertex values), qm_k (all the mode non-zero vertex values)
        Matrix q0k(3 * (int)mat_copy.size(), 1);
        Matrix qmk(3 * (int)mat_copy.size(), (int)m_lambdas.size());
        Matrix c0(3 * (int)mat_copy.size(), 1);

        // obtain the current iterations optimum transformation by calling the helper function
        auto iter_output = find_optimum_transformation(mat_copy, f_reg, threshold, advanced);
        closest_set = std::get<0>(iter_output);
        pre_err = cur_err;
        cur_err = std::get<1>(iter_output);

        // check the threshold to determine whether to continue updated deformed ratios
        if (cur_err / pre_err > threshold)
        {
            break;
        }
        
        // obtain the current coefficients for each closest point with respect to the trhee coordiates
        // of the triangular mesh it belong to.
        // fill matrices q0_k, qm_k, and c0 based on the coefficients and previous data
        Matrix coefs;
        for (int i = 0; i < (int)closest_set.size(); ++i)
        {
            TriangleMesh tri = std::get<0>(closest_set.at(i));
            Matrix c = std::get<1>(closest_set.at(i));
            coefs = tri.get_bary(c);
            Matrix q0k_mat = tri.get_original_coord(0) * coefs.get_pos(0, 0) +
                             tri.get_original_coord(1) * coefs.get_pos(1, 0) +
                             tri.get_original_coord(2) * coefs.get_pos(2, 0);
            q0k.assign(0 + 3 * i, 0, q0k_mat.get_pos(0, 0));
            q0k.assign(1 + 3 * i, 0, q0k_mat.get_pos(1, 0));
            q0k.assign(2 + 3 * i, 0, q0k_mat.get_pos(2, 0));
            c0.assign(0 + 3 * i, 0, c.get_pos(0, 0));
            c0.assign(1 + 3 * i, 0, c.get_pos(1, 0));
            c0.assign(2 + 3 * i, 0, c.get_pos(2, 0));
            for (int j = 0; j < (int)m_lambdas.size(); ++j)
            {
                Matrix qmk_mat = tri.get_mode_coord(0, j) * coefs.get_pos(0, 0) +
                                 tri.get_mode_coord(1, j) * coefs.get_pos(1, 0) +
                                 tri.get_mode_coord(2, j) * coefs.get_pos(2, 0);
                qmk.assign(0 + 3 * i, j, qmk_mat.get_pos(0, 0));
                qmk.assign(1 + 3 * i, j, qmk_mat.get_pos(1, 0));
                qmk.assign(2 + 3 * i, j, qmk_mat.get_pos(2, 0));
            }
        }

        // initiate a new data structure to store transformed matrices of query
        Matrix s(3 * (int)mat_copy.size(), 1);
        // recalculate the estimate for each point using the finalized frame
        // transformation estimate
        for (int k = 0; k < (int)mat_copy.size(); ++k)
        {
            Matrix f_mat = f_reg * mat_copy.at(k);
            s.assign(0 + 3 * k, 0, f_mat.get_pos(0, 0));
            s.assign(1 + 3 * k, 0, f_mat.get_pos(1, 0));
            s.assign(2 + 3 * k, 0, f_mat.get_pos(2, 0));
        }

        Matrix s_minus_q0k = s - q0k;

        // least square to find lambdas that minimize |s - (q0k + lambda * qmk)  
        //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        Matrix lambda = Matrix(qmk.get().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(s_minus_q0k.get()));
        // assign the lambdas with adjusted values
        for (int z = 0; z < (int)m_lambdas.size(); ++z)
        {
            m_lambdas.at(z) = lambda.get_pos(z, 0);
        }
    }
    // return the estimated frame transformation and corresponding closest points
    return std::make_tuple(f_reg, closest_set);
}

// finding the optimum transformation provided a setting of lambda values,
// helper function to deformed find optimum transformation
std::tuple<std::vector<std::tuple<TriangleMesh, Matrix>>, float> Mesh::find_optimum_transformation(const std::vector<Matrix> &mat, Frame &frame, float threshold, bool advanced)
{
    std::vector<Matrix> mat_copy(mat);
    // initiate a frame transformation with identity rotation and zero translation
    // initiate the previous error and current error values to be sufficiently big
    float pre_err = 10000;
    float cur_err = 1000;

    // initiate the matrix to store the closest points
    std::vector<std::tuple<TriangleMesh, Matrix>> c_ks;

    // iterate while the ratio between current error and previous err has not exceeded the
    // threshold

    // allocate space for an array of BoundingSphere pointers.
    BoundingSphere **spheres = new BoundingSphere *[m_num_triangles];
    int nSphere = 0;

    for (int i = 0; i < m_num_triangles; i++)
    {
        spheres[nSphere++] = new BoundingSphere(m_triangles.at(i));
    }

    // construct BoundingBoxTreeNode
    BoundingBoxTreeNode node(spheres, nSphere, false);

    while (cur_err / pre_err <= threshold)
    {
        // find the current error and modify the frame transformation as well as the closest point
        // sets using find_transformation_helper function
        pre_err = cur_err;
        cur_err = find_transformation_helper(mat_copy, c_ks, frame, &node, advanced, false);
    }
    c_ks.clear();
    std::vector<Matrix> s;
    std::vector<Matrix> c;
    // recalculate the estimate for each point using the finalized frame
    // transformation estimate
    for (Matrix m : mat)
    {
        s.push_back(frame * m);
    }
    // recalculate the closest points
    if (advanced)
    {
        c_ks = find_closest_point_advanced(s, &node);
        for (int i = 0; i < (int)c_ks.size(); ++i)
        {
            c.push_back(std::get<1>(c_ks.at(i)));
        }
    }
    else
    {
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
            c.push_back(std::get<1>(c_ks.at(i)));
        }
    }
    // return both the frame transformation estimates and the set of closest points
    for (int i = 0; i < nSphere; ++i)
    {
        delete spheres[i]; // Deletes each BoundingSphere object
    }

    delete[] spheres;

    return std::make_tuple(c_ks, cur_err);
}

// find a specific transformation based on previous transformation offsets,
// helper function to find optimum transformation
float Mesh::find_transformation_helper(std::vector<Matrix> &mat, std::vector<std::tuple<TriangleMesh, Matrix>> &c_ks, Frame &frame, BoundingBoxTreeNode *node,
                                       bool advanced, bool has_outlier)
{
    c_ks.clear();
    // find the current estimates for points
    std::vector<Matrix> s;
    for (Matrix m : mat)
    {
        s.push_back(frame * m);
    }
    // find the closest points for each of the points
    if (advanced)
    {
        c_ks = find_closest_point_advanced(s, node);
    }
    else
    {
        c_ks.clear();
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
        }
    }
    // perform point-cloud registration. Adding all the closest points to
    // matrix b (before transformation) and all the estimate points to matrix a
    // (after transformation)
    Registration regis = Registration();
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        regis.add_matrix_a(s.at(i));
        regis.add_matrix_b(std::get<1>(c_ks.at(i)));
    }
    // modify the current frame transformation estimate with the new residual frame
    // transformation
    Frame modification = regis.point_cloud_registration();
    frame = regis.point_cloud_registration() * frame;

    // initiate error float, and add the error magnitude of each point
    float err = 0;
    std::vector<Matrix> s_s;
    std::vector<Matrix> c_s;
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        s_s.push_back(frame * mat.at(i));
        c_s.push_back(std::get<1>(c_ks.at(i)));
    }
    err = get_error(s_s, c_s);

    // if outliers are expected to be excluded, then delete any point whose error is larger than
    // three times the average error
    if (has_outlier)
    {
        for (int i = 0; i < (int)mat.size(); ++i)
        {
            float individual_err = ((frame * mat.at(i)) - std::get<1>(c_ks.at(i))).magnitude();
            if (individual_err > 3 * err)
            {
                mat.erase(mat.begin() + i);
            }
        }
    }
    // return the average error
    return err;
}

// helper function to obtain error value based on current frame transformation and lambdas estimates
float Mesh::get_error(std::vector<Matrix> s_s, std::vector<Matrix> c_s)
{
    // plug in s and c into a big matrix with dimension (3 * len(s), 1)
    Matrix s_mat = Matrix(s_s.size() * 3, 1);
    Matrix c_mat = Matrix(c_s.size() * 3, 1);
    for (int i = 0; i < (int)c_s.size(); i++)
    {
        s_mat.assign(0 + i * 3, 0, s_s.at(i).get_pos(0, 0));
        s_mat.assign(1 + i * 3, 0, s_s.at(i).get_pos(1, 0));
        s_mat.assign(2 + i * 3, 0, s_s.at(i).get_pos(2, 0));
        c_mat.assign(0 + i * 3, 0, c_s.at(i).get_pos(0, 0));
        c_mat.assign(1 + i * 3, 0, c_s.at(i).get_pos(1, 0));
        c_mat.assign(2 + i * 3, 0, c_s.at(i).get_pos(2, 0));
    }
    // directly compare the two large matrices and return their difference in magnitude
    return (s_mat - c_mat).magnitude();
}