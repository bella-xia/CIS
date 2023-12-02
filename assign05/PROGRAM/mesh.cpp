#include "mesh.h"

Mesh::Mesh() : m_vertices(std::vector<Matrix>()), m_triangles(std::vector<TriangleMesh>()),
               m_num_vertices(0), m_num_triangles(0) {}

Mesh::~Mesh()
{
}

void Mesh::insert_vertex(float m1, float m2, float m3)
{
    m_vertices.push_back(Matrix(m1, m2, m3));
    m_num_vertices++;
}
void Mesh::insert_triangle(Matrix m1, Matrix m2, Matrix m3,
                           int n_idx1, int n_idx2, int n_idx3, int v_idx1, int v_idx2, int v_idx3)
{
    m_triangles.push_back(TriangleMesh(m1, m2, m3, n_idx1, n_idx2, n_idx3, v_idx1, v_idx2, v_idx3));
    m_num_triangles++;
}

Matrix Mesh::get_vertex_at(int idx)
{
    assert((int)m_vertices.size() > idx);
    return m_vertices.at(idx);
}

Matrix Mesh::find_closest_point(Matrix mat)

{
    Matrix min_mat;            // closest point
    float min_dist = INFINITY; // bound

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
        }
    }

    return min_mat;
}

std::vector<Matrix> Mesh::find_closest_point_advanced(const std::vector<Matrix> &mat) const
{
    // allocate space for an array of BoundingSphere pointers.
    BoundingSphere **spheres = new BoundingSphere *[m_num_triangles];
    int nSphere = 0;

    for (int i = 0; i < m_num_triangles; i++)
    {
        spheres[nSphere++] = new BoundingSphere(m_triangles.at(i));
    }

    // construct BoundingBoxTreeNode
    BoundingBoxTreeNode node(spheres, nSphere, false);

    // vectors to store the closest point to each target matrix.
    std::vector<Matrix> closests;

    // initialize the closest point for each target matrix.
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        closests.push_back(Matrix(3, 1));
    }

    // find the closest point in eahc target matrix using the bounding box search
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        float bound = INFINITY;
        node.findClosestPoint(mat.at(i), bound, closests.at(i));
    }

    for (int i = 0; i < nSphere; ++i)
    {
        delete spheres[i]; // Deletes each BoundingSphere object
    }

    delete[] spheres;

    return closests;
}

std::tuple<Frame, std::vector<Matrix>> Mesh::find_optimum_transformation(const std::vector<Matrix> &mat, float threshold, bool advanced)
{
    std::vector<Matrix> mat_copy(mat);
    // initiate a frame transformation with identity rotation and zero translation
    Frame estimate = Frame(Rotation(), Position());
    // initiate the previous error and current error values to be sufficiently big
    float pre_err = 10000;
    float cur_err = 1000;

    // initiate the matrix to store the closest points
    std::vector<Matrix> c_ks;

    // iterate while the ratio between current error and previous err has not exceeded the
    // threshold
    while (cur_err / pre_err <= threshold)
    {
        // find the current error and modify the frame transformation as well as the closest point
        // sets using find_transformation_helper function
        pre_err = cur_err;
        cur_err = find_transformation_helper(mat_copy, c_ks, estimate, advanced);
        std::cout << "current error: " << cur_err << std::endl;
    }
    c_ks.clear();
    std::vector<Matrix> s;
    // recalculate the estimate for each point using the finalized frame
    // transformation estimate
    for (Matrix m : mat)
    {
        s.push_back(estimate * m);
    }
    // recalculate the closest points
    if (advanced)
    {
        c_ks = find_closest_point_advanced(s);
    }
    else
    {
        c_ks.clear();
        for (int i = 0; i < (int)s.size(); ++i)
        {
            c_ks.push_back(find_closest_point(s.at(i)));
        }
    }
    // return both the frame transformation estimates and the set of closest points
    return std::make_tuple(estimate, c_ks);
}

float Mesh::find_transformation_helper(std::vector<Matrix> &mat, std::vector<Matrix> &c_ks, Frame &frame, bool advanced, bool has_outlier)
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
        c_ks = find_closest_point_advanced(s);
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
        regis.add_matrix_b(c_ks.at(i));
    }
    // modify the current frame transformation estimate with the new residual frame
    // transformation
    frame = regis.point_cloud_registration() * frame;

    // initiate error float, and add the error magnitude of each point
    float err = 0;
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        err += ((frame * mat.at(i)) - c_ks.at(i)).magnitude();
    }
    err /= (int)mat.size();

    // if outliers are expected to be excluded, then delete any point whose error is larger than
    // three times the average error
    if (has_outlier)
    {
        for (int i = 0; i < (int)mat.size(); ++i)
        {
            float individual_err = ((frame * mat.at(i)) - c_ks.at(i)).magnitude();
            if (individual_err > 3 * err)
            {
                mat.erase(mat.begin() + i);
                std::cout << "erased matrix at index " << i << std::endl;
            }
        }
    }
    // return the average error
    return err;
}
