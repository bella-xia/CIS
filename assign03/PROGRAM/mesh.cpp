#include "mesh.h"

Mesh::Mesh() : m_vertices(std::vector<Matrix>()), m_triangles(std::vector<TriangleMesh>()),
               m_num_vertices(0), m_num_triangles(0) {}

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
    Matrix min_mat;
    float min_dist = INFINITY;
    for (TriangleMesh trig : m_triangles)
    {
        std::tuple<float, Matrix> trig_find = trig.find_closest_point_in_triangle(mat);
        // std::cout << "got here" << std::endl;
        float dist = std::get<0>(trig_find);
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
    BoundingSphere **spheres = new BoundingSphere *[m_num_triangles];
    std::cout << "number triangles: " << m_num_triangles << std::endl;
    int nSphere = 0;
    for (int i = 0; i < m_num_triangles; i++)
    {
        spheres[nSphere++] = new BoundingSphere(m_triangles.at(i));
    }
    std::cout << "finsih filling with " << nSphere << " spheres" << std::endl;
    BoundingBoxTreeNode node(spheres, nSphere);
    std::vector<Matrix> closests;
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        closests.push_back(Matrix(3, 1));
    }
    float bound = INFINITY;
    for (int i = 0; i < (int)mat.size(); ++i)
    {
        node.findClosestPoint(mat.at(i), bound, closests.at(i));
    }
    return closests;
}
