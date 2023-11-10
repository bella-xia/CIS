#ifndef MESH_H
#define MESH_H

#include "triangle_mesh.h"

class Mesh
{
private:
    std::vector<Matrix> m_vertices;
    std::vector<TriangleMesh> m_triangles;
    int m_num_vertices, m_num_triangles;

public:
    Mesh();

    void insert_vertex(float m1, float m2, float m3);
    void insert_triangle(Matrix m1, Matrix m2, Matrix m3,
                         int n_idx1, int n_idx2, int n_idx3, int v_idx1, int v_idx2, int v_idx3);
    Matrix get_vertex_at(int idx);
    Matrix find_closest_point(Matrix mat);
};

#endif
