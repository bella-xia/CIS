#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H

#include "matrix.h"

class TriangleMesh
{
private:
    std::vector<Matrix> m_coords;
    std::vector<int> m_neighbor_index;
    std::vector<int> m_vertex_index;

    Matrix get_bary(Matrix mat);
    std::tuple<float, Matrix> get_project(Matrix &target_mat, int vertex_idx1, int vertex_idx2);

public:
    TriangleMesh();
    TriangleMesh(Matrix mat1, Matrix mat2, Matrix mat3,
                 int neighbor_idx1, int neighbor_idx2, int neighbor_idx3,
                 int vertex_idx1, int vertex_idx2, int vertex_idx3);
    std::vector<Matrix> get_coords() const { return m_coords; }
    Matrix get_coord_at(int idx) const { return m_coords[idx]; }
    std::tuple<float, Matrix> find_closest_point_in_triangle(Matrix mat);
};
#endif