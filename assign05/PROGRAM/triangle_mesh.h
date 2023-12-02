#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H

#include "matrix.h"

class TriangleMesh
{
    /**
     * CITATION: This class is created basing on the "finding point-pairs" slides.
     * This class abstracts a triangle, cotaining the following information:
     * 1. the three coordinates of the triangle.
     * 2. the neighbor of the triangles. The neighber[0] corresponding to the
     * triangle opposite of the coordinate[0].
     */
private:
    std::vector<Matrix> m_coords;
    std::vector<int> m_neighbor_index;
    std::vector<int> m_vertex_index;

    // Calculate the Barycentric formula to find the cloest point in the triangle.
    Matrix get_bary(Matrix mat);

    // Calculate the projection of the target_mat on the edge formed by vertex_idx1 and vertex_idx2.
    std::tuple<float, Matrix> get_project(Matrix &target_mat, int vertex_idx1, int vertex_idx2);

public:
    // Default constructor
    TriangleMesh();

    // Construct a triangle basing on the vertices, neightber and vertice indices.
    TriangleMesh(Matrix mat1, Matrix mat2, Matrix mat3,
                 int neighbor_idx1, int neighbor_idx2, int neighbor_idx3,
                 int vertex_idx1, int vertex_idx2, int vertex_idx3);

    ~TriangleMesh();

    // Return the vertices of the triangle
    std::vector<Matrix> get_coords() const { return m_coords; }

    // Return the vertice of a given index of the triangle.
    Matrix get_coord_at(int idx) const { return m_coords[idx]; }

    // Return the closest point on the triangle to a given matrix, along with the distance.
    std::tuple<float, Matrix> find_closest_point_in_triangle(Matrix mat);
};
#endif