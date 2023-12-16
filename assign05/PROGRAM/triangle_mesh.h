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
    std::vector<std::vector<Matrix>> *m_vertices_modes;
    std::vector<int> m_neighbor_index;
    std::vector<int> m_vertex_index;
    std::vector<float> *m_lambdas;

    // Calculate the Barycentric formula to find the cloest point in the triangle.

    float get_area(Matrix v1, Matrix v2, Matrix v3);

    // Calculate the projection of the target_mat on the edge formed by vertex_idx1 and vertex_idx2.
    std::tuple<float, Matrix> get_project(Matrix &target_mat, int vertex_idx1, int vertex_idx2);
    // check whether the point is in the triangle
    bool inTriangle(Matrix m);

public:
    // Default constructor
    TriangleMesh();

    // Construct a triangle basing on the vertices, neightber and vertice indices.
    TriangleMesh(std::vector<std::vector<Matrix>> *vertices_modes,
                 int neighbor_idx1, int neighbor_idx2, int neighbor_idx3,
                 int vertex_idx1, int vertex_idx2, int vertex_idx3,
                 std::vector<float> *lambdas);

    ~TriangleMesh();

    // Return the vertices of the triangle
    std::vector<Matrix> get_coords() const;
    // Return the vertice of a given index of the triangle.
    Matrix get_coord(int idx) const;

    // Return the original coordinates of the triangle without modes.
    Matrix get_original_coord(int idx) const { return m_vertices_modes->at(0).at(m_vertex_index.at(idx)); }

    // Return the mode associate with each coordinate
    Matrix get_mode_coord(int idx, int num_mode) const { return m_vertices_modes->at(num_mode + 1).at(m_vertex_index.at(idx)); }

    // Return the closest point on the triangle to a given matrix, along with the distance.
    std::tuple<float, Matrix> find_closest_point_in_triangle(Matrix mat);

    // Return the barycentric coordinate of a given matrix relative to the coordinates of the triangle.
    Matrix get_bary(Matrix mat);
};
#endif