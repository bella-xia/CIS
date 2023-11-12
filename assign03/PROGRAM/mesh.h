#ifndef MESH_H
#define MESH_H

#include "bounding_box_tree_node.h"

class Mesh
{
/**
 * This class creates an abstraction of the 3D structures consists of triangles.
*/
private:
    std::vector<Matrix> m_vertices; 
    std::vector<TriangleMesh> m_triangles;
    int m_num_vertices, m_num_triangles;

public:
    // Default constructor of the Mesh
    Mesh();

    // Insert a vertex into the mesh.
    void insert_vertex(float m1, float m2, float m3);

    // Construct a triangle besing on mk (vertices of the triangle)
    // n_idxk (the index of its neightbers) and v_idxk (the index of its vertices).
    // This triangle is then inserted into m_triangles vector.
    void insert_triangle(Matrix m1, Matrix m2, Matrix m3,
                         int n_idx1, int n_idx2, int n_idx3, int v_idx1, int v_idx2, int v_idx3);
    
    // Return the vertex at a given index.
    Matrix get_vertex_at(int idx);

    // Return the closest point in one of the triangle relative to a given Matrix mat
    // through SIMPLE SEARCH
    Matrix find_closest_point(Matrix mat);

    // Return the closest point in one of the triangle relative to a given Matrix mat
    // through BOUNDING BOX TREE NODE SEARCH
    std::vector<Matrix> find_closest_point_advanced(const std::vector<Matrix> &mat) const;
};

#endif
