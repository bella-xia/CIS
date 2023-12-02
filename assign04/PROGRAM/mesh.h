#ifndef MESH_H
#define MESH_H

#include "bounding_box_tree_node.h"
#include "registration.h"

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

    ~Mesh();

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

    //  iteratively find the optimum transformation based on the set of points provided in the parameter
    // paired with the triangular meshes stored in Mesh class member variables. The threshold is used to
    // indicate the condition to stop iteration. advanced specifies whether advanced or linear closest point
    // search is used.
    std::tuple<Frame, std::vector<Matrix>> find_optimum_transformation(const std::vector<Matrix> &mat,
                                                                       float threshold, bool advanced = true);

private:
    // One iteration of finding the closest point correspondence to each point of interest, calculate the point cloud
    // registration, then return the error of the current frame transformation estimation. Helper function to
    // find-optimum-transformation
    float find_transformation_helper(std::vector<Matrix> &mat, std::vector<Matrix> &c_ks, Frame &frame,
                                     bool advanced = true, bool has_outlier = false);
};

#endif
