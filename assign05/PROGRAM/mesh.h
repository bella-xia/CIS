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
    std::vector<Matrix> m_vertices; // data of triangles' vertices
    std::vector<std::vector<Matrix>> m_modes; // modes data, where mode[0] stores the original vertice value.
    std::vector<TriangleMesh> m_triangles; // triangles in the mesh
    std::vector<float> m_lambdas; // weight (lambda) associated with each mode
    int m_num_vertices, m_num_triangles; // number of vertices and number of triangles

public:
    // Default constructor of the Mesh
    Mesh();

    Mesh(int n_m, std::vector<std::vector<Matrix>> modes);

    ~Mesh();

    // initialize lambda to be 0
    void initiate_lambdas(int n_m = 1);

    // Insert a vertex into the mesh.
    void insert_vertex(float m1, float m2, float m3);

    // Construct a triangle besing on mk (vertices of the triangle)
    // n_idxk (the index of its neightbers) and v_idxk (the index of its vertices).
    // This triangle is then inserted into m_triangles vector.
    void insert_triangle(Matrix m1, Matrix m2, Matrix m3,
                         int n_idx1, int n_idx2, int n_idx3, int v_idx1, int v_idx2, int v_idx3);
    
    // insert data of modes into the mesh
    void add_modes(const std::vector<std::vector<Matrix>> &modes) { m_modes = modes; }
    
    // return the lambdas
    std::vector<float> get_lambdas() { return m_lambdas; }

    // Return the vertex at a given index.
    Matrix get_vertex_at(int idx);

    // Return the closest point in one of the triangle relative to a given Matrix mat
    // through SIMPLE SEARCH
    std::tuple<TriangleMesh, Matrix> find_closest_point(Matrix mat);

    // Return the closest point in one of the triangle relative to a given Matrix mat
    // through BOUNDING BOX TREE NODE SEARCH
    std::vector<std::tuple<TriangleMesh, Matrix>> find_closest_point_advanced(const std::vector<Matrix> &mat, BoundingBoxTreeNode *node) const;

    //  iteratively find the optimum transformation based on the set of points provided in the parameter
    // paired with the triangular meshes stored in Mesh class member variables. The threshold is used to
    // indicate the condition to stop iteration. advanced specifies whether advanced or linear closest point
    // search is used.
    std::tuple<std::vector<std::tuple<TriangleMesh, Matrix>>, float> find_optimum_transformation(const std::vector<Matrix> &mat, Frame &frame,
                                                                                                 float threshold, bool advanced = true);

    // iteratively find the optimum transformation when deformation with respect to a set of provided modes exist
    std::tuple<Frame, std::vector<std::tuple<TriangleMesh, Matrix>>> deformed_find_optimum_transformation(const std::vector<Matrix> &mat,
                                                                                                          float threshold, bool advanced = true);

private:
    // One iteration of finding the closest point correspondence to each point of interest, calculate the point cloud
    // registration, then return the error of the current frame transformation estimation. Helper function to
    // find-optimum-transformation
    float find_transformation_helper(std::vector<Matrix> &mat, std::vector<std::tuple<TriangleMesh, Matrix>> &c_ks, Frame &frame, BoundingBoxTreeNode *node,
                                     bool advanced = true, bool has_outlier = false);
    
    // calculate the error between two sets of points.
    float get_error(std::vector<Matrix> s_s, std::vector<Matrix> c_s);
};

#endif
