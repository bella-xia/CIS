#include "bounding_sphere.h"
#include <algorithm>

BoundingSphere::BoundingSphere() : center(Matrix(3, 1)) {}

// the most commonly used constructor
BoundingSphere::BoundingSphere(TriangleMesh tri) : center(Matrix(3, 1)), m_idx(1)
{
    triangle = tri;
    Matrix a = tri.get_coord_at(0);
    Matrix b = tri.get_coord_at(1);
    Matrix c = tri.get_coord_at(2);
    Matrix q = calculateCenter(a, b, c);
    center = q;

    float qa = (q - a).magnitude(); //qa = ||q - a||
    float qb = (q - b).magnitude(); //qb = ||q - b||
    float qc = (q - c).magnitude(); //qc = ||q - c||

    // set the longest of qa, qb, qc to be the radius.
    if (qa > qb && qa > qc)
    {
        radius = qa;
    }
    else if (qb > qa && qb > qc)
    {
        radius = qb;
    }
    else
    {
        radius = qc;
    }
}

bool BoundingSphere::isCenter(Matrix a, Matrix b, Matrix c, Matrix q)
{   
    /** 
     * If a-b is the longest edge and q is the center of the sphere, it must satisfy:
     * 1. (b - q) * (b - q) = (a - q) * (a - q)
     * 2. (c - q) * (c - q) <= (a - q) * (a - q)
     * 3. (b - a) x (c - a) * (q - a) = 0
    */
    float b_q = (((b - q)).transpose() * (b - q)).get_pos(0, 0); // calculate 1 left
    float a_q = (((a - q)).transpose() * (a - q)).get_pos(0, 0); // calculate 1 & 2 right
    float c_q = ((c - q).transpose() * (c - q)).get_pos(0, 0); // calculate 2 left
    float m = ((b - a).cross(c - a).transpose() * (q - a)).get_pos(0, 0); // calculate 3 left
    return (b_q == a_q && c_q <= a_q && m == 0);
}

Matrix BoundingSphere::calculateCenter(Matrix a1, Matrix b1, Matrix c1)
{
    std::vector<Matrix> edges = getLongestEdge(a1, b1, c1);
    Matrix a = edges[0];
    Matrix b = edges[1];
    Matrix c = edges[2];
    // check whether the midpoint of a-b is the center.
    Matrix f = (a + b) * 0.5; 

    if (isCenter(a, b, c, f))
    {
        return f;
    }

    Matrix u = a - f;
    Matrix v = c - f;
    Matrix d = (u.cross(v)).cross(u);
    float y = ((v.transpose() * v).get_pos(0, 0) - (u.transpose() * u).get_pos(0, 0)) / ((d.transpose() * (v - u)).get_pos(0, 0) * 2);
    y = (y <= 0) ? 0 : y;
    return f + d * y;
}

std::vector<Matrix> BoundingSphere::getLongestEdge(Matrix a1, Matrix b1, Matrix c1)
{
    float a_b_distance = (a1 - b1).magnitude();
    float a_c_distance = (a1 - c1).magnitude();
    float b_c_distance = (b1 - c1).magnitude();
    Matrix *a;
    Matrix *b;
    Matrix *c;
    // result[0] and result[1] is the longest edge vertice;
    // result[2] is the other vertex
    std::vector<Matrix> result;
    if (a_b_distance > a_c_distance && a_b_distance > b_c_distance)
    {
        // if ab is the longest edge
        a = &a1;
        b = &b1;
        c = &c1;
    }
    else if (b_c_distance > a_c_distance && b_c_distance > a_b_distance)
    {
        // if bc is the longest edge
        a = &b1;
        b = &c1;
        c = &a1;
    }
    else
    {
        // if ac is the longest edge
        a = &a1;
        b = &c1;
        c = &b1;
    }
    result.push_back(*a);
    result.push_back(*b);
    result.push_back(*c);
    return result;
}

bool BoundingSphere::mightBeCloser(Matrix a, float bound)
{
    return (center - a).magnitude() - radius < bound;
}
