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
    float qa = (q - a).magnitude();
    float qb = (q - b).magnitude();
    float qc = (q - c).magnitude();
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
    // std::cout << radius << std::endl;
}

Matrix BoundingSphere::getCenter()
{
    // std::cout << "got here" << std::endl;
    // std::cout << "center dimension:" << center.get_col() << ", " << center.get_row() << std::endl;
    // Matrix returns(center.get());
    // center.print_str();
    // std::cout << "trying to get triangle" << std::endl;
    // triangle.get_coord_at(0).print_str();
    // std::cout << "end get triangle" << std::endl;
    // std::cout << "index:  " << m_idx << std::endl;
    // std::cout << "trying to get radius" << std::endl;
    // std::cout << "get radius: " << getRadius() << std::endl;
    // std::cout << "end getting radius" << std::endl;
    return center;
}

bool BoundingSphere::isCenter(Matrix a, Matrix b, Matrix c, Matrix q)
{
    float b_q = (((b - q)).transpose() * (b - q)).get_pos(0, 0);
    float a_q = (((a - q)).transpose() * (a - q)).get_pos(0, 0);
    float c_q = ((c - q).transpose() * (c - q)).get_pos(0, 0);
    float m = ((b - a).cross(c - a).transpose() * (q - a)).get_pos(0, 0);
    return (b_q == a_q && c_q <= a_q && m == 0);
}

Matrix BoundingSphere::calculateCenter(Matrix a1, Matrix b1, Matrix c1)
{
    std::vector<Matrix> edges = getLongestEdge(a1, b1, c1);
    Matrix a = edges[0];
    Matrix b = edges[1];
    Matrix c = edges[2];
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
