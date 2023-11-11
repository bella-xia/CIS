#ifndef BOUNDING_SPHERE_H
#define BOUNDING_SPHERE_H

#include "triangle_mesh.h"
class BoundingSphere
{
private:
    // the center and radius of the sphere
    Matrix center;
    float radius;
    int m_idx;
    // the triangle contained in the sphere
    TriangleMesh triangle;
    bool isCenter(Matrix a1, Matrix b1, Matrix c1, Matrix q);
    Matrix calculateCenter(Matrix a, Matrix b, Matrix c);
    std::vector<Matrix> getLongestEdge(Matrix a1, Matrix b1, Matrix c1);

public:
    BoundingSphere();
    BoundingSphere(TriangleMesh tri);
    Matrix getCenter();
    float getRadius() { return radius; }
    TriangleMesh getTriangle() { return triangle; }

    bool mightBeCloser(Matrix a, float bound);
};
#endif