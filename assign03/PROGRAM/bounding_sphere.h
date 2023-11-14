#ifndef BOUNDING_SPHERE_H
#define BOUNDING_SPHERE_H

#include "triangle_mesh.h"
class BoundingSphere
/**
 * CITATION: This class is created basing on the "finding point-pairs" slides.
 * This class creates a structure of a bounding sphere basing on a triangle,
 * such that all three vertices of the triangle is on the outline of sphere.
*/
{
private:
    // the center and radius of the sphere
    Matrix center;
    float radius;
    int m_idx;
    // the triangle contained in the sphere
    TriangleMesh triangle;
    
    /* Check whether a given vector q is the center of the circle built on the
     * triangle with vertices a, b, and c, and a-b is the longest edge */
    bool isCenter(Matrix a, Matrix b, Matrix c, Matrix q);

    /* Calculate the center of the circle built on the triangle with vertices
     * a, b, and c. */
    Matrix calculateCenter(Matrix a, Matrix b, Matrix c);

    /* Get the longest edge of the triangle with vertices a1, b1, and c1. The
     * first two vectors in the returning vector is two ends of the longest 
     * edge and the third is the other vertex. */
    std::vector<Matrix> getLongestEdge(Matrix a1, Matrix b1, Matrix c1);

public:
    /* Default constructor. */
    BoundingSphere();

    /* Construct a bounding sphere basing on an input sphere. */
    BoundingSphere(TriangleMesh tri);

    ~BoundingSphere();

    /* Return the center of the bounding sphere. */
    Matrix getCenter() {return center; }

    /* Return the radius of the boudning sphere. */
    float getRadius() { return radius; }

    /* Return the triangle of the bounding sphere. */
    TriangleMesh getTriangle() { return triangle; }

    /* Check wthere it is possible for this tirangle in the sphere to have 
     * a closer distance to the matrix a than a given bound.*/
    bool mightBeCloser(Matrix a, float bound);
};
#endif