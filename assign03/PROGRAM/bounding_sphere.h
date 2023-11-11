#ifndef BOUNDING_SPHERE_H
#define BOUNDING_SPHERE_H

#include "triangle_mesh.h"
class BoundingSphere{
    private:
        Matrix center;
        float radius;
        TriangleMesh triangle;
        bool isCenter(Matrix a1, Matrix b1, Matrix c1, Matrix q);
        Matrix calculateCenter(Matrix a, Matrix b, Matrix c);
        std::vector<Matrix> getLongestEdge(Matrix a1, Matrix b1, Matrix c1);
     
    public:
       
        BoundingSphere() {}
        BoundingSphere(TriangleMesh tri);
        Matrix getCenter(){return center;}
        float getRadius() {return radius;}
        TriangleMesh getTriangle() {return triangle;}
        
        bool mightBeCloser(Matrix a, float bound);
};
#endif