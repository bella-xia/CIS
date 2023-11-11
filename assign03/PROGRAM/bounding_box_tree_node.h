#ifndef BOUNDING_BOX_TREE_NODE_H
#define BOUNDING_BOX_TREE_NODE_H

#include "bounding_sphere.h"

class BoundingBoxTreeNode
{
private:
    // spliting point
    Matrix center;
    // corners of box
    Matrix UB;
    Matrix LB;
    bool haveSubtrees = false;
    int nSpheres = 0;
    float maxRadius = -1;

    int minCount = 3;
    float minDiag = 3;

    BoundingBoxTreeNode *subtrees[2][2][2];
    BoundingSphere **spheres;
    void calculateCenter();
    void calculateMaxRadius();
    void calcultateMaxMinCoordinate();
    void splitSort(int &nnn, int &npn, int &npp, int &nnp, int &pnn, int &ppn, int &ppp, int &pnp);

public:
    BoundingBoxTreeNode();
    BoundingBoxTreeNode(BoundingSphere **BS, int sphereNum);
    void constructSubtrees();
    void findClosestPoint(Matrix v, float &bound, Matrix &closest);
};

#endif