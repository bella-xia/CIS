#ifndef BOUNDING_BOX_TREE_NODE_H
#define BOUNDING_BOX_TREE_NODE_H

#include "bounding_sphere.h"

/**
 * CITATION: This class is created basing on the "finding point-pairs" slides.
 * This class build a boudning box searching tree node, increasing the
 * efficiency of searching by ignoring the whole box completely if its
 * closest distance is longer than the current closest distance.
 * This class contains:
 * All BoundingSpheres inside the box;
 * The subtrees;
 */
class BoundingBoxTreeNode
{
private:
    // spliting point
    Matrix center;
    // corners of box
    Matrix UB; // upper bound
    Matrix LB; // lower bound
    bool haveSubtrees = false;
    int nSpheres = 0;
    bool m_iter = false;
    float maxRadius = -1;

    int minCount = 3;  // minimum trees required to construct subtrees
    float minDiag = 3; // minimum diagonal required to construct subtree

    BoundingBoxTreeNode *subtrees[2][2][2];
    BoundingSphere **spheres;

    /* Calculate the new splitting point by getting the mean x,y,z of
     * the center of all spheres*/
    void calculateCenter();

    /* Calculate the maximum radius of all spheres. */
    void calculateMaxRadius();

    /* Calculate the upper and lower bound of the box. */
    void calcultateMaxMinCoordinate();

    /* Update the count of the subtree basing on the number of triangles
     * in each side of the center.
     * Sort the subtrees such that the first nnn are in subtree[0][0][0],
     * then the npn are in subtree [0][1][1]...*/
    void splitSort(int &nnn, int &npn, int &npp, int &nnp, int &pnn, int &ppn, int &ppp, int &pnp);

public:
    /* Default constructor. */
    BoundingBoxTreeNode();

    /* Construct the BoundingBoxTreeNode basing on the BoundingSpheres and sphere num.*/
    BoundingBoxTreeNode(BoundingSphere **BS, int sphereNum, bool iter);

    ~BoundingBoxTreeNode();

    /* Construct subtrees if  minCount and diagnol length */
    void constructSubtrees();

    /* Update the closest and bound if there is a tree with distance shorter than the bound.*/
    void findClosestPoint(Matrix v, float &bound, std::tuple<TriangleMesh, Matrix> &closest);
};

#endif