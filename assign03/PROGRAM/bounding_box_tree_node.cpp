#include "bounding_box_tree_node.h"
#include <map>

BoundingBoxTreeNode::BoundingBoxTreeNode() {}

BoundingBoxTreeNode::BoundingBoxTreeNode(BoundingSphere **BS, int sphereNum) : BoundingBoxTreeNode()
{
    BoundingBoxTreeNode();
    spheres = BS;
    nSpheres = sphereNum;
    // Calculate the center, max radius, upper and lower bound
    // if the box is not empty.
    if (sphereNum != 0)
    {
        calculateCenter();
        calculateMaxRadius();
        calcultateMaxMinCoordinate();
        constructSubtrees();
    }
}
void BoundingBoxTreeNode::constructSubtrees()
{   
    // construct subtree only if the nsphere is larger than the 
    // minCount and the diagonal is longer than the minDiag
    if (nSpheres <= minCount || (UB - LB).magnitude() <= minDiag)
    {
        haveSubtrees = false;
        return;
    }
    haveSubtrees = true;
    int nnn, npn, npp, nnp, pnn, ppn, ppp, pnp;

    // Sort the spheres and update the number of sphere in each subtree. 
    splitSort(nnn, npn, npp, nnp, pnn, ppn, ppp, pnp);
    
    // temporary pointer for the sphere array.
    BoundingSphere **ptr = spheres;
    
    //construct the subtrees
    subtrees[0][0][0] = new BoundingBoxTreeNode(ptr, nnn);

    ptr += nnn; // move the pointer forward by nnn

    subtrees[0][1][0] = new BoundingBoxTreeNode(ptr, npn);
    ptr += npn;
    subtrees[0][1][1] = new BoundingBoxTreeNode(ptr, npp);
    ptr += npp;
    subtrees[0][0][1] = new BoundingBoxTreeNode(ptr, nnp);
    ptr += nnp;
    subtrees[1][0][0] = new BoundingBoxTreeNode(ptr, pnn);
    ptr += pnn;
    subtrees[1][1][0] = new BoundingBoxTreeNode(ptr, ppn);
    ptr += ppn;
    subtrees[1][1][1] = new BoundingBoxTreeNode(ptr, ppp);
    ptr += ppp;
    subtrees[1][0][1] = new BoundingBoxTreeNode(ptr, pnp);
}

void BoundingBoxTreeNode::findClosestPoint(Matrix v, float &bound, Matrix &closest)
{
    float dist = bound + maxRadius;
    float vx = v.get_pos(0, 0);
    float vy = v.get_pos(1, 0);
    float vz = v.get_pos(2, 0);

    // The whole box is out of the bound
    if (vx > UB.get_pos(0, 0) + dist || vy > UB.get_pos(1, 0) + dist || vz > UB.get_pos(2, 0) + dist ||
        vx < LB.get_pos(0, 0) - dist || vy < LB.get_pos(1, 0) - dist || vz < LB.get_pos(2, 0) - dist)
    {
        return;
    }
    // search all subtrees.
    if (haveSubtrees)
    {
        subtrees[0][0][0]->findClosestPoint(v, bound, closest);
        subtrees[0][0][1]->findClosestPoint(v, bound, closest);
        subtrees[0][1][0]->findClosestPoint(v, bound, closest);
        subtrees[0][1][1]->findClosestPoint(v, bound, closest);
        subtrees[1][0][0]->findClosestPoint(v, bound, closest);
        subtrees[1][0][1]->findClosestPoint(v, bound, closest);
        subtrees[1][1][0]->findClosestPoint(v, bound, closest);
        subtrees[1][1][1]->findClosestPoint(v, bound, closest);
    }
    else // check the sphere one by one to see whether there is a closer sphere
    {
        for (int i = 0; i < nSpheres; i++)
        {
            if (spheres[i]->mightBeCloser(v, bound))
            {
                std::tuple<float, Matrix> cur = spheres[i]->getTriangle().find_closest_point_in_triangle(v);
                float cur_dis = std::get<0>(cur);
                if (cur_dis < bound) // if there is a closer sphere
                {
                    bound = cur_dis; // update bound
                    closest = std::get<1>(cur); // update distance
                }
            }
        }
    }
}

void BoundingBoxTreeNode::calculateCenter()
{
    
    float x;
    float y;
    float z;
    // calculate the sum of x, y, z
    for (int i = 0; i < nSpheres; i++)
    {
        x += spheres[i]->getCenter().get_pos(0, 0);
        y += spheres[i]->getCenter().get_pos(1, 0);
        z += spheres[i]->getCenter().get_pos(2, 0);
    }
    // get the average x, y, z
    x /= nSpheres;
    y /= nSpheres;
    z /= nSpheres;
    center = Matrix(x, y, z);
}

void BoundingBoxTreeNode::calculateMaxRadius()
{
    //iterate through the spheres to find the max radius
    float maxR = 0;
    for (int i = 0; i < nSpheres; i++)
    {
        float sphereRadius = spheres[i]->getRadius();
        if (sphereRadius > maxR)
        {
            maxR = sphereRadius;
        }
    }
    maxRadius = maxR;
}

void BoundingBoxTreeNode::calcultateMaxMinCoordinate()
{
    
    float maxX = -1 * INFINITY;
    float maxY = -1 * INFINITY;
    float maxZ = -1 * INFINITY;
    float minX = INFINITY;
    float minY = INFINITY;
    float minZ = INFINITY;
    // iterate through the spheres to find the max and min x, y, z
    for (int i = 0; i < nSpheres; i++)
    {
        float spherex = spheres[i]->getCenter().get_pos(0, 0);
        float spherey = spheres[i]->getCenter().get_pos(1, 0);
        float spherez = spheres[i]->getCenter().get_pos(2, 0);
        float sphereRadius = spheres[i]->getRadius();
        /* find upper bound*/
        if (spherex + sphereRadius > maxX)
        {
            maxX = spherex + sphereRadius;
        }
        if (spherey + sphereRadius > maxY)
        {
            maxY = spherey + sphereRadius;
        }
        if (spherez + sphereRadius > maxZ)
        {
            maxZ = spherez + sphereRadius;
        }
        /* find lower bound*/
        if (spherex - sphereRadius < minX)
        {
            minX = spherex - sphereRadius;
        }
        if (spherey - sphereRadius < minY)
        {
            minY = spherey - sphereRadius;
        }
        if (spherez - sphereRadius < minZ)
        {
            minZ = spherez - sphereRadius;
        }
    }
    UB = Matrix(maxX, maxY, maxZ);
    LB = Matrix(minX, minY, minZ);
}

void BoundingBoxTreeNode::splitSort(int &nnn, int &npn, int &npp, int &nnp, int &pnn, int &ppn, int &ppp, int &pnp)
{
    int x, y, z;
    int count[2][2][2] = {};

    // temporary vector to store the spheres corresponding to each subtree.
    std::vector<BoundingSphere *> nnn_spheres;
    std::vector<BoundingSphere *> npn_spheres;
    std::vector<BoundingSphere *> npp_spheres;
    std::vector<BoundingSphere *> nnp_spheres;
    std::vector<BoundingSphere *> pnn_spheres;
    std::vector<BoundingSphere *> ppn_spheres;
    std::vector<BoundingSphere *> pnp_spheres;
    std::vector<BoundingSphere *> ppp_spheres;

    // a map to easily find the corresponding vectors in the loop below.
    std::map<std::tuple<int, int, int>, std::vector<BoundingSphere *> *> vector_map({
        {std::make_tuple(0, 0, 0), &nnn_spheres},
        {std::make_tuple(0, 0, 1), &nnp_spheres},
        {std::make_tuple(0, 1, 0), &npn_spheres},
        {std::make_tuple(0, 1, 1), &npp_spheres},
        {std::make_tuple(1, 0, 0), &pnn_spheres},
        {std::make_tuple(1, 0, 1), &pnp_spheres},
        {std::make_tuple(1, 1, 0), &ppn_spheres},
        {std::make_tuple(1, 1, 1), &ppp_spheres},
    });
    
    // for all spheres, check which substree it belong and add it to corresponding vector.
    for (int i = 0; i < nSpheres; i++)
    {
        float spherex = spheres[i]->getCenter().get_pos(0, 0);
        float spherey = spheres[i]->getCenter().get_pos(1, 0);
        float spherez = spheres[i]->getCenter().get_pos(2, 0);
        x = (spherex < center.get_pos(0, 0)) ? 0 : 1;
        y = (spherey < center.get_pos(1, 0)) ? 0 : 1;
        z = (spherez < center.get_pos(2, 0)) ? 0 : 1;
        vector_map[std::make_tuple(x, y, z)]->push_back(spheres[i]);
        count[x][y][z]++;
    }
    // store the count bakc in the int references.
    nnn = count[0][0][0];
    nnp = count[0][0][1];
    npn = count[0][1][0];
    npp = count[0][1][1];
    pnn = count[1][0][0];
    pnp = count[1][0][1];
    ppn = count[1][1][0];
    ppp = count[1][1][1];
    std::vector<std::tuple<int, int, int>> order({std::make_tuple(0, 0, 0), std::make_tuple(0, 1, 0),
                                                  std::make_tuple(0, 1, 1), std::make_tuple(0, 0, 1),
                                                  std::make_tuple(1, 0, 0), std::make_tuple(1, 1, 0),
                                                  std::make_tuple(1, 1, 1), std::make_tuple(1, 0, 1)});
    int sum = 0;
    // copy the spheres stored in vector back to the sphere array.
    for (std::tuple<int, int, int> idx : order)
    {
        for (BoundingSphere *ele : *(vector_map[idx]))
        {
            spheres[sum] = ele;
            sum++;
        }
    }
}
