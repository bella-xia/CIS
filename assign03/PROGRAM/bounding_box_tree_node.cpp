#include "bounding_box_tree_node.h"
    BoundingBoxTreeNode::BoundingBoxTreeNode(BoundingSphere** BS, int sphereNum){
        spheres = BS;
        nSpheres = sphereNum;
        calculateCenter();
        calculateMaxRadius();
        calcultateMaxMinCoordinate();
        constructSubtrees();
    }
    void BoundingBoxTreeNode::constructSubtrees(){
        if(nSpheres <= minCount || (UB - LB).magnitude() <- minDiag) {
            haveSubtrees = false;
            return;
        }
        haveSubtrees = true;
        int nnn, npn, npp, nnp, pnn, ppn, ppp, pnp;
        splitSort(nnn, npn, npp, nnp, pnn, ppn, ppp, pnp);
        BoundingSphere** ptr = spheres;
        subtrees[0][0][0] = new BoundingBoxTreeNode(ptr,nnn);
        ptr += nnn;
        subtrees[0][1][0] = new BoundingBoxTreeNode(ptr,npn);
        ptr += npn;
        subtrees[0][1][1] = new BoundingBoxTreeNode(ptr,npp);
        ptr += npp;
        subtrees[0][0][1] = new BoundingBoxTreeNode(ptr,nnp);
        ptr += nnp;
        subtrees[1][0][0] = new BoundingBoxTreeNode(ptr,pnn);
        ptr += pnn;
        subtrees[1][1][0] = new BoundingBoxTreeNode(ptr,ppn);
        ptr += ppn;
        subtrees[1][1][1] = new BoundingBoxTreeNode(ptr,ppp);
        ptr += ppp;
        subtrees[1][0][1] = new BoundingBoxTreeNode(ptr,pnp);
    }

    void BoundingBoxTreeNode::findClosestPoint(Matrix v, float &bound, Matrix &closest){
        float dist = bound + maxRadius;
        float vx = v.get_pos(0,0);
        float vy = v.get_pos(1,0);
        float vz = v.get_pos(2,0);
        if (vx > UB.get_pos(0,0) + dist || vy > UB.get_pos(1,0) + dist || vz > UB.get_pos(2,0) + dist ||
            vx < LB.get_pos(0,0) - dist || vy < LB.get_pos(1,0) - dist || vz < LB.get_pos(2,0) - dist) {
                return;
            }
        if(haveSubtrees) {
            subtrees[0][0][0]->findClosestPoint(v,bound,closest);
            subtrees[0][0][1]->findClosestPoint(v,bound,closest);
            subtrees[0][1][0]->findClosestPoint(v,bound,closest);
            subtrees[0][1][1]->findClosestPoint(v,bound,closest);
            subtrees[1][0][0]->findClosestPoint(v,bound,closest);
            subtrees[1][0][1]->findClosestPoint(v,bound,closest);
            subtrees[1][1][0]->findClosestPoint(v,bound,closest);
            subtrees[1][1][1]->findClosestPoint(v,bound,closest);
        } else {
            for (int i = 0; i < nSpheres; i++){
                if (spheres[i]->mightBeCloser(v, bound)){
                    std::tuple<float, Matrix> cur = spheres[i]->getTriangle().find_closest_point_in_triangle(v);
                    float cur_dis = std::get<0>(cur);
                    if (cur_dis < bound) {
                        bound = cur_dis;
                        closest = std::get<1>(cur);
                    }
                }
            }
        }

    }

    void BoundingBoxTreeNode::calculateCenter(){
        //center coordinate
        float x;
        float y;
        float z;
        for(int i = 0; i < nSpheres; i++) {
            //calculate center
            x += spheres[i]->getCenter().get_pos(0,0);
            y += spheres[i]->getCenter().get_pos(1,0);
            z += spheres[i]->getCenter().get_pos(2,0);
        }
        x /= nSpheres;
        y /= nSpheres;
        z /= nSpheres;
        center = Matrix(x, y, z);
    }

    void BoundingBoxTreeNode::calculateMaxRadius(){
        float maxR = 0;
        for(int i = 0; i < nSpheres; i++) {
            //calculate center
            float sphereRadius = spheres[i]->getRadius();
            if(sphereRadius > maxR) {
                maxR = sphereRadius;
            }
        }
        maxRadius = maxR;
    }

    void BoundingBoxTreeNode::calcultateMaxMinCoordinate(){
        float maxX = -1 * INFINITY;
        float maxY = -1 * INFINITY;
        float maxZ = -1 * INFINITY;
        float minX = INFINITY;
        float minY = INFINITY;
        float minZ = INFINITY;
        for(int i = 0; i < nSpheres; i++) {
            //calculate center
            float spherex = spheres[i]->getCenter().get_pos(0,0);
            float spherey = spheres[i]->getCenter().get_pos(1,0);
            float spherez = spheres[i]->getCenter().get_pos(2,0);
            float sphereRadius = spheres[i]->getRadius();
            /* find upper bound*/
            if(spherex + sphereRadius > maxX) {
                maxX = spherex + sphereRadius;
            }
            if(spherey + sphereRadius > maxY) {
                maxY = spherey + sphereRadius;
            }
            if(spherez + sphereRadius > maxZ) {
                maxZ = spherez + sphereRadius;
            }
            /* find lower bound*/
            if(spherex - sphereRadius < minX) {
                minX = spherex - sphereRadius;
            }
            if(spherey - sphereRadius < minY) {
                minY = spherey - sphereRadius;
            }
            if(spherez - sphereRadius < minZ) {
                minZ = spherez - sphereRadius;
            }
        }
        UB = Matrix(maxX, maxY, maxZ);
        LB = Matrix(minX, minY, minZ);
    }

    void BoundingBoxTreeNode::splitSort(int& nnn, int& npn, int& npp, int& nnp, int& pnn, int& ppn, int& ppp, int& pnp){
        int x, y, z;
        int count[2][2][2];
        for(int i = 0; i < nSpheres; i++) {
            float spherex = spheres[i]->getCenter().get_pos(0, 0);
            float spherey = spheres[i]->getCenter().get_pos(1, 0);
            float spherez = spheres[i]->getCenter().get_pos(2, 0);
            x = (spherex < center.get_pos(0, 0))? 0 : 1;
            y = (spherey < center.get_pos(1, 0))? 0 : 1;
            z = (spherez < center.get_pos(2, 0))? 0 : 1;
            count[x][y][z]++;
        }
        nnn = count[0][0][0];
        nnp = count[0][0][1];
        npn = count[0][1][0];
        npp = count[0][1][1];
        pnn = count[1][0][0];
        pnp = count[1][0][1];
        ppn = count[1][1][0];
        ppp = count[1][1][1];

        int index[2][2][2];
        int fill_in[2][2][2];
        fill_in[0][1][0] = nnn;
        fill_in[0][1][1] = fill_in[0][1][0] + npn;
        fill_in[0][0][1] = fill_in[0][1][1] + npp;
        fill_in[1][0][0] = fill_in[0][0][1] + nnp;
        fill_in[1][1][0] = fill_in[1][0][0] + pnn;
        fill_in[1][1][1] = fill_in[1][1][0] + ppn;
        fill_in[1][0][1] = fill_in[1][1][1] + ppp;
        int sum = 0;
        BoundingSphere* cur = spheres[0];
        BoundingSphere* copy;
        while (sum < nSpheres) {
            float spherex = cur ->getCenter().get_pos(0, 0);
            float spherey = cur->getCenter().get_pos(1, 0);
            float spherez = cur->getCenter().get_pos(2, 0);
            x = (spherex < center.get_pos(0, 0))? 0 : 1;
            y = (spherey < center.get_pos(1, 0))? 0 : 1;
            z = (spherez < center.get_pos(2, 0))? 0 : 1;
            copy = cur;
            cur = spheres[index[x][y][z] + fill_in[x][y][z]];
            spheres[index[x][y][z] + fill_in[x][y][z]] = copy;
            index[x][y][z]++;
            sum++;
        }
    }
