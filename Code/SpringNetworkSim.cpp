#include "SpringNetworkSim.h"
#include <cmath>


SpringNetworkSim::SpringNetworkSim(world* world):curWorld(world) {
    this->deltaTime = 0;
    this->lastTime = steady_clock::now();
    this->orgWorld = *world;
}

Hit SpringNetworkSim::getRayCastPoint(point rayOrigin, point rayDir) {
    Hit curHit;
    point rayHit = point(rayOrigin);
    double minT = 1000000;
    bool rayHitValidPoint = false;
    for (int oc = 0; oc < curWorld->objectCount; oc++) {
        world::object* curObject = &curWorld->objects[oc];
        point objCenter = curObject->getCenter();
        
        for (int fn = 1; fn <= 6; fn++) {
            world::object::Face curFace = (world::object::Face)fn;
            
            point faceCenter = curObject->getFaceCenter(curFace);
            point faceNormal;
            pDIFFERENCE(faceCenter, objCenter, faceNormal)
            point pointOriginDiff;
            pDIFFERENCE(faceCenter, rayOrigin, pointOriginDiff)
            double normPointOrginDiffDot = DOTPRODUCT(pointOriginDiff, faceNormal);
            double normDirDot = DOTPRODUCT(faceNormal, rayDir);
            double t = normPointOrginDiffDot / normDirDot;
            point intersectPoint;
            if (t > 0) {
                pMULTIPLY(rayDir, t, intersectPoint)
                pSUM(rayOrigin, intersectPoint, intersectPoint)
                double minFX = 100000, minFY = 100000, minFZ = 100000;
                double maxFX = -100000, maxFY = -100000, maxFZ = -100000;
                std::array<point, 4> corners = curObject->getFaceCorners(curFace);
                for (int cn = 0; cn < 4; cn++) {
                    point currentCorner = corners[cn];
                    minFX = min(minFX, currentCorner.x);
                    minFY = min(minFY, currentCorner.y);
                    minFZ = min(minFZ, currentCorner.z);
                    maxFX = max(maxFX, currentCorner.x);
                    maxFY = max(maxFY, currentCorner.y);
                    maxFZ = max(maxFZ, currentCorner.z);
                }
                if (minFX <= intersectPoint.x && intersectPoint.x <= maxFX && minFY <= intersectPoint.y && intersectPoint.y <= maxFY && minFZ <= intersectPoint.z && intersectPoint.z <= maxFZ) {
                    rayHitValidPoint = true;
                    if (t < minT) {
                        minT = t;
                        //interpolate based on w,b ot h and length and
                        rayHit = intersectPoint;
                        std::array<point, 2> indexRange = curObject->getIndexRange(curFace);
                        double minDistance = 999999;
                        for (int fiX = indexRange[0].x; fiX <= indexRange[1].x; fiX++) {
                            for (int fiY = indexRange[0].y; fiY <= indexRange[1].y; fiY++) {
                                for (int fiZ = indexRange[0].z; fiZ <= indexRange[1].z; fiZ++) {
                                    double curDistance;
                                    pDISTANCE(intersectPoint, curObject->p[fiX][fiY][fiZ], curDistance)
                                    if (curDistance < minDistance) {
                                        minDistance = curDistance;
                                        curHit.hitPoint = &curObject->p[fiX][fiY][fiZ];
                                        curHit.forward = faceNormal;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (ENABLE_LOGS) {
        printf("Intersect Point: (%lf, %lf, %lf)\n", rayHit.x, rayHit.y, rayHit.z);
        if (curHit.isValid()) {
            printf("Current Point: (%lf, %lf, %lf)\n", curHit.hitPoint->x, curHit.hitPoint->y, curHit.hitPoint->z);
        }
    }
    return curHit;
}

#pragma region Time Handling

void SpringNetworkSim::updateTime() {
	steady_clock::time_point currentTime = steady_clock::now();
	deltaTime = duration_cast<milliseconds>(currentTime - lastTime).count();
	lastTime = currentTime;
}

#pragma endregion

/*
#pragma region Object Handling

void SpringNetworkSim::addObject(SpringNetworkObject* obj) {
	if (!this->hasObject(obj->id)) {
		this->objMap.insert({ obj->id, obj });
	}
}

bool SpringNetworkSim::removeObject(SpringNetworkObject* obj) {
	return this->removeObject(obj->id);
}

bool SpringNetworkSim::removeObject(string id) {
	return this->objMap.erase(id);
}

SpringNetworkObject* SpringNetworkSim::getObject(string id) {
	if (this->hasObject(id)) {
		return this->objMap[id];
	}
	else {
		return nullptr;
	}
}

bool SpringNetworkSim::hasObject(string id) {
	if (this->objMap.find(id) != this->objMap.end()) {
		return true;
	}
	return false;
}
#pragma endregion

*/

#pragma region Physics Simulation


void SpringNetworkSim::updateSim() {
    for (int oc = 0; oc < curWorld->objectCount; oc++)
    {
        world::object* obj = &curWorld->objects[oc];

        if (strcmp(curWorld->integrator, "Euler") == 0) {
            Euler(obj);
        }
        else {
            RK4(obj);
        }
       

        /*if (ENABLE_LOGS) {

            printf("POSITION:\n");
            for (int i = 0; i < obj->w; i++)
            {
                for (int j = 0; j < obj->b; j++)
                {
                    for (int k = 0; k < obj->h; k++)
                    {
                        printf("(%lf,%lf,%lf), ", obj->p[i][j][k].x, obj->p[i][j][k].y, obj->p[i][j][k].z);
                    }
                    printf("\n");
                }
                printf("-------------\n");
            }
            printf("\n");

            printf("VELOCITY:\n");
            for (int i = 0; i < obj->w; i++)
            {
                for (int j = 0; j < obj->b; j++)
                {
                    for (int k = 0; k < obj->h; k++)
                    {
                        printf("(%lf,%lf,%lf), ", obj->v[i][j][k].x, obj->v[i][j][k].y, obj->v[i][j][k].z);
                    }
                    printf("\n");
                }
                printf("-------------\n");
            }
            printf("\n");


            printf("\n");
            printf("\n");
        }*/
    }
}

/* Computes acceleration to every control point of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void SpringNetworkSim::computeAcceleration(struct world::object* obj, struct point***& a)
{
    /* for you to implement ... */
    for (int i = 0; i < obj->w; i++) {
        for (int j = 0; j < obj->b; j++) {
            for (int k = 0; k < obj->h; k++) {
                point force = getForceOnPoint(obj, i, j, k);
                pMULTIPLY(force, 1.0/obj->mass, a[i][j][k])
            }
        }
    }

    if (ENABLE_LOGS) {

        printf("ACCELERATION:\n");

        for (int i = 0; i < obj->w; i++)
        {
            for (int j = 0; j < obj->b; j++)
            {
                for (int k = 0; k < obj->h; k++)
                {
                    printf("(%lf,%lf,%lf), ", a[i][j][k].x, a[i][j][k].y, a[i][j][k].z);
                }
                printf("\n");
            }
            printf("-------------\n");
        }

        printf("\n");

    }

}

point SpringNetworkSim::getForceOnPoint(world::object* obj, int i, int j, int k) {
    point force;
    
    if (IsPointValid(obj, i, j, k)) {
        point curPoint = obj->p[i][j][k];

        int ip, jp, kp;
        point neighborPoint, pointsDiff;
        double curLength;
        

#define GET_SPRING_FORCE(di, dj, dk, sf) \
        double orgLength = std::sqrt(di*di*obj->getWSpringLength()*obj->getWSpringLength() + dj*dj*obj->getBSpringLength()*obj->getBSpringLength() + dk*dk*obj->getHSpringLength()*obj->getHSpringLength());\
        double tempValue = 0;\
        if(!(ARE_ALMOSTEQUAL(orgLength, curLength))){\
            tempValue =  (obj->kElastic * (orgLength - curLength)/curLength); \
            pMULTIPLY( pointsDiff, tempValue,sf)\
        }\
        else{\
            sf = point();\
        }\

#define GET_DAMPING_FORCE(df) \
        point curPointV = obj->v[i][j][k];\
        point neighborPointV = obj->v[ip][jp][kp];\
        point velocitiesDiff;\
        pDIFFERENCE(curPointV, neighborPointV, velocitiesDiff)\
        double cpd = (-1*obj->dElastic *(DOTPRODUCT(velocitiesDiff, pointsDiff)))/(curLength*curLength);\
        pMULTIPLY( pointsDiff, cpd, df)\


        

#define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if(IsPointValid(obj, ip, jp, kp))\
    {\
        neighborPoint = obj->p[ip][jp][kp];\
        pDIFFERENCE(curPoint, neighborPoint, pointsDiff)\
        double curLength = pMAGNITUDE(pointsDiff);\
        point springForce, dampingForce;\
        GET_SPRING_FORCE(di, dj, dk, springForce)\
        GET_DAMPING_FORCE(dampingForce)\
        pSUM(force, springForce, force)\
        pSUM(force, dampingForce, force)\
    }\

        for (int bbi = -1; bbi <= 1; bbi++) {
            for (int bbj = -1; bbj <= 1; bbj++) {
                for (int bbk = -1; bbk <= 1; bbk++) {
                    if (!(bbi == 0 && bbj == 0 && bbk == 0)) {
                        PROCESS_NEIGHBOUR(bbi, bbj, bbk)
                    }
                }
            }
        }

        PROCESS_NEIGHBOUR(2, 0, 0)
        PROCESS_NEIGHBOUR(-2, 0, 0)
        PROCESS_NEIGHBOUR(0, 2, 0)
        PROCESS_NEIGHBOUR(0, -2, 0)
        PROCESS_NEIGHBOUR(0, 0, 2)
        PROCESS_NEIGHBOUR(0, 0, -2)

        point forceFieldF = getForceFieldAtPoint(curPoint);
        pSUM(force, forceFieldF, force)

        CollissionData collissionData = getCollissionDataWithBoundingBox(obj, obj->p[i][j][k]);
        if (collissionData.isValid()) {
            point collisionForce;
            double tempValue = (obj->kCollision * (0 - collissionData.length));
            pMULTIPLY(collissionData.normal, tempValue, collisionForce)
            double cpd = (-1 * obj->dCollision * (DOTPRODUCT(collissionData.normal, obj->v[i][j][k])));
            point collissionDampingForce;
            pMULTIPLY(collissionData.normal, cpd, collissionDampingForce)
            pSUM(collisionForce, collissionDampingForce, force)
        }

        for (int oc = 0; oc < curWorld->objectCount; oc++)
        {
            world::object* collidingObj = &curWorld->objects[oc];
            if (strcmp(collidingObj->id, obj->id) != 0) {
                CollissionData collissionData = getCollissionDataWithObject(obj, collidingObj, obj->p[i][j][k]);
                if (collissionData.isValid()) {
                    //double collisionK = 2 * sqrt(obj->kCollision * obj->kCollision + collidingObj->kCollision * collidingObj->kCollision);
                    //double collisionD = 2 * sqrt(obj->dCollision * obj->dCollision + collidingObj->dCollision * collidingObj->dCollision);
                    double collisionK = (obj->kCollision + collidingObj->kCollision) / 2.0;
                    double collisionD = (obj->dCollision + collidingObj->dCollision) / 2.0;
                    point collisionForce;
                    double tempValue = (collisionK * (0 - collissionData.length));
                    pMULTIPLY(collissionData.normal, tempValue, collisionForce)
                    double cpd = (-1 * collisionD * (DOTPRODUCT(collissionData.normal, obj->v[i][j][k])));
                    point collissionDampingForce;
                    pMULTIPLY(collissionData.normal, cpd, collissionDampingForce)
                    pSUM(collisionForce, collissionDampingForce, force)
                }
            }
        }

    }
    return force;
}

point SpringNetworkSim::getForceFieldAtPoint(point position) {
    double u = ((position.x + curWorld->bbw / 2.0) / curWorld->bbw) * (curWorld->resolution - 1);
    double v = ((position.y + curWorld->bbb / 2.0) / curWorld->bbb) * (curWorld->resolution - 1);
    double w = ((position.z + curWorld->bbh / 2.0) / curWorld->bbh) * (curWorld->resolution - 1);

    u = max(0.0, u);
    v = max(0.0, v);
    w = max(0.0, w);

    u = min(curWorld->resolution - 1, u);
    v = min(curWorld->resolution - 1, v);
    w = min(curWorld->resolution - 1, w);

    int minU = std::floor(u);
    int minV = std::floor(v);
    int minW = std::floor(w);

    minU = min(curWorld->resolution - 2, minU);
    minV = min(curWorld->resolution - 2, minV);
    minW = min(curWorld->resolution - 2, minW);

    int maxU = minU + 1;
    int maxV = minV + 1;
    int maxW = minW + 1;

    u = u - minU;
    v = v - minV;
    w = w - minW;

    point f000 = curWorld->getForceFieldAtPoint(minU, minV, minW);
    point f001 = curWorld->getForceFieldAtPoint(minU, minV, maxW);
    point f010 = curWorld->getForceFieldAtPoint(minU, maxV, minW);
    point f011 = curWorld->getForceFieldAtPoint(minU, maxV, maxW);
    point f100 = curWorld->getForceFieldAtPoint(maxU, minV, minW);
    point f101 = curWorld->getForceFieldAtPoint(maxU, minV, maxW);
    point f110 = curWorld->getForceFieldAtPoint(maxU, maxV, minW);
    point f111 = curWorld->getForceFieldAtPoint(maxU, maxV, maxW);

    pMULTIPLY(f000, (1 - u) * (1 - v) * (1 - w), f000)
    pMULTIPLY(f001, (1 - u) * (1 - v) * (w), f001)
    pMULTIPLY(f010, (1 - u) * (v) * (1 - w), f010)
    pMULTIPLY(f011, (1 - u) * (v) * (w), f011)
    pMULTIPLY(f100, (u) * (1 - v) * (1 - w), f100)
    pMULTIPLY(f101, (u) * (1 - v) * (w), f101)
    pMULTIPLY(f110, (u) * (v) * (1 - w), f110)
    pMULTIPLY(f111, (u) * (v) * (w), f111)

    point force;
    pSUM(force, f000, force)
    pSUM(force, f001, force)
    pSUM(force, f010, force)
    pSUM(force, f011, force)
    pSUM(force, f100, force)
    pSUM(force, f101, force)
    pSUM(force, f110, force)
    pSUM(force, f111, force)

    return force;
}

#define CALCULATE_PENETRATION(normal, planePoint, p)\
    normal.x*(p.x - planePoint.x) + normal.y*(p.y - planePoint.y) + normal.z*(p.z - planePoint.z)

SpringNetworkSim::CollissionData SpringNetworkSim::getCollissionDataWithBoundingBox(struct world::object* obj, point p) {
    CollissionData collissionData;

    point bbDim(curWorld->bbw/2.0, curWorld->bbb/2.0, curWorld->bbh/2.0);

    point collissionVector;

    point curFacePoint;
    double penetrationLength;

#define CHECK_ADD_PLANE_COLLISSION(normal)\
     pVECMULTIPLY(normal, bbDim, curFacePoint)\
     penetrationLength = CALCULATE_PENETRATION(normal, curFacePoint, p);\
     if (penetrationLength > 0) {\
         point penetrationNormal;\
         pMULTIPLY(normal, penetrationLength, penetrationNormal)\
         pSUM(collissionVector, penetrationNormal, collissionVector);\
     }

    CHECK_ADD_PLANE_COLLISSION(point(0, 0, -1))
    CHECK_ADD_PLANE_COLLISSION(point(0, 0, 1))
    CHECK_ADD_PLANE_COLLISSION(point(0, -1, 0))
    CHECK_ADD_PLANE_COLLISSION(point(0, 1, 0))
    CHECK_ADD_PLANE_COLLISSION(point(-1, 0, 0))
    CHECK_ADD_PLANE_COLLISSION(point(1, 0, 0))

    collissionData.length = pMAGNITUDE(collissionVector);
    pNORMALIZED(collissionVector, collissionData.normal)

    return collissionData;
}

SpringNetworkSim::CollissionData SpringNetworkSim::getCollissionDataWithObject(struct world::object* obj, struct world::object* collidingObj, point p) {
    CollissionData collissionData;
    point collissionVector;

    point objCenter = obj->getCenter();
    point collidingObjCenter = collidingObj->getCenter();
    point centersVector;
    pDIFFERENCE(objCenter, collidingObjCenter, centersVector)
    bool collided = true;
    for (int fn = 1; fn <= 6; fn++) {
        world::object::Face collidingObjFace = (world::object::Face)fn;
        point collidingObjFaceCenter = collidingObj->getFaceCenter(collidingObjFace);
        point faceNormal;
        double length;
        pDIFFERENCE(collidingObjCenter, collidingObjFaceCenter, faceNormal)
        double faceExtend = pMAGNITUDE(faceNormal);
        pNORMALIZE(faceNormal)
        float facePenetration = CALCULATE_PENETRATION(faceNormal, collidingObjFaceCenter, p);
        if (facePenetration < 0) {
            collided = false;
        }
        else {
            point extendNormal;
            pMULTIPLY(faceNormal, -1, extendNormal)
            if (DOTPRODUCT(centersVector, extendNormal) >= faceExtend) {
                point penetrationNormal;
                pMULTIPLY(faceNormal, facePenetration, penetrationNormal)
                pSUM(collissionVector, penetrationNormal, collissionVector);
            }
        }
    }

    if (collided) {
        collissionData.length = pMAGNITUDE(collissionVector);
        pNORMALIZED(collissionVector, collissionData.normal)
            //printf("Collided\n");
    }

    return collissionData;
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void SpringNetworkSim::Euler(struct world::object* obj)
{
    int i, j, k;
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, a)

    computeAcceleration(obj, a);

    for (i = 0; i < obj->w; i++)
        for (j = 0; j < obj->b; j++)
            for (k = 0; k < obj->h; k++)
            {
                obj->p[i][j][k].x += curWorld->dt * obj->v[i][j][k].x;
                obj->p[i][j][k].y += curWorld->dt * obj->v[i][j][k].y;
                obj->p[i][j][k].z += curWorld->dt * obj->v[i][j][k].z;
                obj->v[i][j][k].x += curWorld->dt * a[i][j][k].x;
                obj->v[i][j][k].y += curWorld->dt * a[i][j][k].y;
                obj->v[i][j][k].z += curWorld->dt * a[i][j][k].z;

            }
    DELETE_3D_POINTER(obj->w, obj->b, obj->h, a)
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void SpringNetworkSim::RK4(struct world::object* obj)
{
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F1p)
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F1v)
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F2p)
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F2v)
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F3p)
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F3v)
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F4p)
    CREATE_3D_POINTER(obj->w, obj->b, obj->h, F4v)

    CREATE_3D_POINTER(obj->w, obj->b, obj->h, a)


    struct world::object buffer;

    int i, j, k;

    buffer = *obj; // make a copy of jello

    computeAcceleration(obj, a);

    for (i = 0; i < obj->w; i++)
        for (j = 0; j < obj->b; j++)
            for (k = 0; k < obj->h; k++)
            {
                pMULTIPLY(obj->v[i][j][k], curWorld->dt, F1p[i][j][k]);
                pMULTIPLY(a[i][j][k], curWorld->dt, F1v[i][j][k]);
                pMULTIPLY(F1p[i][j][k], 0.5, buffer.p[i][j][k]);
                pMULTIPLY(F1v[i][j][k], 0.5, buffer.v[i][j][k]);
                pSUM(obj->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(obj->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);

    for (i = 0; i < obj->w; i++)
        for (j = 0; j < obj->b; j++)
            for (k = 0; k < obj->h; k++)
            {
                // F2p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], curWorld->dt, F2p[i][j][k]);
                // F2v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], curWorld->dt, F2v[i][j][k]);
                pMULTIPLY(F2p[i][j][k], 0.5, buffer.p[i][j][k]);
                pMULTIPLY(F2v[i][j][k], 0.5, buffer.v[i][j][k]);
                pSUM(obj->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(obj->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);

    for (i = 0; i < obj->w; i++)
        for (j = 0; j < obj->b; j++)
            for (k = 0; k < obj->h; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], curWorld->dt, F3p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], curWorld->dt, F3v[i][j][k]);
                pMULTIPLY(F3p[i][j][k], 1.0, buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k], 1.0, buffer.v[i][j][k]);
                pSUM(obj->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(obj->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);


    for (i = 0; i < obj->w; i++)
        for (j = 0; j < obj->b; j++)
            for (k = 0; k < obj->h; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], curWorld->dt, F4p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], curWorld->dt, F4v[i][j][k]);

                pMULTIPLY(F2p[i][j][k], 2, buffer.p[i][j][k]);
                pMULTIPLY(F3p[i][j][k], 2, buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F1p[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F4p[i][j][k], buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], obj->p[i][j][k], obj->p[i][j][k]);

                pMULTIPLY(F2v[i][j][k], 2, buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k], 2, buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F1v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F4v[i][j][k], buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], obj->v[i][j][k], obj->v[i][j][k]);
            }

    DELETE_3D_POINTER(obj->w, obj->b, obj->h, F1p)
        DELETE_3D_POINTER(obj->w, obj->b, obj->h, F1v)
        DELETE_3D_POINTER(obj->w, obj->b, obj->h, F2p)
        DELETE_3D_POINTER(obj->w, obj->b, obj->h, F2v)
        DELETE_3D_POINTER(obj->w, obj->b, obj->h, F3p)
        DELETE_3D_POINTER(obj->w, obj->b, obj->h, F3v)
        DELETE_3D_POINTER(obj->w, obj->b, obj->h, F4p)
        DELETE_3D_POINTER(obj->w, obj->b, obj->h, F4v)

        DELETE_3D_POINTER(obj->w, obj->b, obj->h, a)

    return;
}


#pragma endregion