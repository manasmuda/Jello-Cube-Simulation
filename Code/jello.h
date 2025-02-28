/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _JELLO_H_
#define _JELLO_H_

#include "openGL-headers.h"
#include "pic.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include<iostream>
#include <array>
#include "StringUtils.h"

#define pi 3.141592653589793238462643383279 

#define ENABLE_LOGS 0

// camera angles
extern double Theta;
extern double Phi;
extern double R;

// number of images saved to disk so far
extern int sprite;

// mouse control
extern int g_vMousePos[2];
extern int g_vMouseDelta[2];
extern int g_iLeftMouseButtonDown,g_iMiddleMouseButtonDown,g_iRightMouseButtonDown;
extern int g_iLeftMouseButtonStarted, g_iMiddleMouseButtonStarted, g_iRightMouseButtonStarted;
extern int g_iLeftMouseButtonEnded, g_iMiddleMouseButtonEnded, g_iRightMouseButtonEnded;



#define IsPointValid(obj, i, j, k)\
   (i >= 0 && i < obj->w) && (j >= 0 && j < obj->b) && (k >= 0 && k < obj->h)\

// computes crossproduct of two vectors, which are specified as points, and stores the result into dest
// struct point vector1, vector2, dest
// result goes into dest
#define CROSSPRODUCTp(vector1,vector2,dest)\
  CROSSPRODUCT( (vector1).x, (vector1).y, (vector1).z,\
                (vector2).x, (vector2).y, (vector2).z,\
                (dest).x, (dest).y, (dest).z )

// computes crossproduct of two vectors, which are specified by floating-point coordinates, and stores the result into x,y,z
// double x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
  x = (y1) * (z2) - (y2) * (z1);\
  y = (x2) * (z1) - (x1) * (z2);\
  z = (x1) * (y2) - (x2) * (y1)

#define DOTPRODUCT(v1,v2)\
v1.x*v2.x+v1.y*v2.y+v1.z*v2.z\

//  vector magnitude
#define pMAGNITUDE(dest)\
  sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z)\

//  points distance
#define pDISTANCE(source,dest, dist)\
  point pDISTANCETEMPVECTOR;\
  pDIFFERENCE(source, dest, pDISTANCETEMPVECTOR)\
  dist = pMAGNITUDE(pDISTANCETEMPVECTOR);


// normalizes vector dest
// struct point dest
// result returned in dest
// must declare a double variable called 'length' somewhere inside the scope of the NORMALIZE macrp
// macro will change that variable
#define pNORMALIZE(dest)\
\
  length = pMAGNITUDE(dest);\
  (dest).x /= length;\
  (dest).y /= length;\
  (dest).z /= length;

#define pNORMALIZED(src, dest)\
\
  double length_##src = pMAGNITUDE(src);\
  (dest).x = (src).x / length_##src;\
  (dest).y = (src).y / length_##src;\
  (dest).z = (src).z / length_##src;

// copies vector source to vector dest
// struct point source,dest
#define pCPY(source,dest)\
\
  (dest).x = (source).x;\
  (dest).y = (source).y;\
  (dest).z = (source).z;

// assigns values x,y,z to point vector dest
// struct point dest
// double x,y,z
#define pMAKE(x1,y1,z1,dest)\
\
  (dest).x = (x1);\
  (dest).y = (y1);\
  (dest).z = (z1);

// sums points src1 and src2 to dest
// struct point src1,src2,dest
#define pSUM(src1,src2,dest)\
\
  (dest).x = (src1).x + (src2).x;\
  (dest).y = (src1).y + (src2).y;\
  (dest).z = (src1).z + (src2).z;

// dest = src1 - src2
// struct point src1,src2,dest
#define pDIFFERENCE(src1,src2,dest)\
\
  (dest).x = (src1).x - (src2).x;\
  (dest).y = (src1).y - (src2).y;\
  (dest).z = (src1).z - (src2).z;

// mulitplies components of point src by scalar and returns the result in dest
// struct point src,dest
// double scalar
#define pMULTIPLY(src,scalar,dest)\
\
  (dest).x = (src).x * (scalar);\
  (dest).y = (src).y * (scalar);\
  (dest).z = (src).z * (scalar);


#define pVECMULTIPLY(src1,src2,dest)\
\
  (dest).x = (src1).x * (src2).x;\
  (dest).y = (src1).y * (src2).y;\
  (dest).z = (src1).z * (src2).z;

#define CREATE_3D_POINTER(i,j,k, p)\
  point*** p;\
  p = new point * *[i];\
    for (int firstDimensionNumberTracker = 0; firstDimensionNumberTracker < i; firstDimensionNumberTracker++) {\
        p[firstDimensionNumberTracker] = new point * [j];\
        for (int secondDimensionNumberTracker = 0; secondDimensionNumberTracker < j; secondDimensionNumberTracker++) {\
            p[firstDimensionNumberTracker][secondDimensionNumberTracker] = new point[j];\
        }\
    }

#define DELETE_3D_POINTER(i,j,k, p)\
for (int firstDimensionNumberTracker = 0; firstDimensionNumberTracker < i; firstDimensionNumberTracker++) {\
for (int secondDimensionNumberTracker = 0; secondDimensionNumberTracker < j; secondDimensionNumberTracker++) {\
    delete[] p[firstDimensionNumberTracker][secondDimensionNumberTracker];\
}\
delete[] p[firstDimensionNumberTracker];\
}\
delete[] p;\

#define ARE_ALMOSTEQUAL(n1, n2)\
    std::fabs(n1 - n2) < 0.0001\

struct point 
{
   double x;
   double y;
   double z;

   point() :x(0), y(0), z(0) {}
   point(double x, double y, double z) : x(x), y(y), z(z) {}
   point(const point& p) : x(p.x), y(p.y), z(p.z) {}
};

struct Hit{
    point* hitPoint;
    point forward;

    Hit(): hitPoint(nullptr){}

    bool isValid() {
        return hitPoint != nullptr;
    }
};

// these variables control what is displayed on the screen
extern int shear, bend, structural, pause, viewingMode, saveScreenToFile, transparentJello;

struct world
{

    struct object {

        enum Face {
            BOTTOM = 1,
            TOP = 6,
            FRONT = 2,
            BACK = 5,
            LEFT = 3,
            RIGHT = 4
        };

        char id[5];
        double kElastic; // Hook's elasticity coefficient for all springs except collision springs
        double dElastic; // Damping coefficient for all springs except collision springs
        double kCollision; // Hook's elasticity coefficient for collision springs
        double dCollision; // Damping coefficient collision springs
        double mass; // mass of each of the 512 control points, mass assumed to be equal for every control point
        struct point*** p; // position of the 512 control points
        struct point*** v; // velocities of the 512 control points
        int w, b, h;
        double width, breadth, height;

        object() : w(0), b(0), h(0), p(nullptr), v(nullptr) {
        }

        object(const object& other) {
            strncpy(id, other.id, sizeof(id));
            kElastic = other.kElastic;
            dElastic = other.dElastic;
            kCollision = other.kCollision;
            dCollision = other.dCollision;
            w = other.w;
            b = other.b;
            h = other.h;
            width = other.width;
            breadth = other.breadth;
            height = other.height;
            mass = other.mass;

            // Allocate and copy p
            p = new point * *[w];
            for (int i = 0; i < w; i++) {
                p[i] = new point * [b];
                for (int j = 0; j < b; j++) {
                    p[i][j] = new point[h];
                    for (int k = 0; k < h; k++) {
                        p[i][j][k] = other.p[i][j][k];  // Copy each point
                    }
                }
            }

            // Allocate and copy v
            v = new point * *[w];
            for (int i = 0; i < w; i++) {
                v[i] = new point * [b];
                for (int j = 0; j < b; j++) {
                    v[i][j] = new point[h];
                    for (int k = 0; k < h; k++) {
                        v[i][j][k] = other.v[i][j][k];  // Copy each velocity
                    }
                }
            }
        }

        // Destructor to free memory
        ~object() {
            if (p) {
                for (int i = 0; i < w; i++) {
                    for (int j = 0; j < b; j++) {
                        delete[] p[i][j];
                    }
                    delete[] p[i];
                }
                delete[] p;
            }

            if (v) {
                for (int i = 0; i < w; i++) {
                    for (int j = 0; j < b; j++) {
                        delete[] v[i][j];
                    }
                    delete[] v[i];
                }
                delete[] v;
            }
        }

        object& operator=(const object& other) {
            if (this == &other) {
                return *this; // Self-assignment check
            }

            // Now perform the deep copy
            strncpy(id, other.id, sizeof(id));
            kElastic = other.kElastic;
            dElastic = other.dElastic;
            kCollision = other.kCollision;
            dCollision = other.dCollision;
            w = other.w;
            b = other.b;
            h = other.h;
            width = other.width;
            breadth = other.breadth;
            height = other.height;
            mass = other.mass;

            // Allocate memory for p and v and copy data
            p = new point * *[w];
            for (int i = 0; i < w; i++) {
                p[i] = new point * [b];
                for (int j = 0; j < b; j++) {
                    p[i][j] = new point[h];
                    for (int k = 0; k < h; k++) {
                        p[i][j][k] = other.p[i][j][k];
                    }
                }
            }

            v = new point * *[w];
            for (int i = 0; i < w; i++) {
                v[i] = new point * [b];
                for (int j = 0; j < b; j++) {
                    v[i][j] = new point[h];
                    for (int k = 0; k < h; k++) {
                        v[i][j][k] = other.v[i][j][k];
                    }
                }
            }

            return *this;
        }

        bool isValid() const{
            return w > 0 && b > 0 && h > 0;
        }

        double getWSpringLength() const{
            return width / (w - 1);
        }

        double getBSpringLength() const{
            return breadth / (b - 1);
        }

        double getHSpringLength() const{
            return height / (h - 1);
        }

        point getCenter() const {
            point center;
            center.x = (p[0][0][0].x + p[w - 1][0][0].x + p[0][0][h-1].x + p[w-1][0][h-1].x + p[0][b-1][0].x + p[w - 1][b-1][0].x + p[0][b-1][h - 1].x + p[w - 1][b-1][h - 1].x)/8.0;
            center.y = (p[0][0][0].y + p[w - 1][0][0].y + p[0][0][h-1].y + p[w-1][0][h-1].y + p[0][b-1][0].y + p[w - 1][b-1][0].y + p[0][b-1][h - 1].y + p[w - 1][b-1][h - 1].y)/8.0;
            center.z = (p[0][0][0].z + p[w - 1][0][0].z + p[0][0][h-1].z + p[w-1][0][h-1].z + p[0][b-1][0].z + p[w - 1][b-1][0].z + p[0][b-1][h - 1].z + p[w - 1][b-1][h - 1].z)/8.0;
            return center;
        }

        point getFaceCenter(Face face) {
            std::array<point, 4> faceCorners = getFaceCorners(face);
            point center;
            for (int i = 0; i < 4; i++)
            {
                pSUM(center, faceCorners[i], center);
            }
            pMULTIPLY(center, 0.25, center)
            return center;
        }

        std::array<point, 4> getFaceCorners(Face face) {
            std::array<point, 4> faceCorners;
            switch (face) {
                case BOTTOM:
                    faceCorners = { p[0][0][0], p[w - 1][0][0], p[0][b - 1][0], p[w - 1][b - 1][0] };
                    break;
                case TOP:
                    faceCorners = { p[0][0][h-1], p[w - 1][0][h-1], p[0][b - 1][h-1], p[w - 1][b - 1][h-1] };
                    break;
                case FRONT:
                    faceCorners = { p[0][0][0], p[w - 1][0][0],  p[0][0][h - 1], p[w - 1][0][h - 1] };
                    break;
                case BACK:
                    faceCorners = { p[0][b - 1][0], p[w - 1][b - 1][0], p[0][b - 1][h - 1], p[w - 1][b - 1][h - 1] };
                    break;
                case LEFT:
                    faceCorners = { p[0][0][0], p[0][b - 1][0], p[0][0][h - 1], p[0][b - 1][h - 1] };
                    break;
                case RIGHT:
                    faceCorners = { p[w-1][0][0], p[w-1][b - 1][0], p[w-1][0][h - 1], p[w-1][b - 1][h - 1] };
                    break;
            }
            return faceCorners;
        }
        
        std::array<point, 2> getIndexRange(Face face) {
            std::array<point, 2> faceRange;
            switch (face) {
                case BOTTOM:
                    faceRange = { point(0,0,0), point(w-1, b-1, h-1)};
                    break;
                case TOP:
                    faceRange = { point(0,0,h-1), point(w - 1, b - 1, h-1) };
                    break;
                case FRONT:
                    faceRange = { point(0,0,0), point(w - 1, 0, h-1) };
                    break;
                case BACK:
                    faceRange = { point(0,b-1,0), point(w - 1, b-1, h - 1) };
                    break;
                case LEFT:
                    faceRange = { point(0,0,0), point(0, b - 1, h - 1) };
                    break;
                case RIGHT:
                    faceRange = { point(w-1,0,0), point(w-1, b - 1, h - 1) };
                    break;
                }
            return faceRange;
        }
    };

    char integrator[10]; // "RK4" or "Euler"
    double dt; // timestep, e.g.. 0.001
    int n; // display only every nth timestep

    int incPlanePresent; // Is the inclined plane present? 1 = YES, 0 = NO
    double a, b, c, d; // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
    int resolution; // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
    point* forceField; // pointer to the array of values of the force field

    int objectCount;
    object* objects;

    double bbw, bbb, bbh;

    point getForceFieldAtPoint(int i, int j, int k) {
        return forceField[resolution * resolution * i + resolution * j + k];
    }

};

extern struct world jello;




#endif



