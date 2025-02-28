/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  createWorld utility to create your own world files

  Note: this utility uses its own copy of writeWorld routine, which is identical to the one
  found in input.cpp . If you need to change that routine, or even the definition of the
  world structure (you don't have to do this unless you decide to do some fancy
  extra credit), you have to update both copies.

*/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include<iostream>
#include <string.h>
#include "StringUtils.h"
#include <cstring>

using namespace std;

struct point 
{
   double x;
   double y;
   double z;

   point() : x(0), y(0), z(0) {}

   point(double x, double y, double z) : x(x), y(y), z(z) {}
};

struct world
{

    struct object {
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
            string newId = generateRandomString(3);
            strncpy(id, newId.c_str(), sizeof(id) - 1);
            id[4] = '\0';
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

        bool isValid() const {
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

    };

  char integrator[10]; // "RK4" or "Euler"
  double dt; // timestep, e.g.. 0.001
  int n; // display only every nth timestep
 
  int incPlanePresent; // Is the inclined plane present? 1 = YES, 0 = NO
  double a,b,c,d; // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
  int resolution; // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
  point * forceField; // pointer to the array of values of the force field

  int objectCount;
  object* objects;

  double bbw, bbb, bbh;

};


/* writes the world parameters to a world file on disk*/
/* fileName = string containing the name of the output world file, ex: jello1.w */
/* function creates the output world file and then fills it corresponding to the contents
   of structure 'jello' */
/* function aborts the program if can't access the file */

/* writes the world parameters to a world file on disk*/
/* fileName = string containing the name of the output world file, ex: jello1.w */
/* function creates the output world file and then fills it corresponding to the contents
   of structure 'jello' */
/* function aborts the program if can't access the file */
void writeWorld(const char * fileName, struct world * jello)
{
  int i,j,k;
  FILE * file;
  
  file = fopen(fileName, "w");
  if (file == NULL) {
    printf ("can't open file\n");
    exit(1);
  }

  /* write integrator algorithm */ 
  fprintf(file,"%s\n",jello->integrator);

  /* write timestep */
  fprintf(file,"%lf %d\n",jello->dt,jello->n);

  

  /* write info about the plane */
  fprintf(file, "%d\n", jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fprintf(file, "%lf %lf %lf %lf\n", jello->a, jello->b, jello->c, jello->d);

  fprintf(file, "%lf %lf %lf\n", jello->bbw, jello->bbb, jello->bbh);

  /* write info about the force field */
  fprintf(file, "%d\n", jello->resolution);
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
          fprintf(file, "%lf %lf %lf\n", 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
  
  fprintf(file, "%d\n", jello->objectCount);

  for (int on = 0; on < jello->objectCount; on++) {
      world::object curObject = jello->objects[on];

      fprintf(file, "%s\n", curObject.id);

      /* write physical parameters */
      fprintf(file, "%lf %lf %lf %lf\n",
          curObject.kElastic, curObject.dElastic, curObject.kCollision, curObject.dCollision);

      /* write mass */
      fprintf(file, "%lf\n", curObject.mass);

      fprintf(file, "%lf %lf %lf\n", curObject.width, curObject.breadth, curObject.height);
      fprintf(file, "%d %d %d\n", curObject.w, curObject.b, curObject.h);

      /* write initial point positions */
      for (i = 0; i < curObject.w; i++)
      {
          for (j = 0; j < curObject.b; j++)
          {
              for (k = 0; k < curObject.h; k++)
                  fprintf(file, "%lf %lf %lf\n",
                      curObject.p[i][j][k].x, curObject.p[i][j][k].y, curObject.p[i][j][k].z);
          }
      }

      /* write initial point velocities */
      for (i = 0; i < curObject.w; i++)
      {
          for (j = 0; j < curObject.b; j++)
          {
              for (k = 0; k < curObject.h; k++)
                  fprintf(file, "%lf %lf %lf\n",
                      curObject.v[i][j][k].x, curObject.v[i][j][k].y, curObject.v[i][j][k].z);
          }
      }
  }

  

  fclose(file);
  
  return;
}


world::object createObject(point center, int w = 8, int b = 8, int h = 8, double width = 1.0, double breadth = 1.0, double height = 1.0, double kElastic = 200.0, double dElastic = 0.25, double kCollision = 400.0, double dCollision = 0.25, double totalMass = 1.0, point initialVelocity = point(10.0, -10.0, 20.0)) {
    world::object obj;

    int i, j, k;

    obj.kElastic = kElastic;
    obj.dElastic = dElastic;
    obj.kCollision = kCollision;
    obj.dCollision = dCollision;
    obj.w = w;
    obj.b = b;
    obj.h = h;
    obj.width = width;
    obj.breadth = breadth;
    obj.height = height;

    obj.mass = totalMass / (obj.w * obj.b * obj.h);

    // set the positions of control points
    obj.p = new point * *[obj.w];
    for (i = 0; i < obj.w; i++) {
        obj.p[i] = new point * [obj.b];
        for (j = 0; j < obj.b; j++) {
            obj.p[i][j] = new point[obj.h];
            for (k = 0; k < obj.h; k++)
            {
                obj.p[i][j][k].x = obj.width * i / (obj.w - 1) + center.x - width / 2.0;
                obj.p[i][j][k].y = obj.breadth * j / (obj.b - 1) + center.y - breadth / 2.0;
                obj.p[i][j][k].z = obj.height * k / ((obj.h - 1)) + center.z - height / 2.0;
            }
        }
    }

    // set the velocities of control points
    obj.v = new point * *[obj.w];
    for (i = 0; i < obj.w; i++) {
        obj.v[i] = new point * [obj.b];
        for (j = 0; j < obj.b; j++) {
            obj.v[i][j] = new point[obj.h];
            for (k = 0; k < obj.h; k++)
            {
                obj.v[i][j][k].x = initialVelocity.x;
                obj.v[i][j][k].y = initialVelocity.y;
                obj.v[i][j][k].z = initialVelocity.z;
            }
        }
    }
    return obj;
}


void Test() {
    struct world jello;
    int i, j, k;
    double x, y, z;

    strcpy(jello.integrator, "RK4");
    jello.dt = 0.0005000;
    jello.n = 1;

    jello.incPlanePresent = 1;
    jello.a = -1;
    jello.b = 1;
    jello.c = 1;
    jello.d = 2;

    jello.bbw = 4.0;
    jello.bbb = 4.0;
    jello.bbh = 4.0;

    jello.resolution = 30;
    jello.forceField = (struct point*)malloc(jello.resolution * jello.resolution * jello.resolution * sizeof(struct point));
    for (i = 0; i <= jello.resolution - 1; i++)
        for (j = 0; j <= jello.resolution - 1; j++)
            for (k = 0; k <= jello.resolution - 1; k++)
            {
                jello.forceField[i * jello.resolution * jello.resolution + j * jello.resolution + k].x = 0;
                jello.forceField[i * jello.resolution * jello.resolution + j * jello.resolution + k].y = 0;
                jello.forceField[i * jello.resolution * jello.resolution + j * jello.resolution + k].z = 0;
            }

    jello.objectCount = 1;
    jello.objects = new world::object[jello.objectCount];
    jello.objects[0] = createObject(point());
    writeWorld("jello.w", &jello);
}


void TestDefault() {
    struct world jello;
    int i, j, k;
    double x, y, z;

    strcpy(jello.integrator, "RK4");
    jello.dt = 0.0005000;
    jello.n = 1;

    jello.incPlanePresent = 1;
    jello.a = -1;
    jello.b = 1;
    jello.c = 1;
    jello.d = 2;

    jello.bbw = 4.0;
    jello.bbb = 4.0;
    jello.bbh = 4.0;

    jello.resolution = 30;
    jello.forceField = (struct point*)malloc(jello.resolution * jello.resolution * jello.resolution * sizeof(struct point));
    for (i = 0; i <= jello.resolution - 1; i++)
        for (j = 0; j <= jello.resolution - 1; j++)
            for (k = 0; k <= jello.resolution - 1; k++)
            {
                jello.forceField[i * jello.resolution * jello.resolution + j * jello.resolution + k].x = 0;
                jello.forceField[i * jello.resolution * jello.resolution + j * jello.resolution + k].y = 0;
                jello.forceField[i * jello.resolution * jello.resolution + j * jello.resolution + k].z = 0;
            }

    jello.objectCount = 1;
    jello.objects = new world::object[jello.objectCount];
    jello.objects[0] = createObject(point());
    writeWorld("default-jello.w", &jello);
}


void TestDefaultWithForceField() {
    struct world jello;
    int i, j, k;
    double x, y, z;

    // set the integrator and the physical parameters
    // the values below are EXAMPLES, to be modified by you as needed
    strcpy(jello.integrator, "RK4");
    jello.dt = 0.0005000;
    jello.n = 1;


    // set the inclined plane (not used in this assignment; ignore)
    jello.incPlanePresent = 1;
    jello.a = -1;
    jello.b = 1;
    jello.c = 1;
    jello.d = 2;

    jello.bbw = 4.0;
    jello.bbb = 4.0;
    jello.bbh = 4.0;

    // set the external force field
    jello.resolution = 30;
    jello.forceField =
        (struct point*)malloc(jello.resolution * jello.resolution * jello.resolution * sizeof(struct point));
    for (i = 0; i <= jello.resolution - 1; i++)
        for (j = 0; j <= jello.resolution - 1; j++)
            for (k = 0; k <= jello.resolution - 1; k++)
            {
                // set the force at node i,j,k
                // actual space location = x,y,z
                x = -1 * (jello.bbw / 2) + jello.bbw * (1.0 * i / (jello.resolution - 1));
                y = -1 * (jello.bbb / 2) + jello.bbb * (1.0 * j / (jello.resolution - 1));
                z = -1 * (jello.bbh / 2) + jello.bbh * (1.0 * k / (jello.resolution - 1));

                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].x = -x / 3.0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].y = -y / 3.0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].z = -z / 3.0;
            }



    jello.objectCount = 1;
    jello.objects = new world::object[jello.objectCount];

    jello.objects[0] = createObject(point());

    // write the jello variable out to file on disk
    // change jello.w to whatever you need
    writeWorld("jello-dynamic-force.w", &jello);

}

void TestDefaulGravity() {
    struct world jello;
    int i, j, k;
    double x, y, z;

    // set the integrator and the physical parameters
    // the values below are EXAMPLES, to be modified by you as needed
    strcpy(jello.integrator, "RK4");
    jello.dt = 0.0005000;
    jello.n = 1;


    // set the inclined plane (not used in this assignment; ignore)
    jello.incPlanePresent = 1;
    jello.a = -1;
    jello.b = 1;
    jello.c = 1;
    jello.d = 2;

    jello.bbw = 4.0;
    jello.bbb = 4.0;
    jello.bbh = 4.0;

    // set the external force field
    jello.resolution = 30;
    jello.forceField =
        (struct point*)malloc(jello.resolution * jello.resolution * jello.resolution * sizeof(struct point));
    for (i = 0; i <= jello.resolution - 1; i++)
        for (j = 0; j <= jello.resolution - 1; j++)
            for (k = 0; k <= jello.resolution - 1; k++)
            {
                // set the force at node i,j,k
                // actual space location = x,y,z
                x = -1 * (jello.bbw / 2) + jello.bbw * (1.0 * i / (jello.resolution - 1));
                y = -1 * (jello.bbb / 2) + jello.bbb * (1.0 * j / (jello.resolution - 1));
                z = -1 * (jello.bbh / 2) + jello.bbh * (1.0 * k / (jello.resolution - 1));

                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].x = 0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].y = 0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].z = -0.06;
            }



    jello.objectCount = 1;
    jello.objects = new world::object[jello.objectCount];

    jello.objects[0] = createObject(point());

    // write the jello variable out to file on disk
    // change jello.w to whatever you need
    writeWorld("default-jello-gravity.w", &jello);

}

void TestRotateObject() {
    struct world jello;
    int i, j, k;
    double x, y, z;

    // set the integrator and the physical parameters
    // the values below are EXAMPLES, to be modified by you as needed
    strcpy(jello.integrator, "RK4");
    jello.dt = 0.0005000;
    jello.n = 1;


    // set the inclined plane (not used in this assignment; ignore)
    jello.incPlanePresent = 1;
    jello.a = -1;
    jello.b = 1;
    jello.c = 1;
    jello.d = 2;

    jello.bbw = 4.0;
    jello.bbb = 4.0;
    jello.bbh = 4.0;

    // set the external force field
    jello.resolution = 30;
    jello.forceField =
        (struct point*)malloc(jello.resolution * jello.resolution * jello.resolution * sizeof(struct point));
    for (i = 0; i <= jello.resolution - 1; i++)
        for (j = 0; j <= jello.resolution - 1; j++)
            for (k = 0; k <= jello.resolution - 1; k++)
            {
                // set the force at node i,j,k
                // actual space location = x,y,z
                x = -1 * (jello.bbw / 2) + jello.bbw * (1.0 * i / (jello.resolution - 1));
                y = -1 * (jello.bbb / 2) + jello.bbb * (1.0 * j / (jello.resolution - 1));
                z = -1 * (jello.bbh / 2) + jello.bbh * (1.0 * k / (jello.resolution - 1));

                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].x = 0.019531;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].y = -0.019531;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].z = 0;
            }



    jello.objectCount = 1;
    jello.objects = new world::object[jello.objectCount];

    jello.objects[0] = createObject(point());

    // write the jello variable out to file on disk
    // change jello.w to whatever you need
    writeWorld("default-jello-rotate.w", &jello);

}

void TestTwoCubes() {

    struct world jello;
    int i, j, k;
    double x, y, z;

    // set the integrator and the physical parameters
    // the values below are EXAMPLES, to be modified by you as needed
    strcpy(jello.integrator, "RK4");
    jello.dt = 0.0005000;
    jello.n = 1;


    // set the inclined plane (not used in this assignment; ignore)
    jello.incPlanePresent = 1;
    jello.a = -1;
    jello.b = 1;
    jello.c = 1;
    jello.d = 2;

    jello.bbw = 4.0;
    jello.bbb = 4.0;
    jello.bbh = 4.0;

    // set the external force field
    jello.resolution = 30;
    jello.forceField =
        (struct point*)malloc(jello.resolution * jello.resolution * jello.resolution * sizeof(struct point));
    for (i = 0; i <= jello.resolution - 1; i++)
        for (j = 0; j <= jello.resolution - 1; j++)
            for (k = 0; k <= jello.resolution - 1; k++)
            {
                // set the force at node i,j,k
                // actual space location = x,y,z
                x = -1 * (jello.bbw / 2) + jello.bbw * (1.0 * i / (jello.resolution - 1));
                y = -1 * (jello.bbb / 2) + jello.bbb * (1.0 * j / (jello.resolution - 1));
                z = -1 * (jello.bbh / 2) + jello.bbh * (1.0 * k / (jello.resolution - 1));

                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].x = -x/10.0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].y = -y/10.0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].z = -z/10.0;
            }

    

    jello.objectCount = 2;
    jello.objects = new world::object[jello.objectCount];


    jello.objects[0] = createObject(point(0.6, 1.0, 0.9), 8, 8, 8, 0.8, 0.8, 0.8, 170, 0.25, 340, 0.25, 1.0, point(-15.0, -11.0, -12.0));
    jello.objects[1] = createObject(point(-0.6, -0.9, -1.0), 8, 8, 8, 0.8, 0.8, 0.8, 170, 0.25, 340, 0.25, 1.0, point(15.0, 11.0, 12.0));

    // write the jello variable out to file on disk
    // change jello.w to whatever you need
    writeWorld("two-jellos.w", &jello);


}

void TestLargeCube() {

    struct world jello;
    int i, j, k;
    double x, y, z;

    // set the integrator and the physical parameters
    // the values below are EXAMPLES, to be modified by you as needed
    strcpy(jello.integrator, "RK4");
    jello.dt = 0.0005000;
    jello.n = 1;


    // set the inclined plane (not used in this assignment; ignore)
    jello.incPlanePresent = 1;
    jello.a = -1;
    jello.b = 1;
    jello.c = 1;
    jello.d = 2;

    jello.bbw = 8.0;
    jello.bbb = 8.0;
    jello.bbh = 8.0;

    // set the external force field
    jello.resolution = 30;
    jello.forceField =
        (struct point*)malloc(jello.resolution * jello.resolution * jello.resolution * sizeof(struct point));
    for (i = 0; i <= jello.resolution - 1; i++)
        for (j = 0; j <= jello.resolution - 1; j++)
            for (k = 0; k <= jello.resolution - 1; k++)
            {
                // set the force at node i,j,k
                // actual space location = x,y,z
                x = -1 * (jello.bbw / 2) + jello.bbw * (1.0 * i / (jello.resolution - 1));
                y = -1 * (jello.bbb / 2) + jello.bbb * (1.0 * j / (jello.resolution - 1));
                z = -1 * (jello.bbh / 2) + jello.bbh * (1.0 * k / (jello.resolution - 1));

                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].x = 0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].y = 0;
                jello.forceField[i * jello.resolution * jello.resolution
                    + j * jello.resolution + k].z = 0;
            }



    jello.objectCount = 1;
    jello.objects = new world::object[jello.objectCount];


    jello.objects[0] = createObject(point(0.0, 0.0, 0.0), 8, 8, 8, 2, 2, 2, 170, 0.25, 340, 0.25, 1.0, point(-15.0, -11.0, -12.0));


    // write the jello variable out to file on disk
    // change jello.w to whatever you need
    writeWorld("large-cube.w", &jello);


}


/* modify main to screate your own world */
int main()
{
  Test();
  //TestDefault();
  //TestDefaulGravity();
  //TestDefaultWithForceField();
  //TestRotateObject();
  //TestTwoCubes();
  //TestLargeCube();
  return 0;
}

