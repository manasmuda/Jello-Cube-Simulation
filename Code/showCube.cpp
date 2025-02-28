/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/



#include "jello.h"
#include "showCube.h"

extern Hit curSelectedHit;

point pointMap(world::object* obj, world::object::Face side, int i, int j)
{
  point p;

  switch (side)
  {
  case world::object::Face::BOTTOM: //[i][j][0] bottom face
    p = obj->p[i][j][0];
    break;
  case world::object::Face::TOP: //[i][j][7] top face
    p = obj->p[i][j][obj->h -1];
    break;
  case world::object::Face::FRONT: //[i][0][j] front face
    p = obj->p[i][0][j];
    break;
  case world::object::Face::BACK: //[i][7][j] back face
    p = obj->p[i][obj->b - 1][j];
    break;
  case world::object::Face::LEFT: //[0][i][j] left face
    p = obj->p[0][i][j];
    break;
  case world::object::Face::RIGHT: //[7][i][j] right face
    p = obj->p[obj->w - 1][i][j];
    break;
  }

  return p;
}

void renderWorld(struct world* jellowWorld) {

    for (int i = 0; i < jellowWorld->objectCount; i++)
    {
        world::object* obj = &jellowWorld->objects[i];
        if (fabs(obj->p[0][0][0].x) > jellowWorld->bbw)
        {
            printf("Your cube somehow escaped way out of the box.\n");
            exit(0);
        }

        renderObject(obj);
    }
}

void renderObject(struct world::object* jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  

  int faceN;
  double faceFactor, length;

 

  #define NODE(face,i,j) pointMap(jello, (face),(i),(j))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if(IsPointValid(jello, ip, jp, kp) && IsPointValid(jello, i, j, k)) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\

 
  if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    for (i=0; i< jello->w; i++)
      for (j=0; j< jello->b; j++)
        for (k=0; k< jello->h; k++)
        {
          if (i*j*k*(jello->w-i-1)*(jello->b-j-1)*(jello->h-k-1) != 0) // not surface point
            continue;

          glBegin(GL_POINTS); // draw point
            glColor4f(0,0,0,0);  
            glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
          glEnd();

          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;

          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
            glColor4f(0,0,1,1);
            PROCESS_NEIGHBOUR(1,0,0);
            PROCESS_NEIGHBOUR(0,1,0);
            PROCESS_NEIGHBOUR(0,0,1);
            PROCESS_NEIGHBOUR(-1,0,0);
            PROCESS_NEIGHBOUR(0,-1,0);
            PROCESS_NEIGHBOUR(0,0,-1);
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0,1,0,1);
            PROCESS_NEIGHBOUR(1,1,0);
            PROCESS_NEIGHBOUR(-1,1,0);
            PROCESS_NEIGHBOUR(-1,-1,0);
            PROCESS_NEIGHBOUR(1,-1,0);
            PROCESS_NEIGHBOUR(0,1,1);
            PROCESS_NEIGHBOUR(0,-1,1);
            PROCESS_NEIGHBOUR(0,-1,-1);
            PROCESS_NEIGHBOUR(0,1,-1);
            PROCESS_NEIGHBOUR(1,0,1);
            PROCESS_NEIGHBOUR(-1,0,1);
            PROCESS_NEIGHBOUR(-1,0,-1);
            PROCESS_NEIGHBOUR(1,0,-1);

            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)
            PROCESS_NEIGHBOUR(1,1,-1)
            PROCESS_NEIGHBOUR(-1,1,-1)
            PROCESS_NEIGHBOUR(-1,-1,-1)
            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(1,0,0,1);
            PROCESS_NEIGHBOUR(2,0,0);
            PROCESS_NEIGHBOUR(0,2,0);
            PROCESS_NEIGHBOUR(0,0,2);
            PROCESS_NEIGHBOUR(-2,0,0);
            PROCESS_NEIGHBOUR(0,-2,0);
            PROCESS_NEIGHBOUR(0,0,-2);
          }           
          glEnd();
        }
    glEnable(GL_LIGHTING);
  }
  
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL); 
    
    for (faceN=1; faceN <= 6; faceN++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
        
        int horizontal = 0;
        int vertical = 0;

        if (faceN == 1 || faceN == 6) {
            horizontal = jello->w;
            vertical = jello->b;
        }
        else if (faceN == 2 || faceN == 5) {
            horizontal = jello->w;
            vertical = jello->h;
        }
        else {
            horizontal = jello->b;
            vertical = jello->h;
        }
      
      if ((faceN==1) || (faceN==3) || (faceN==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;

      point** normal = new point * [horizontal];
      int** counter = new int* [horizontal];

      for (i = 0; i < horizontal; i++) { // reset buffers
          normal[i] = new point[vertical];
          counter[i] = new int[vertical];
          for (j = 0; j < vertical; j++)
          {
              normal[i][j].x = 0; normal[i][j].y = 0; normal[i][j].z = 0;
              counter[i][j] = 0;
          }
      }

      /* process triangles, accumulate normals for Gourad shading */
  
      world::object::Face face = (world::object::Face)faceN;

      for (i=0; i < (horizontal - 1); i++)
        for (j=0; j < (vertical - 1); j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face, i + 1, j), NODE(face, i, j), r1);
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

      
        /* the actual rendering */
        for (j=1; j< vertical; j++) 
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation
         
          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<horizontal; i++)
          {
            glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
              normal[i][j].z / counter[i][j]);
            glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
            glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
              normal[i][j-1].z / counter[i][j-1]);
            glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
          }
          glEnd();
        }
        
        for (i = 0; i < horizontal; i++) {
            delete[] normal[i];
            delete[] counter[i];
        }
        delete[] normal;
        delete[] counter;
    }  
  } // end for loop over faces

  if (curSelectedHit.isValid()) {
      point* curSelectedPoint = curSelectedHit.hitPoint;

      glPushMatrix(); // Save current transformation

      // Move to the point's position
      glTranslatef(curSelectedPoint->x, curSelectedPoint->y, curSelectedPoint->z);

      // Set color
      glColor4f(1, 1, 0, 1);

      // Draw a small solid sphere
      float radius = 0.03f; // Adjust size as needed
      glutSolidSphere(radius, 10, 10);

      glPopMatrix();
  }

  glFrontFace(GL_CCW);
}

void showBoundingBox(struct world* jello)
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  double minX = -1.0 * jello->bbw / 2.0;
  double maxX = 1.0 * jello->bbw / 2.0;

  double minY = -1.0 * jello->bbb / 2.0;
  double maxY = 1.0 * jello->bbb / 2.0;

  double minZ = -1.0 * jello->bbh / 2.0;
  double maxZ = 1.0 * jello->bbh / 2.0;

  // front face
  for(i=minX; i<=maxX; i++)
  {
    glVertex3f(i, minY, minZ);
    glVertex3f(i, minY, maxZ);
  }
  for(j=minZ; j<=maxZ; j++)
  {
    glVertex3f(minX,minY,j);
    glVertex3f(maxX, minY, j);
  }

  // back face
  for (i = minX; i <= maxX; i++)
  {
    glVertex3f(i,maxY,minZ);
    glVertex3f(i,maxY,maxZ);
  }
  for(j=minZ; j<=maxZ; j++)
  {
    glVertex3f(minX,maxY,j);
    glVertex3f(maxX,maxY,j);
  }

  // left face
  for(i=minY; i<=maxY; i++)
  {
    glVertex3f(minX,i,minZ);
    glVertex3f(minX,i,maxZ);
  }
  for(j=minZ; j<=maxZ; j++)
  {
    glVertex3f(minX,minY,j);
    glVertex3f(minX,maxY,j);
  }

  // right face
  for(i=minY; i<=maxY; i++)
  {
    glVertex3f(maxX,i,minZ);
    glVertex3f(maxX,i,maxZ);
  }
  for(j=minZ; j<=maxZ; j++)
  {
    glVertex3f(maxX,minY,j);
    glVertex3f(maxX,maxY,j);
  }

  glColor4f(1, 0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);

  glColor4f(0, 1, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);

  glColor4f(0, 0, 1, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1);
  
  glEnd();

  return;
}

