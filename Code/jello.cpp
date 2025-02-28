/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  <write your name here>

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "SpringNetworkSim.h"
#include <gl/GLU.h>

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_vMouseDelta[2];
int g_iLeftMouseButtonDown,g_iMiddleMouseButtonDown,g_iRightMouseButtonDown;
int g_iLeftMouseButtonStarted,g_iMiddleMouseButtonStarted,g_iRightMouseButtonStarted;
int g_iLeftMouseButtonEnded,g_iMiddleMouseButtonEnded,g_iRightMouseButtonEnded;

Hit curSelectedHit;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear=0, bend=0, structural=1, pause=0, viewingMode=1, saveScreenToFile=0, transparentJello = 0;

struct world;


SpringNetworkSim* sim;


int windowWidth, windowHeight;

void myinit()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color to grey
  glClearColor(0.5, 0.5, 0.5, 0.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  //glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  return; 
}

void reshape(int w, int h) 
{
  // Prevent a divide by zero, when h is zero.
  // You can't make a window of zero height.
  if(h == 0)
    h = 1;

  glViewport(0, 0, w, h);

  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the perspective
  double aspectRatio = 1.0 * w / h;
  gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 

  windowWidth = w;
  windowHeight = h;

  glutPostRedisplay();
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // camera parameters are Phi, Theta, R
  gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
	        0.0,0.0,0.0, 0.0,0.0,1.0);


  /* Lighting */
  /* You are encouraged to change lighting parameters or make improvements/modifications
     to the lighting model . 
     This way, you will personalize your assignment and your assignment will stick out. 
  */

  // global ambient light
  GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };
  
  // light 's ambient, diffuse, specular
  GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
  GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

  GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
  GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

  GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

  GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
  GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

  GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
  GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

  GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

  // light positions and directions
  GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
  GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
  GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
  GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
  GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
  GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
  GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
  GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };
  
  // jelly material color

  GLfloat mKa[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat mKd[] = { 0.3, 0.3, 0.3, 1.0 };
  GLfloat mKs[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

  GLfloat mtKa[] = { 0.0, 0.0, 0.0, 0.5 };
  GLfloat mtKd[] = { 0.3, 0.3, 0.3, 0.5 };
  GLfloat mtKs[] = { 1.0, 1.0, 1.0, 0.5 };
  GLfloat mtKe[] = { 0.0, 0.0, 0.0, 0.5 };

  /* set up lighting */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);


  if (!transparentJello) {
      // set up cube color
      glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
      glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
      glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
      glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
      glMaterialf(GL_FRONT, GL_SHININESS, 120);
  }
  else {
      glMaterialfv(GL_FRONT, GL_AMBIENT, mtKa);
      glMaterialfv(GL_FRONT, GL_DIFFUSE, mtKd);
      glMaterialfv(GL_FRONT, GL_SPECULAR, mtKs);
      glMaterialfv(GL_FRONT, GL_EMISSION, mtKe);
      glMaterialf(GL_FRONT, GL_SHININESS, 120);
  }
  

  // macro to set up light i
  #define LIGHTSETUP(i, j)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##j);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##j);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##j);\
  glEnable(GL_LIGHT##i)

  if (!transparentJello) {

      LIGHTSETUP(0,0);
      LIGHTSETUP(1,1);
      LIGHTSETUP(2,2);
      LIGHTSETUP(3,3);
      LIGHTSETUP(4,4);
      LIGHTSETUP(5,5);
      LIGHTSETUP(6,6);
      LIGHTSETUP(7,7);
  }
  else {
      LIGHTSETUP(0, 1);
      LIGHTSETUP(1, 1);
      LIGHTSETUP(2, 1);
      LIGHTSETUP(3, 1);
      LIGHTSETUP(4, 1);
      LIGHTSETUP(5, 1);
      LIGHTSETUP(6, 1);
      LIGHTSETUP(7, 1);
  }

  // enable lighting
  glEnable(GL_LIGHTING);    
  glEnable(GL_DEPTH_TEST);

  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // show the cube
  renderWorld(sim->curWorld);

  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);

  glDisable(GL_LIGHTING);

  // show the bounding box
  showBoundingBox(sim->curWorld);
 
  glutSwapBuffers();
}



void castRayAndSelectPoint() {
    float mouseX = (float)g_vMousePos[0];
    float mouseY = (float)g_vMousePos[1];

    GLint viewport[4];
    GLdouble modelview[16], projection[16];
    GLfloat winX, winY, winZ;
    GLdouble worldStartX, worldStartY, worldStartZ, worldEndX, worldEndY, worldEndZ;

    // Get matrices and viewport
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    winX = mouseX;
    winY = (float)viewport[3] - mouseY; // Flip Y axis

    glReadPixels(mouseX, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
    gluUnProject(winX, winY, 0.0, modelview, projection, viewport, &worldStartX, &worldStartY, &worldStartZ);

    gluUnProject(winX, winY, 1.0, modelview, projection, viewport, &worldEndX, &worldEndY, &worldEndZ);

    // Compute ray direction
    double rayStartX = worldStartX;
    double rayStartY = worldStartY;
    double rayStartZ = worldStartZ;
    double rayDirX = worldEndX - worldStartX;
    double rayDirY = worldEndY - worldStartY;
    double rayDirZ = worldEndZ - worldStartZ;

    // Normalize ray direction
    double length = sqrt(rayDirX * rayDirX + rayDirY * rayDirY + rayDirZ * rayDirZ);
    rayDirX /= length;
    rayDirY /= length;
    rayDirZ /= length;

    if (ENABLE_LOGS) {
        printf("Ray Origin: (%lf, %lf, %lf)\n", rayStartX, rayStartY, rayStartZ);
        printf("Ray End: (%lf, %lf, %lf)\n", worldEndX, worldEndY, worldEndZ);
        printf("Ray Dir: (%lf, %lf, %lf)\n", rayDirX, rayDirY, rayDirZ);
    }

    curSelectedHit = sim->getRayCastPoint(point(rayStartX, rayStartY, rayStartZ), point(rayDirX, rayDirY, rayDirZ));
}

void updatePointAndMovePose() {
    double moveY = g_vMouseDelta[1];
    point moveForward, finalDest;
    pMULTIPLY(curSelectedHit.forward, moveY/20.0, moveForward)
        pSUM(*curSelectedHit.hitPoint, moveForward, finalDest)
        curSelectedHit.hitPoint->x = finalDest.x;
        curSelectedHit.hitPoint->y = finalDest.y;
        curSelectedHit.hitPoint->z = finalDest.z;
}


void setAndMovePoints() {
    if (g_iLeftMouseButtonStarted) {
        castRayAndSelectPoint();
    }

    if (g_iLeftMouseButtonDown) {
        if (curSelectedHit.isValid()) {
            updatePointAndMovePose();
        }
    }

    if (g_iLeftMouseButtonEnded) {
        curSelectedHit = Hit();
    }
}



void updateMouseInputs() {
    if (g_iLeftMouseButtonStarted) {
        g_iLeftMouseButtonStarted = false;
    }

    if (g_iLeftMouseButtonEnded) {
        g_iLeftMouseButtonEnded = false;
    }

    if (g_iRightMouseButtonStarted) {
        g_iRightMouseButtonStarted = false;
    }

    if (g_iRightMouseButtonEnded) {
        g_iRightMouseButtonEnded = false;
    }

    g_vMouseDelta[0] = 0;
    g_vMouseDelta[1] = 0;
}


int fCount = 0;

void doIdle()
{
  char s[20]="picxxxx.ppm";
  int i;
  
  // save screen to file
  s[3] = 48 + (sprite / 1000);
  s[4] = 48 + (sprite % 1000) / 100;
  s[5] = 48 + (sprite % 100 ) / 10;
  s[6] = 48 + sprite % 10;

  if (saveScreenToFile==1)
  {
    saveScreenshot(windowWidth, windowHeight, s);
    saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 300) // allow only 300 snapshots
  {
    exit(0);	
  }

  //sim->updateTime();

  if (pause == 0)
  {
      fCount++;

      if (fCount == sim->curWorld->n) {
          sim->updateSim();
          fCount = 0;
      }
  }

  setAndMovePoints();

  glutPostRedisplay();

  updateMouseInputs(); // Always at the end of loop
}



int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  world jello;

  readWorld(argv[1],&jello);


  sim = new SpringNetworkSim(&jello);

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE); // Add GLUT_MULTISAMPLE

  
  windowWidth = 640;
  windowHeight = 480;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Jello cube");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for window size changes */
  glutReshapeFunc(reshape);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  /* forever sink in the black hole */
  glutMainLoop();

  return(0);
}

