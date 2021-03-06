#include <iostream>
#include <cmath>
#include "bezier.h"
#include "bezierCurve.h"
#include "bezierSurface.h"
#include "bezierFile.h"
//#include <GL\glut.h>

#ifdef _MSC_VER
#pragma comment(lib, "opengl32.lib")
#include <windows.h>
#endif


#include "glew.h"
#ifdef _MSC_VER
#pragma comment(lib, "glew32.lib")
#endif

#include "freeglut.h"

#include <vector>


using namespace std;

#define PI 3.1415926535898

static int width = 600;
static int height = 600;
float camera_theta = PI/2.0;
float camera_phi = 0.0;
float camera_r = 30.0;

int precision = 5;
bool showControlPoints = true;
bezier * b;

void init(){
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glEnable(GL_NORMALIZE);
  glClearColor(0.5, 0.5, 0.5, 1.0);
  glShadeModel(GL_FLAT);
  glDisable(GL_COLOR_MATERIAL);
}


void display(){



  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(1, 1, width-2, height-2);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 50.0);
//  gluOrtho2D(0, width-2, height-2, 0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  gluLookAt(camera_r*sin(camera_theta)*cos(camera_phi), camera_r*cos(camera_theta), camera_r*sin(camera_theta)*sin(camera_phi),
         0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
  
  b->draw();
  glPopMatrix();
  glFlush();
  glutSwapBuffers();


}


void keyboard(unsigned char key, int x, int y)
{

  switch (key) {
  case 'z':
         if(camera_r - 1.0/18.0 > 0)
                 camera_r -= 1.0;
         break;
  case 's':
         camera_r += 1.0;
         break;

  case '4':
      //b->Quaternion .Rotate(10f, 0, 0);
      break;

  case '+':
      b->setPrecisions(b->precision+1);
      break;
  case '-':
      if(b->precision > 2)
        b->setPrecisions(b->precision-1);
      break;
  case 'x':
	     showControlPoints = !showControlPoints;
         b->controlPoints(showControlPoints);
         break;
  case 'a':
      camera_theta -= PI / 18.0;
      break;
  case 'e':
      camera_theta += PI / 18.0;
      break;
  case 'q':
      camera_phi += PI / 18;
      break;
  case 'd':
      camera_phi -= PI / 18;
      break;
  }
  glutPostRedisplay();


}

void special(int key, int x, int y)
{
        switch(key) {
        case GLUT_KEY_LEFT:
                camera_theta -= PI/18.0;
                break;
        case GLUT_KEY_RIGHT:
                camera_theta += PI/18.0;
                break;
        case GLUT_KEY_UP:
                camera_phi += PI/18;
                break;
        case GLUT_KEY_DOWN:
                camera_phi -= PI/18;
                break;
        }
        glutPostRedisplay();

}

int main(int argc, char * argv[])
{


    //char* wait = "i";
    //std::cout << "press a key to start" << std::endl;
    //std::cin >> wait;

    std::vector<char*> arguments;
    arguments.push_back("1");
    arguments.push_back("surfaceDemo");
    //arguments.push_back("3");

    argc = arguments.size();


        //don't forget to free
	    if (argc < 2) {
			 std::cout << "Usage is: <infile>\n";
			 //exit(0);
		}



        b = readBezierFile(arguments[1]);
		b->setPrecisions(5);
		b->controlPoints(showControlPoints);



        glutInit(&argc, &arguments[0]);
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
        glutInitWindowSize(width, height);
        glutInitWindowPosition(100, 100);
        glutCreateWindow("Bezier");
        init();

        


        glutDisplayFunc(display);
        glutKeyboardFunc(keyboard);
        glutSpecialFunc(special);
        glutMainLoop();
        return 0;
}
