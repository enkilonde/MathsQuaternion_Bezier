#include <iostream>
#include <math.h>
#include "bezierSurface.h"
#include "bezierMath.h"
//#include <GL\glut.h>
#include <freeglut/include/GL/glut.h>
#define ALGORITHM 0

bezierSurface::bezierSurface(int uPoints, int vPoints, float **points[3]):bezier(){
	this->uPoints = uPoints;
	this->vPoints = vPoints;
	controlPoints = points;

    Quaternion = Quat();
}

void bezierSurface::setShowPoints(bool showControlPoints){
	this->showControlPoints = showControlPoints;
}

void bezierSurface::calSurface(){
	curvePoints = new float**[precision+1];
	for (int i = 0; i <= precision; i++){
		curvePoints[i] = new float*[precision+1];
	}
    int n = 0;
    // de Casteljau's
    for (int ui = 0; ui <= precision; ui++) {
        float u = float(ui) / float(precision);
        for (int vi = 0; vi <= precision; vi++) {
            float v = float(vi) / float(precision);
            float **qPoints = new float*[uPoints + 1];
            for (int i = 0; i <= uPoints; i++) {
                qPoints[i] = deCasteljau(controlPoints[i], vPoints, v);
            }
            curvePoints[ui][vi] = deCasteljau(qPoints, uPoints, u);

            

        }
    }
}

void bezierSurface::draw(){
	this->calSurface();
	glPushMatrix();
    bool color = false;
    float black[] = {0.3, 0.3, 0.3};
    float white[] = { 0.7, 0.7, 0.7};
	float red[] = {1.0, 0.0, 0.0};
	glColor3fv(black);
    for(int ui = 0; ui < precision; ui++) {
		glBegin(GL_TRIANGLE_STRIP);
        for(int vi = 0; vi <= precision; vi++) {
            if(color) {
				glColor3fv(white);
			} else {
                glColor3fv(black);
            }
			float * p1 = curvePoints[ui][vi];
            float * p2 = curvePoints[ui+1][vi];
            glVertex3fv(p1);
            glVertex3fv(p2);
			color = !color;
        }
		if(precision%2 == 1){
			color = !color;
		}
        glEnd();
    }
	glLineWidth(2.0);
	glColor3fv(red);
	if(showControlPoints){
		for(int m = 0; m <= uPoints; m++){
			glBegin(GL_LINE_STRIP);
			for(int n=0; n<=vPoints; n++){
				float * p = controlPoints[m][n];
				glVertex3fv(p);	
			}
			glEnd();
		}
		for(int n=0; n<=vPoints; n++){
			glBegin(GL_LINE_STRIP);
			for(int m = 0; m <= uPoints; m++){
				float * p = controlPoints[m][n];
				glVertex3fv(p);	
			}
			glEnd();
		}
	}
	glPopMatrix();
}
