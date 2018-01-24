#ifndef _BEZIERSURFACE_H_
#define _BEZIERSURFACE_H_
#include "bezier.h"
#include "MyQuaternion.h"
#include <vector>

class bezierSurface : public bezier{
public:
    int uPoints;
	int vPoints;
    float ***controlPoints;
	float ***curvePoints;


    std::vector<float> vertices;
    std::vector<float> curveIndexes;

	void bezierSurface::calSurface();
	bezierSurface(int uPoints, int vPoints, float **points[3]);
	void bezierSurface::setShowPoints(bool showControlPoints);
	virtual void draw();
        ~bezierSurface();

        /*Quat Quaternion;*/
};
#endif
