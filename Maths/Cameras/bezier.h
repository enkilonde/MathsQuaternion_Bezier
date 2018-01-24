#ifndef _BEZIER_H_
#define _BEZIER_H_

#include "MyQuaternion.h"

class bezier{
public:

    Quat Quaternion;
    
    int precision;
	bool showControlPoints;
	bezier(){
		precision = 5;
		showControlPoints = true;
	}
	virtual void draw(){ 
	}
	void setPrecisions(int precision){
		if(precision < 2){
		precision = 2;
		}
		this->precision = precision;
	}
	void ShowControlPoints(bool flag){
		showControlPoints = flag;
	}
};

#endif
