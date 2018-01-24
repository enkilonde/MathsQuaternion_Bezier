#include <stdio.h>

#include "MyQuaternion.h"

#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923

Quat::Quat() {
	w = 0.0f;
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

Quat::Quat(float _w, float _x, float _y, float _z){
    w = _w;
    x = _x;
    y = _y;
    z = _z;
}

Quat::Quat(float angle, float vect[3]){
    float sin_a = sin(angle / 2);

    w = cos(angle / 2);
    x = vect[0] * sin_a;
    y = vect[1] * sin_a;
    z = vect[2] * sin_a;
}

float Quat::GetVectNorm(float vect[3]){
    return sqrtf(vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2]);
}

float Quat::GetQuatNorm(float quat[4]){
    return sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
}

float* Quat::VectNormalize(float vect[3]){
    float length = sqrt(vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2]);
    vect[0] = vect[0]/length;
    vect[1] = vect[1]/length;
    vect[2] = vect[2]/length;
    return vect;
}

float* Quat::QuatNormalize(float vect[4]){
    float length = GetQuatNorm(vect);
    vect[0] = vect[0]/length;
    vect[1] = vect[1]/length;
    vect[2] = vect[2]/length;
    vect[3] = vect[3]/length;
    return vect;
}

Quat* Quat::QuatNormalize(Quat* quat){
    float tmp [] = {quat->w, quat->x, quat->y, quat->z};
    float length = GetQuatNorm(tmp);
    return new Quat(quat->w/length, quat->x/length, quat->y/length, quat->z/length);
}

Quat* Quat::Conjugate(Quat* quat){
    return new Quat(quat->w, -quat->x, -quat->y, -quat->z);
}

Quat* Quat::VectToQuat(float vect[3]){
    return new Quat(0, vect[0], vect[1], vect[2] );
}

float Quat::RadiansToDegrees(float rad){
    return rad*180/M_PI;
}

float Quat::DegreesToRadians(float deg){
    return deg*M_PI/180;
}

Quat* Quat::QuatMultiply(Quat* quatA, Quat* quatB){
    return new Quat(quatA->w*quatB->w - quatA->x*quatB->x - quatA->y*quatB->y - quatA->z*quatB->z,
                    quatA->w*quatB->x + quatA->x*quatB->w + quatA->y*quatB->z - quatA->z*quatB->y,
                    quatA->w*quatB->y - quatA->x*quatB->z + quatA->y*quatB->w + quatA->z*quatB->x,
                    quatA->w*quatB->z + quatA->x*quatB->y - quatA->y*quatB->x + quatA->z*quatB->w );
}

float* Quat::Rotate(float* axis, float* currentPos, float angle){
    axis = VectNormalize(axis);
    angle = DegreesToRadians(angle);

    // Angle par rapport au centre du monde ou centre de l'objet
    Quat* quat = new Quat(angle, axis);
    quat = quat->QuatNormalize(quat);

    Quat* res = new Quat();
    res = QuatMultiply(QuatMultiply(quat, new Quat(0, currentPos[0], currentPos[1], currentPos[2])), Conjugate(quat));

    //res = res->QuatNormalize(res);
    currentPos[0] = (float) res->x;
    currentPos[1] = (float) res->y;
    currentPos[2] = (float) res->z;

    return currentPos;
}

float* Quat::SommeVert(float* a, float* b)
{
    float* result = new float[3];
    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
    result[2] = a[2] + b[2];
    return result;
}

Quat* Quat::MatToQuat(float** mat){
    float t = mat[0][0] + mat[1][1] + mat[2][2] + 1;
    float s;

    if(t > 0) {
        s = 0.5/t;
        w = 0.25/s;
        x = (mat[2][1] - mat[1][2])*s;
        y = (mat[0][2] - mat[2][0])*s;
        z = (mat[1][0] - mat[0][1])*s;
    }
    else {
        if(mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]){
            s = sqrtf(1 + mat[0][0] - mat[1][1] - mat[2][2])* 2;
            w = (mat[1][2] - mat[2][1])/s;
            x = 0.25*s;
            y = (mat[0][1] - mat[1][0])/s;
            z = (mat[0][2] - mat[2][0])/s;
        }else if(mat[1][1] > mat[0][0] && mat[1][1] > mat[2][2]){
            s = sqrtf(1 - mat[0][0] + mat[1][1] - mat[2][2])* 2;
            w = (mat[0][2] - mat[2][0])/s;
            x = (mat[0][1] - mat[1][0])/s;
            y = 0.25*s;
            z = (mat[1][2] - mat[2][1])/s;
        }else{
            s = sqrtf(1 - mat[0][0] - mat[1][1] + mat[2][2])* 2;
            w = (mat[0][1] - mat[1][0])/s;
            x = (mat[0][2] - mat[2][0])/s;
            y = (mat[1][2] - mat[2][1])/s;
            z = 0.25*s;
        }
    }

    return new Quat(w, x, y, z);
}

float** Quat::QuatToMat(Quat* quat) {
    float xx = quat->x * quat->x;
    float xy = quat->x * quat->y;
    float xz = quat->x * quat->z;
    float xw = quat->x * quat->w;

    float yy = quat->y * quat->y;
    float yz = quat->y * quat->z;
    float yw = quat->y * quat->w;

    float zz = quat->z * quat->z;
    float zw = quat->z * quat->w;

    int width = 3;
    int height = 3;

    float** mat = 0;
    mat = new float*[height];

    for (int h = 0; h < height; h++)
    {
        mat[h] = new float[width];
        for (int w = 0; w < width; w++) {
            mat[h][w] = w + width * h;
        }
    }

    mat[0][0] = 1 - 2*yy - 2*zz;
    mat[0][1] = 2*xy - 2*zw;
    mat[0][2] = 2*xz + 2*yw;

    mat[1][0] = 2*xy + 2*zw;
    mat[1][1] = 1 - 2*xx - 2*zz;
    mat[1][2] = 2*yz - 2*xw;

    mat[2][0] = 2*xz - 2*yw;
    mat[2][1] = 2*yz + 2*xw;
    mat[2][2] = 1 - 2*xx - 2*yy;

    return mat;
}
