#ifndef MYQUAT_H
#define MYQUAT_H

#include <math.h>

class Quat
{
    public:
        Quat();
        Quat(float angle, float vect[3]);
        Quat(float x, float y, float z, float w);
        float GetVectNorm(float vect[3]);
        float GetQuatNorm(float quat[4]);
        float RadiansToDegrees(float rad);
        float DegreesToRadians(float deg);
        float* VectNormalize(float vect[3]);
        float* QuatNormalize(float quat[4]);
        float** QuatToMat(Quat* quat);
        Quat* QuatNormalize(Quat* quat);
        Quat* MatToQuat(float** mat);
        Quat* Conjugate(Quat* quat);
        Quat* VectToQuat(float vect[3]);
        Quat* QuatMultiply(Quat* quatA, Quat* quatB);
        float* Rotate(float* axis, float* currentPos, float angle);
        static float* SommeVert(float* a, float* b);
        //void Rotate(float vectAxis[3], float vectToRotate[3], float radianAngle);

        float x, y, z, w;
        float vect [3];

    protected:
    private:
};

#endif // MAP_H

