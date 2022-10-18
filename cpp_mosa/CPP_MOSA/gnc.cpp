#include "gnc.h"


Matrix3f  Smtrx(const Vector3f &a)
{
    Matrix3f S  {{0,-a[2],a[1]}
                 ,{a[2],0,-a[0]}
                 ,{-a[1],a[0],0}};
    return S;  
}

MatrixXf  Hmtrx(const VectorXf &a)
{
    MatrixXf H = MatrixXf::Identity(6,6);
    return H.block(0,3,3,3) = Smtrx(a).transpose();
}


VectorXf attitudeEuler(const VectorXf &eta,const VectorXf &nu, const float sampleTime)

{
    
}