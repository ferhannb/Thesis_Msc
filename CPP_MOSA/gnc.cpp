#include "gnc.h"
#include <stdexcept>
#include <iostream>

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

Matrix3f Rzyx(float phi,float theta, float psi)
{
    float cphi = cos(phi);
    float sphi = cos(sphi);
    float cth = cos(theta);
    float sth = sin(theta);
    float cpsi = cos(psi);
    float spsi = sin(psi);

    Matrix3f R = {
                { cpsi*cth, -spsi*cphi+cpsi*sth*sphi, spsi*sphi+cpsi*cphi*sth },
                { spsi*cth,  cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi },
                { -sth,      cth*sphi,                 cth*cphi }
                };
 return R;
}

Matrix3f Tzyx( float phi, float theta)
{
    float cphi = cos(phi);
    float sphi = sin(phi);
    float cth  = cos(theta);
    float sth  = sin(theta);

    try
        {
               Matrix3f   T  {
            { 1,  sphi*sth/cth,  cphi*sth/cth },
            { 0,  cphi,          -sphi},
            { 0,  sphi/cth,      cphi/cth} };
            
            return T;
        }

    catch(...) {
            std::cout<<"Tzyx is singular for theta = +-90 degrees."<<std::endl;
               } 

}
VectorXf attitudeEuler(const VectorXf &eta,const VectorXf &nu, const float sampleTime)

{
    Vector3f p_dot = Rzyx(eta[3],eta[4],eta[5]);//*nu(seqN(0,3));
    Vector3f v_dot = Tzyx(eta[3],eta[4]);//*nu(seqN(3,3));

    // // Forward Euler Integration

    // eta.block(0,0,1,3) = eta.head(3)+sampleTime*p_dot;
    // eta.block(0,3,3,1) = eta.tail<3>() +sampleTime*v_dot;

    return p_dot;
}