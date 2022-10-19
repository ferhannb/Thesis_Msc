#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

Matrix3f  Smtrx(const Vector3f &a);

MatrixXf  Hmtrx(const VectorXf &a);

VectorXf attitudeEuler(const VectorXf &eta,const VectorXf &nu, const float sampleTime);

Matrix3f Rzyx( float phi, float theta, float psi);

Matrix3f Tzyx(float phi, float theta);

