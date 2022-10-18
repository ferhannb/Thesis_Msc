#include <Eigen/Dense>

using namespace Eigen;

Matrix3f  Smtrx(const Vector3f &a);

MatrixXf  Hmtrx(const VectorXf &a);

VectorXf attitudeEuler(const VectorXf &eta,const VectorXf &nu, const float sampleTime);