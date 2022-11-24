
#include <iostream>
#include <Eigen/Dense>
#include "gnc.h"
using namespace std;
using namespace Eigen;

int main()
{
    
    MatrixXi A = MatrixXi::Random(3,3);
    Matrix3i B=  Matrix3i::Zero(3,3);
    VectorXi Arr {{1,2,3,53,4,99}};
    VectorXi nu {{14,2,4,6,8,10}}; 
    float phi {0.5};
    float theta {0.6};
    float psi {0.8};
    cout<<Rzyx(phi,theta,psi)<<endl;
    cout<<"----------------"<<endl;
   

    return 0;
}
