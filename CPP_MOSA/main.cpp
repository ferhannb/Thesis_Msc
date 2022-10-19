
#include <iostream>
#include <Eigen/Dense>
#include "gnc.h"
using namespace std;
using namespace Eigen;

int main()
{
    
    MatrixXi A = MatrixXi::Random(3,3);
    Matrix3i B=  Matrix3i::Zero(3,3);
    Vector3i Arr {{1,2,3,53}}; 
    
    cout<<Arr.head(2)<<endl;
    // cout<<(A)<<endl;
    cout<<"----------------"<<endl;
   

    return 0;
}
