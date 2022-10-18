
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main()
{
    
    MatrixXi A = MatrixXi::Random(6,6);
    MatrixXi B=  MatrixXi::Zero(3,3);
    A.block(0,3,3,3) = B;
    cout<<A<<endl;
    cout<<"----------------"<<endl;
    cout<<A.transpose()<<endl;

    return 0;
}
