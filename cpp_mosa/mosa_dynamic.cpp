#include <iostream>
#include <Eigen/Dense>

 
using namespace std;
using namespace Eigen;

int rowsNumber = 5;
int columnNumber= 5;

class Otter_dynamic{
    public:

        MatrixXf C = MatrixXf::Zero(6,6);
        float ref = 0.0;
        float V_c = 0.0;
        float beta_c = 0.0;
        const float T_n = 0.32;
        const float L = 2.0;
        const float B = 1.08;
        VectorXf nu = {0.0,0.0,0.0,0.0,0.0,0.0};
        Vector2f u_actual = {0,0};
        Vector2f u_control = {0,0};

        // Constants 
        const float g = 9.81;        
        const int rho = 1025;
        const float  m  = 55.0; 
        const float  mp = 25.0;
        float m_total = m+mp;
        Vector3f rp = {0,0,0};
        Vector3f rg = {0.2,0,-0.2}
        


}



int main() {

    MatrixXf matrix1zeros;
    matrix1zeros = MatrixXf::Zero(rowsNumber, columnNumber);
    cout << "\n \n"<< matrix1zeros<<endl;

    return 0;
}
