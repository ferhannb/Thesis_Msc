# include <iostream>

using namespace std;

class OtterDynamic {
    public:
    float nu[6]{0},current_eta[6]{0};
    float u_actual[2]{0},u_control[2]{0};
    float C[6][6]{0}; 
    float ref; // r 
    float V_c; // current velocity
    float beta_c; // current angle 
    
    const float  T_n = 0.32; // propeller time constant (s)
    const int L = 2; // Length (m)
    const float B = 1.08; // beam (m)
    const float g = 9.81;
    const int rho = 1025;
    const int m = 55;
    const int mp = 25;
    int m_total = m+mp;
    float rp[3]{0};
    float rg[3]={0.2,0.,-0.2};
    float rg = (m*rg+mp*rp)/(m+mp);

}


