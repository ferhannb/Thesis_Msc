#include "heading_controller/HeadingController.h"

HeadingController::HeadingController(ros::NodeHandle nh){
  /* Autopilot Reference Model Init */
  const double zeta = 1;
  double omega_n = 0.6;
  const double c = -pow(omega_n,3);
  const double b = -(2*zeta+1)*pow(omega_n,2);
  const double a = -(2*zeta+1)*omega_n;

  A_d << 0, 1, 0,
         0, 0, 1,
         c, b, a;
  B_d << 0, 0,-c;

  m_x_d << 0, 0, 0;

  r_max = 3*D2R;
  r_dot_max = 1*D2R;

  accumulation = 0;
  state = 0;
  psi_last = 0;
  // filterInit = true;

  /* Nomoto Model parameters */
  K = -0.1371;
  T = 2.4457;

  /* Feedback variables and parameters*/
  double m = T/K;
  double d = 1/K;
  K_p = m*pow(omega_n,2);
  K_i = omega_n*K_p/10.0;
  K_i = 0;
  K_d = (2*zeta*omega_n*m - d);
  psi_tilde = 0;
  r_tilde = 0;
  prev_psi_d = 0;
  ROS_INFO("Heading Controller Gains: Kp = %f, Ki = %f, Kd = %f", K_p, K_i, K_d);

  /* Init publishers */
  controlInputPub = nh.advertise<std_msgs::Float64>("heading_controller_output", 1);
  hcDataPub = nh.advertise<custom_msgs::HeadingControllerData>("heading_controller_data", 1);

  /*Init subscribers */
  stateSub = nh.subscribe("heading_controller_input", 1, &HeadingController::computeControlInput, this);

  /* Init Service */
  enable_heading_controller_service = nh.advertiseService("enable_heading_control", &HeadingController::setReferenceFilterInitialCondition, this);
}

void HeadingController::computeControlInput(const custom_msgs::HeadingControllerInput &input){
  //return; 
  double delta_ff = 0;
  Vector3d x_d;
  // Calculate dt
  if(!prev_time.isZero()){
    dt = ros::Time::now() - prev_time;
    prev_time = ros::Time::now();
  }
  else{
    // init prev_time and heading
    prev_psi_d = input.state*D2R;
    prev_time = ros::Time::now();
    return;
  }

  double psi_continuous = 0;
  // Create a continuous signal for the reference filter [-Inf Inf]
  psi_continuous = wrapToInf(input.setpoint*D2R);
  x_d = referenceFilter(psi_continuous);
  // Wrap desired heading from reference filter back to [-pi pi]
  x_d(0) = wrapToPi(x_d(0));

  // Compute feedforward term
  delta_ff = computeFeedforward(x_d(1), x_d(2));
 
  prev_psi_d = input.setpoint*D2R;
  std_msgs::Float64 controlInput;
  custom_msgs::HeadingControllerData hcData;

  
  // Compute feedforward term
  // double delta_ff = computeFeedforward(x_d(1), x_d(2));
  // delta_ff = 0;
  
  // Compute feedback term
  double psi_tilde = x_d(0) -input.state*D2R;
  double r_tilde = x_d(1) - input.r;
  
  
  if(psi_tilde > M_PI)
    psi_tilde = psi_tilde - 2*M_PI;
  else if(psi_tilde < -M_PI)
    psi_tilde = psi_tilde + 2*M_PI;
  
  psi_tilde_integral += psi_tilde*dt.toSec();

  // Integrator anti-windup
  if(psi_tilde_integral > 10.0*D2R)
    psi_tilde_integral = 10.0*D2R;
  else if(psi_tilde_integral < -10.0*D2R)
    psi_tilde_integral = -10.0*D2R;
  
  double delta_fb = -(K_p*psi_tilde + K_i*psi_tilde_integral + K_d*r_tilde);
  if(delta_fb != delta_fb){
    ROS_INFO("nan in fb");
    return;
  }
  if(delta_ff != delta_ff){
    ROS_INFO("nan in ff");
    return;
  }
  // Fill message : FF + FB
  
  controlInput.data = (delta_ff + delta_fb)*R2D;
  if(controlInput.data != controlInput.data)
    return; // NaN value

  // Saturate angle
  if(controlInput.data > 45)
    controlInput.data = 45;
  else if(controlInput.data < -45)
    controlInput.data = -45;

  controlInputPub.publish(controlInput);
  hcData.ff = delta_ff*R2D;
  hcData.fb = delta_fb*R2D;
  hcData.psi_tilde = psi_tilde;
  hcData.r_tilde = r_tilde;
  hcData.psi_d = x_d(0);
  hcData.r_d = x_d(1);
  hcData.r_d_dot = x_d(2);
  hcDataPub.publish(hcData);
  std::cout << "Heading Controller Info:"<< std::endl << "FF + FB : " << controlInput.data << " FF: "
  << delta_ff*R2D << " FB: "  << delta_fb*R2D << std::endl << "psi_d: " << x_d(0)*R2D << " r_d: "
  << x_d(1)*R2D << " psi_r: " << input.setpoint << std::endl <<  "psi  : " << input.state << " r  : " << input.r*R2D << std::endl
  << "psi_contin: " << psi_continuous << std::endl<< std::endl;
}

/**
 * Computes reference filter states. I/O in radians
 * */
Vector3d HeadingController::referenceFilter(const double psi_r){
  double h = 0.1;
  // Create local return vector of desired vessel states
  Vector3d x_d_ = m_x_d;

  // Check saturation limits to yaw rate and yaw acceleration
  if(x_d_(1) > r_max){
    x_d_(1) = r_max;
    x_d_(2) = 0;
  }
  else if(x_d_(1) < -r_max){
    x_d_(1) = -r_max;
    x_d_(2) = 0;
  }

  if(x_d_(2) > r_dot_max)
    x_d_(2) = r_dot_max;
  else if(x_d_(2) < -r_dot_max)
    x_d_(2) = -r_dot_max;

  // Euler integration of member variable (global) x_d with local x_d_
  m_x_d = x_d_ + h*(A_d*m_x_d + B_d*psi_r);
  // return x_d_ as it contains current filter states
  return x_d_;
}

/**
 * Computes the feedforward term
 * */
double HeadingController::computeFeedforward(const double r_d, const double r_d_dot){
  double tau_k = 0.2180;
  double tau_m = 60;
  double l_x = 1.2;

  double tau_ff = (T/K)*(r_d_dot + (1/T)*r_d);
  double delta_ff = asin(tau_ff/(-2*l_x*tau_k*tau_m));
  delta_ff=-tau_ff;
  if(delta_ff != delta_ff)
    delta_ff = 0;
  return delta_ff;
}

/**
 * Utility Function for creating a continuous signal as input to
 * the reference model.
 * Source: Nonlinear Maneuvering Control of Underactuated ships (Breivik 2003)
 * */

double HeadingController::wrapToInf(double psi_r){
  if(psi_r >=0 && psi_r <= M_PI/2){
    // First quadrant
    if(state == 3){
      if(psi_r + fabs(psi_last) <= M_PI)
        accumulate = psi_r - psi_last;
      else
        accumulate = psi_last - psi_r + 2*M_PI;
      }
    else
      accumulate = psi_r - psi_last;
    state = 1;
  }
  else if(psi_r < 0 && psi_r >= -M_PI/2){
    // Second quadrant
    if(state ==4){
      if(fabs(psi_r) + psi_last <= M_PI)
        accumulate = psi_r - psi_last;
      else
        accumulate = psi_r - psi_last + 2*M_PI;
    }
    else // state == 1, 2 or 3
      accumulate = psi_r - psi_last;
    state = 2;
  }
  else if(psi_r < -M_PI/2 && psi_r > -M_PI){
    // Third quadrant
    if(state == 1){
      if(fabs(psi_r) + psi_last <= M_PI)
        accumulate = psi_r - psi_last;
      else
        accumulate = psi_r - psi_last + 2*M_PI;
    }
    else if(state == 4)
      accumulate = psi_r - psi_last + 2*M_PI;
    else // state == 2 or 3
      accumulate = psi_r - psi_last;
    state = 3;
  }
  else{
    // Fourth quadrant
    if(state == 2){
      if(psi_r + fabs(psi_last) <= M_PI)
        accumulate = psi_r - psi_last;
      else
        accumulate = psi_last - psi_r + 2*M_PI;
    }
    else if(state == 3){
      accumulate = psi_r - psi_last - 2*M_PI;
    }
    else // state == 1 or 4
      accumulate = psi_r - psi_last;
    state = 4;
  }
  accumulation = accumulation + accumulate;
  psi_last = psi_r;
  return accumulation;
}
/**
 * Utility function for wrapping [-Inf Inf] -> [-pi pi]
 * Source: Nonlinear Maneuvering Control of Underactuated ships (Breivik 2003)
 * */

double HeadingController::wrapToPi(double psi_continuous){
  double n;
  double psi_mapped;
  if(psi_continuous > 0)
    n = floor(psi_continuous/M_PI);
  else if(psi_continuous < 0)
    n = ceil(psi_continuous/M_PI);
  else
    n = 0;
  int rem = remainder(n,2);

  if(rem == 0)
    psi_mapped = psi_continuous - n*M_PI;
  else{ 
    if(rem > 0)
      psi_mapped = psi_continuous - (n+1)*M_PI;
    else
      psi_mapped = psi_continuous - (n-1)*M_PI;
  }
  return psi_mapped;
}

bool HeadingController::setReferenceFilterInitialCondition(custom_msgs::EnableHeadingController::Request &req,
                            custom_msgs::EnableHeadingController::Response &res)
{
  // filterInit = true;
  psi_tilde_integral = 0.0;
  ROS_INFO("Heading Controller Service: Integral Reset");
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv,"HeadingControllerNode2");
  ros::NodeHandle nh;
  HeadingController headingController(nh);
  ros::spin();
  return 0;
}
