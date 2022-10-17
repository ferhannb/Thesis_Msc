#include "guidance_law/GuidanceLaw.h"

GuidanceLaw::GuidanceLaw(ros::NodeHandle nh){
  // Init BODY velocity vector
  v_b = Vector2d::Zero();
  // Init NED velocity vector
  p_n_dot = Vector2d::Zero();
  // Init Rotation Matrix
  R_psi << cos(0.0), -sin(0.0),
            sin(0.0),  cos(0.0);

  f = (SEMIMAJORAXIS - SEMIMINORAXIS) / SEMIMAJORAXIS;
  R = SEMIMAJORAXIS;


  speed_controller_input_pub = nh.advertise<custom_msgs::SpeedControllerInput>( "speed_controller_input", 1);
  heading_controller_input_pub = nh.advertise<custom_msgs::HeadingControllerInput>( "heading_controller_input", 1);
  los_guidance_data_pub = nh.advertise<custom_msgs::LOSData>("los_data", 1);
  guidance_law_input_sub = nh.subscribe("guidance_law_input", 1, &GuidanceLaw::computeOutput, this);
  waypoint_sub = nh.subscribe("waypoint_list", 1, &GuidanceLaw::processWaypoints, this);
  ned_sub = nh.subscribe("ned_origin", 1, &GuidanceLaw::updateNEDOrigin, this);

  k = 0;
  yk=0;
  xk=0;
  yk_1=100;
  xk_1=100;
}

/**
 * Publishes heading reference to /heading_controller_input 
 * and speed reference to /speed_controller_input
 * */
void GuidanceLaw::computeOutput(const custom_msgs::GuidanceLawInput &input){
  // Create messages
  custom_msgs::HeadingControllerInput heading_msg;
  custom_msgs::SpeedControllerInput speed_msg;
  custom_msgs::LOSData los_msg;

  // Decompose Speed over ground U with angle chi and update NED velocity vector
  p_n_dot(0) = input.U*cos(input.chi*D2R); // velocity in north direction
  p_n_dot(1) = input.U*sin(input.chi*D2R); // velocity in east direction
  
  // Update 2D Rotation matrix
  updateRotationMatrix(input.psi*D2R);
  
  // Transform vector from NED to BODY
  v_b = R_psi.transpose()*p_n_dot;

  // Lookahead distance (12m)
  double delta_e = 12;

  // Calculate path tangential angle chi_p/a_k
  double a_k = atan2(yk_1 - yk, xk_1 - xk);
  
  // double e = -(input.N-xk)*sin(a_k) + (input.E-yk)*cos(a_k);
  
  Vector2d p_tilde;
  p_tilde << input.N-xk, input.E-yk;
  
  Matrix2d R_ak;
  R_ak << cos(a_k), -sin(a_k),
          sin(a_k), cos(a_k); 
  
  
  // Calculate cross-track error
  Vector2d se = R_ak.transpose()*p_tilde;
  // Calculate LOS steering law
  double chi_r = atan2(-se(1), delta_e);

  // Calculate desired course chi_d
  double chi_d = a_k + chi_r;

  // Calculate sideslip beta
  double beta;
  if(input.U < 0.2)
    beta = 0;
  else
    beta = asin(v_b(1)/input.U);
  //beta = 0.0;
  // GuidanceLaw Node outputs desired heading (psi_d = chi_d - beta)
  double psi_d = chi_d - beta;
  if(psi_d > M_PI)
    psi_d = psi_d - 2*M_PI;
  else if(psi_d < -M_PI)
    psi_d = psi_d + 2*M_PI;
  
  heading_msg.state = input.psi;
  heading_msg.setpoint = psi_d*R2D; 
  heading_msg.error = psi_d*R2D - input.psi;
  heading_msg.r = input.r;
  heading_msg.source = 1;
  speed_msg.u = v_b(0);
  speed_msg.v = v_b(1);
  speed_msg.u_ref = 1;

  los_msg.s = se(0);
  los_msg.e = se(1);
  los_msg.beta = beta;

  // Publish to topics
  heading_controller_input_pub.publish(heading_msg);
  speed_controller_input_pub.publish(speed_msg);
  los_guidance_data_pub.publish(los_msg);

   // Check to see if increment in setpoints is necessary
   double sk_1 = sqrt(pow(xk_1-xk, 2) + pow(yk_1 - yk, 2));
   
   std::cout << "LOS Guidance Info:" <<std::endl
   << "N: " << input.N << " E: " << input.E << " psi_d: " << psi_d*R2D <<std::endl
   << "e: " << se(1) << ", a_k: " << a_k*R2D << ", beta: " << beta*R2D << " chi_r: " << chi_r*R2D << std::endl
   << "wp_iterator: " << k << std::endl;
   std::cout << "sk_1: " << sk_1 << " s: " << se(0) << " sk_1-s: " << (sk_1-se(0)) << std::endl;
   if(sk_1 - se(0) < 16.0/*Rk_1*/){
     k+=2;
   }
}
/**
 * Callback for waypoint list
 * */
void GuidanceLaw::processWaypoints(const custom_msgs::ExtCommand &msg){
  for(int i = 0; i < msg.data.size(); i++){
    wp_list.push_back((double)msg.data[i]);
  }
  // NED position of the first waypoint
  std::cout.precision(9);
  std::vector<double> pk = getNEDcoordinates(wp_list[k]*D2R, wp_list[k+1]*D2R, lat0*D2R, lon0*D2R);
  //std::cout <<"pk: " << wp_list[k] << " " << wp_list[k+1] << std::endl;
  // NED position of the second waypoint
  std::vector<double> pk_1 = getNEDcoordinates(wp_list[k+2]*D2R, wp_list[k+3]*D2R, lat0*D2R, lon0*D2R);
  //std::cout <<"pk_1: " << wp_list[k+2] << " " << wp_list[k+3] << std::endl;

  xk = pk[0];
  yk = pk[1];
  
  xk_1 = pk_1[0];
  yk_1 = pk_1[1];

  std::cout <<"xk: " << xk << " yk: " << yk << std::endl;
  std::cout <<"xk_1: " << xk_1 << " yk_1: " << yk_1 << std::endl;
  std::cout << " " << std::endl << std::endl;
  wp_list.clear();
}

/**
 * Callback for NED origin
 * */
void GuidanceLaw::updateNEDOrigin(const custom_msgs::gnssGGA &msg){
  lat0 = msg.latitude;
  lon0 = msg.longitude;
  std::cout.precision(9);
  std::cout << "GUIDANCE NODE: Lat0: " << lat0 << " Lon0: " << lon0 << std::endl;
}

/**
 * Utility functions
 * */
void GuidanceLaw::updateRotationMatrix(const double psi){
  R_psi << cos(psi), -sin(psi),
           sin(psi),  cos(psi);
}

std::vector<double> GuidanceLaw::getNEDcoordinates(double lat, double lon, double latRef, double lonRef){
  double dMy = (lat - latRef);
  double dL = (lon - lonRef);

  double rN = (R / (sqrt(1 - (2 * f - f * f) * 
          pow(sin(latRef), 2))));
  double rM = rN * ((1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * 
         pow(sin(latRef), 2)));
  double dN = (dMy / atan(1 / rM));
  double dE = (dL / atan(1 / (rN * cos(latRef))));
  std::vector<double> returnVector;
  returnVector.push_back(dN);
  returnVector.push_back(dE);
  return returnVector;
}

/**
 * Main Function
 * */
int main(int argc, char **argv){
  ros::init(argc, argv, "GuidanceLawNode");
  ros::NodeHandle nh;
  GuidanceLaw guidanceLaw(nh);
  ros::spin();
  return 0;
}
