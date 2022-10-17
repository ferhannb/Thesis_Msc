#include "tcp_datatransceiver/TCPDatatransceiver.h"

using namespace std;

TCPDatatransceiver::TCPDatatransceiver(int portNum, ros::NodeHandle nh){
  // Init control mode subscription here
  control_mode = 1;
  control_mode_sub = nh.subscribe("control_mode", 10, &TCPDatatransceiver::controlModeCallback, this);
  init_connection(portNum);

  // Subscribers initialization
  geodeticPosition_sub = nh.subscribe("vectorVS330/fix", 10, &TCPDatatransceiver::geodeticPositionCallback, this);
  heading_sub = nh.subscribe("vectorVS330/heading", 10, &TCPDatatransceiver::headingCallback, this);
  courseAndSpeed_sub = nh.subscribe("vectorVS330/velocity", 10, &TCPDatatransceiver::courseAndSpeedCallback, this);
  batteryVoltage_sub = nh.subscribe("battery_voltage", 10, &TCPDatatransceiver::batteryVoltageCallback, this);
  bowSpeedAndDirection_sub = nh.subscribe("bow_control", 10, &TCPDatatransceiver::bowSpeedAndDirectionCallback, this);
  sternSpeed_sub = nh.subscribe("stern_thruster_setpoints", 10, &TCPDatatransceiver::sternSpeedCallback, this);
  sternAngle_sub = nh.subscribe("pod_angle_input", 10, &TCPDatatransceiver::sternAngleCallback, this);

  // Publisher initialization
  external_command_pub = nh.advertise<custom_msgs::ExtCommand>("external_command", 10);


  // Initialization of storage variables
  latitude = 0.0;
  longitude = 0.0;
  vessel_speed = 0.0;
  heading = 0.0;
  course = 0.0;
  battery_voltage = 0.0;
  star_rpm = 0.0;
  port_rpm = 0.0;
  bow_rpm = 0.0;
  star_deg = 0.0;
  port_deg = 0.0;
  bow_deg = 0.0;

  north_pos = 1.10;
  east_pos = 3.2;
  crosstrack_error = 10.0;
  alongtrack_distance = 23.0;
  current_wp_number = 2;

  stop = 0;
  begin = ros::Time::now();
}

void TCPDatatransceiver::init_connection(int portNum){
  try{
      sock = new TCPServerSocket(portNum);
      ROS_INFO("TCP: Server for external control started at port %d", portNum);
      serverSocket = sock->accept();
      ROS_INFO("TCP: Client at %s:%d connected", serverSocket->getForeignAddress().c_str(), serverSocket->getForeignPort());
  }
  catch(SocketException se){
    cerr << se.what() << endl;
    exit(0);
  }
}

void TCPDatatransceiver::spin(){

  char sendBuffer[SENDBUFSIZE];

  int bytesReceived;
  ros::Rate rate(10);
  do {
    try{
      // Fill and send buffer to socket
      fillDatabuffer(sendBuffer);
      serverSocket->send(sendBuffer, SENDBUFSIZE);

      // Receive control signals
      bytesReceived = serverSocket->recv(recvBuffer, RECVBUFSIZE);
      recvBuffer[bytesReceived] = '\0';
      decodeExternalControlSignals(recvBuffer);
    }
    catch(SocketException se){

    }
    ros::spinOnce();
    rate.sleep();
  } while(ros::ok() && stop != 1);
  ROS_INFO("TCP: Client at %s:%d is disconnecting...", serverSocket->getForeignAddress().c_str(), serverSocket->getForeignPort());
  ROS_INFO("TCP: Socket successfully closed");
  delete sock;
  delete serverSocket;
}

void TCPDatatransceiver::decodeExternalControlSignals(char *recvBuffer){
  ostringstream oss;
  oss << recvBuffer;
  string recvString = oss.str();
  string mode = recvString.substr(0,2);
  replace(recvString.begin()+3, recvString.end(), ':', ' ');
  recvString = recvString.substr(3, recvString.size()-1);
  stringstream ss(recvString);
  // cout << ss << endl;
  vector<double> array;
  double temp;
  while(ss >> temp){
    array.push_back(temp);
  }
  if(mode.compare("HA") == 0){
    control_mode = 1;
  }
  else if(mode.compare("DP") == 0){
    control_mode = 3;
  }
  else if(mode.compare("GC") == 0){
    control_mode = 4;
    processWaypoints(array);
  }
  else if(mode.compare("EX") == 0){
    // signal to disable external commands
    control_mode = 6;
  }
  else if(mode.compare("ST") == 0)
  {
    // Stops the node, respawns after ~2 seconds
    stop = 1;
    return;
  }
  custom_msgs::ExtCommand external_command_msg;
  external_command_msg.mode = control_mode;
  
  for(int i = 0; i < array.size(); i++)
    external_command_msg.data.push_back(array[i]);

  external_command_pub.publish(external_command_msg);

}

void TCPDatatransceiver::processWaypoints(std::vector<double> array){

}

/**
* Creates the output stream to send to the RMC Station
* Fields in array:
* 0  - Latitude
* 1  - Longitude
* 2  - Heading
* 3  - Course
* 4  - Speed
* 5  - Battery Voltage
* 6  - Bow RPM
* 7  - Starboard RPM
* 8  - Port RPM
* 9  - Bow Angle Degree
* 10 - Starboard Angle Degree
* 11 - Port Angle Degree
* 12 - Control Mode
* 13 - Operator Override
* 14 - RC Remote Heading Ref
* 15 - RC Remote Speed Ref
*
* 16 - North Position
* 17 - East Position
* 18 - Crosstrack-Error
* 19 - Alongtrack-Distance
* 20 - Waypoint Number Currently Tracked
**/
void TCPDatatransceiver::fillDatabuffer(char *buffer){
  ostringstream oss;
  // Create the store all data in the line
  oss.precision(9);
  // Vessel Data
  oss
  << latitude     << ":" << longitude       << ":" << heading       << ":" << course   << ":"
  << vessel_speed << ":" << battery_voltage << ":" << bow_rpm       << ":" << star_rpm << ":"
  << port_rpm     << ":" << bow_deg         << ":" << star_deg      << ":" << port_deg << ":"
  << control_mode << ":" << op_override     << ":" << rc_heading_ref<< ":" << rc_speed_ref << ":";

  // Path Following Data
  oss
  << north_pos << ":" << east_pos << ":" << crosstrack_error << ":" << alongtrack_distance << ":"
  << current_wp_number << ":";

  // Collision Avoidance Data (Currently Max two obstacles tracked simultaneously)
  // set true to simulate a collision
  // << "obs1_id" << ":" << "obs1_N" << ":" << "obs1_E" << ":" << "obs1_vel" << ":" "obs1_course" << ":" << "radius" <<":";
  // Quick solution for simulating obstacles
  if(/*ros::Time::now().toSec() - begin.toSec() > 20*/false){
    oss << 1 << ":" << 1.4 << ":" << 3.2 << ":" << 0.0 << ":" << 0.0 << ":" << 5.0 << ":";
  }
  if(false){
    oss
    << "obs2_id" << ":" << "obs2_N" << ":" << "obs2_E" << ":" << "obs2_vel" << ":" << "obs2_course" << ":";
  }


  // appropriate conversions
  string var = oss.str();
  const char *var2 = var.c_str();
  // copy string to buffer
  strcpy(buffer, var2);
}

void TCPDatatransceiver::controlModeCallback(const std_msgs::UInt8& msg){
  control_mode = msg.data;
  ROS_INFO("Control mode callback in TCPDatatransceiver changed to %d", control_mode);
}

void TCPDatatransceiver::geodeticPositionCallback(const custom_msgs::gnssGGA &geoPosMsg){
  latitude = geoPosMsg.latitude;
  longitude = geoPosMsg.longitude;
  // ROS_INFO("In geodeticPositionCallback()");
}
void TCPDatatransceiver::headingCallback(const custom_msgs::gnssHDT &headingMsg){
  heading = headingMsg.heading;
  // ROS_INFO("In headingCallback()");
}
void TCPDatatransceiver::courseAndSpeedCallback(const custom_msgs::gnssRMC &courseAndSpeedMsg){
  course = courseAndSpeedMsg.course;
  vessel_speed = courseAndSpeedMsg.speed_mps;
  // ROS_INFO("In courseAndSpeedCallback()");
}
void TCPDatatransceiver::batteryVoltageCallback(const std_msgs::Float64 &battvolMsg){
  battery_voltage = battvolMsg.data;
  // ROS_INFO("In batteryVoltageCallback()");
}
void TCPDatatransceiver::bowSpeedAndDirectionCallback(const custom_msgs::bowControl &bowMsg){
  bow_rpm = bowMsg.throttle_bow;
  bow_deg = bowMsg.position_bow;
  // ROS_INFO("In bowSpeedAndDirectionCallback()");
}
void TCPDatatransceiver::sternSpeedCallback(const custom_msgs::SternThrusterSetpoints &sternMsg){
  star_rpm = sternMsg.star_effort;
  port_rpm = sternMsg.port_effort;
  // ROS_INFO("In sternSpeedCallback()");
}
void TCPDatatransceiver::sternAngleCallback(const custom_msgs::podAngle &stepperMsg){
  star_deg = stepperMsg.star;
  port_deg = stepperMsg.port;
  // ROS_INFO("In sternAngleCallback()");
}

/**
* Due to accept() function, OS needs to escalate to SIGTERM
* to kill node. Signal handler in Node now reacts to SIGINT (ctrl+c)
*/
void my_handler(int s){
 ROS_INFO("Caught signal %d\n",s);
 exit(1);
}

void mySignalHandler()
{

   struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "TCPDatatransceiver");
  ros::NodeHandle nh;
  mySignalHandler();
  TCPDatatransceiver dt(2345, nh);
  dt.spin();
  return 0;
}