#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "custom_msgs/SpeedControllerInput.h"
#include "custom_msgs/ExtCommand.h"
#include "custom_msgs/HeadingControllerInput.h"
#include "custom_msgs/GuidanceLawInput.h"
#include "custom_msgs/gnssGGA.h"
#include "custom_msgs/LOSData.h"

#define D2R M_PI/180.0
#define R2D 180.0/M_PI

#define SEMIMAJORAXIS 6378137.0
#define SEMIMINORAXIS 6356752.0

using namespace Eigen;

class GuidanceLaw{
  public:
    explicit GuidanceLaw(ros::NodeHandle nh);
  private:
    ros::Publisher speed_controller_input_pub;
    ros::Publisher heading_controller_input_pub;
    ros::Publisher los_guidance_data_pub;
    ros::Subscriber guidance_law_input_sub;
    ros::Subscriber waypoint_sub;
    ros::Subscriber ned_sub;

    void computeOutput(const custom_msgs::GuidanceLawInput &input);
    void updateRotationMatrix(const double psi);
    std::vector<double> getNEDcoordinates(double lat, double lon, double latRef, double lonRef);

    // Callback for waypoint topic
    void processWaypoints(const custom_msgs::ExtCommand &msg);

    // Callback for NED origin update
    void updateNEDOrigin(const custom_msgs::gnssGGA &msg);

    // Velocity vector expressed in BODY
    Vector2d v_b;

    // Velocity vector expressed in NED
    Vector2d p_n_dot;
    
    // 2D Rotation Matrix
    Matrix2d R_psi;

    std::vector<double> wp_list;

    double yk_1;
    double yk;
    double xk_1;
    double xk;
    // waypoint number selector
    double k;

    double lat0;
    double lon0;
    double f;
    double R;

};
