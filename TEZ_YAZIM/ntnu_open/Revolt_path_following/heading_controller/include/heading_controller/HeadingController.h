#include <ros/ros.h>
#include <custom_msgs/HeadingControllerInput.h>
#include "custom_msgs/HeadingControllerData.h"
#include <std_msgs/Float64.h>
#include "custom_msgs/EnableHeadingController.h"
#include <Eigen/Dense>
#include <cmath>
 #include <tgmath.h>

#define D2R M_PI/180.0
#define R2D 180.0/M_PI

using namespace Eigen;

class HeadingController{
  public:
  explicit HeadingController(ros::NodeHandle nh);

  private:
    /* Control algorithm callback */
    void computeControlInput(const custom_msgs::HeadingControllerInput &input);

    /* Time variables */
    double h;
    ros::Time prev_time;
    ros::Duration dt;

    /* Autopilot Reference Model */

    // Method for computing reference filter states, psi_r = heading reference
    Vector3d referenceFilter(const double psi_r);
    double wrapToInf(double psi_r);
    double wrapToPi(double psi_continuous);
    bool setReferenceFilterInitialCondition(custom_msgs::EnableHeadingController::Request &req,
                            custom_msgs::EnableHeadingController::Response &res);
    Matrix3d A_d;
    Vector3d B_d;
    Vector3d m_x_d; // x_d = [psi_d, r_d, r_d_dot]^T  "m" for member field
    double r_max;
    double r_dot_max;
    double psi_last; // reference signal that doesn't wrap
    double accumulation;
    double accumulate;
    int state;
    bool filterInit;

    /*Nomoto Model parameters */
    double K;
    double T;

    /* Feedback variables and parameters*/
    double K_p;
    double K_i;
    double K_d;
    double psi_tilde;
    double r_tilde;
    double psi_tilde_integral;

    double prev_psi_d;

    /* Feedforward method*/
    double computeFeedforward(const double r_d, const double r_d_dot);

    /* Maneuvering variables*/
    // Vessel heading
    double psi;
    // Vessel Yaw rate
    double r;

    /* Publishers */
    // Total control input to rudder
    ros::Publisher controlInputPub;
    // Heading controller data publisher (for logging)
    ros::Publisher hcDataPub;

    /* Subscribers */
    ros::Subscriber stateSub;
    /* Service */
    ros::ServiceServer enable_heading_controller_service;


};
