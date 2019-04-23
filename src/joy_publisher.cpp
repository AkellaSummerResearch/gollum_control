#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <tf/tf.h>
#include "vicon_driver/vicon_driver.h"
#include "vicon_driver/vicon_calib.h"
#include "vicon/Subject.h"
#include "vicon/Markers.h"
#include "vicon/SetPose.h"
#include <math.h>
#include <std_msgs/Int16MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <mg_msgs/follow_PolyPVA_XY_trajectoryAction.h>
#include "polynomials.h"
#define pi 3.14159265

float radius = 1,rad = 2*pi/60;
ros::Subscriber vicon_sub;
double yaw,x,y;
double Xc,Yc,rightw,leftw;
ros::Publisher vel_pub,ref_pub;
float r = 0.05;
float D = 0.05;
float L = 0.05;
geometry_msgs::PoseStamped reftraj;
using namespace tucker_polynomials;
Trajectory2D trajectory;

typedef actionlib::SimpleActionServer<mg_msgs::follow_PolyPVA_XY_trajectoryAction> Server;

double Xt(double time)
{
  Eigen::Vector2d test = trajectory.TrajAtTime(time);
  return test(0);
  //return radius*cos(time*rad);
}

double Yt(double time)
{
  Eigen::Vector2d test = trajectory.TrajAtTime(time);
  return test(1);
  //return radius*sin(time*rad);
}

double dXt(double time)
{
  Eigen::Vector2d test = trajectory.TrajDiffAtTime(time);
  return test(0);
  //return -rad*radius*sin(time*rad);
}

double dYt(double time)
{
  Eigen::Vector2d test = trajectory.TrajDiffAtTime(time);
  return test(1);
  //return rad*radius*cos(time*rad);
}

void executeCB(const mg_msgs::follow_PolyPVA_XY_trajectoryGoalConstPtr &goal, Server* as) {
  /*std::vector<mg_msgs::PolyPVA> segments_x, segments_y;

  for(int i = 0;i<goal->X.size();i++){
  segments_x.push_back(goal->X[i]);
  segments_y.push_back(goal->Y[i]);
  }*/
  //Trajectory2D trajectory(segments_x,segments_y);
  trajectory.InitTraj(goal->X,goal->Y);
  ros::Rate loop_rate(10);
  ros::Time start = ros::Time::now();
  ros::Duration d;
  double time;
  std_msgs::Int16MultiArray twist;
  float rho = D/L;
  double J11,J12,J21,J22;
  reftraj.header.frame_id = "map";
  mg_msgs::follow_PolyPVA_XY_trajectoryFeedback feedback;
  while(ros::ok() && time<trajectory.tf_ && !as->isPreemptRequested()){
    d = ros::Time::now() - start;
    time = d.toSec();
    
    //Make the Jinv matrix
    J11 = (1/r)*(cos(yaw)+sin(yaw)/rho);
    J12 = (1/r)*(sin(yaw)-cos(yaw)/rho);
    J21 = (1/r)*(cos(yaw)-sin(yaw)/rho);
    J22 = (1/r)*(sin(yaw)+cos(yaw)/rho);
    
    //Publish a reference trajectory to monitor on rviz
    reftraj.pose.position.x = Xt(time);
    reftraj.pose.position.y = Yt(time);
    reftraj.pose.position.z = 0;
    reftraj.pose.orientation.w = cos(time*rad/2.0);
    reftraj.pose.orientation.z = sin(-time*rad/2.0);

    //Calculate wheel speeds
    Xc = dXt(time) + 20*(Xt(time) - x);
    Yc = dYt(time) + 20*(Yt(time) - y);
    leftw = -(J11*Xc + J12*Yc);
    rightw = -(J21*Xc + J22*Yc);
    
    //Saturate wheel speeds and push it into the topic type
    if (leftw>255) leftw = 255;
    if (leftw<-255) leftw = -255;
    if (rightw>255) rightw = 255;
    if (rightw<-255) rightw = -255;
    twist.data.clear();
    twist.data.push_back(leftw);
    twist.data.push_back(rightw);
    twist.data.push_back(0);
    twist.data.push_back(0);
    
    //Publish necessary topics
    vel_pub.publish(twist);
    ref_pub.publish(reftraj);
    //ros::spinOnce();
    feedback.currentState.Pos.x = x;
    feedback.currentState.Pos.y = y;
    feedback.currentState.yaw = yaw;
    as->publishFeedback(feedback);
    loop_rate.sleep();
  }
  as->setSucceeded();
}


void viconCallback(vicon::Subject rover)
{
  x = rover.position.x;
  y = rover.position.y;

  tf::Quaternion q(rover.orientation.x,rover.orientation.y,rover.orientation.z,rover.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch;
  m.getRPY(roll,pitch,yaw);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh;
  Server as(nh, "Gollum", boost::bind(&executeCB, _1, &as), false);
  
  vicon_sub = nh.subscribe("/vicon/Gollum", 100, viconCallback);
  vel_pub = nh.advertise<std_msgs::Int16MultiArray>("/cmd",10);
  ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/ref_traj",10);

  ros::spin();
  
}
