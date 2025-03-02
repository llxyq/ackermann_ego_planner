#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>
// #include <tf/transformation_datatype.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Publisher pos_cmd_pub;
ros::Publisher pos_vel_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;
using namespace std;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
double robot_yaw_world_;

geometry_msgs::Pose odom_Pose;
Eigen::Vector3f odom_vel(0,0,0);

bool goFlag_=0;
void goFlagCallback(const std_msgs::Int16::ConstPtr msg);

ros::Publisher control_point_state_pub_;
nav_msgs::Odometry control_point_state;
double LimitSpeed(const double vel_input,const double upper,const double lower);

void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  double yaw_control_point = tf::getYaw(control_point_state.pose.pose.orientation);
  double yaw_robot = tf::getYaw(odom_Pose.orientation);

  double yaw_diff = yaw_control_point- yaw_robot;
  if (yaw_diff > PI) yaw_diff = yaw_diff - 2 * PI;
  else if (yaw_diff < -PI) yaw_diff = yaw_diff + 2 * PI;
  yaw_yawdot.first = yaw_diff;
  yaw_yawdot.second = yaw_diff /  t_cur;

  // cout << "yaw  " << yaw_control_point*180/3.14 << "  " <<yaw_robot*180/3.14 << "  " <<yaw_diff*180/3.14 << endl;
  return yaw_yawdot;
}

void goFlagCallback(const std_msgs::Int16::ConstPtr msg)
{
  goFlag_ = msg->data;

}
void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  geometry_msgs::Twist robotVelocity_BASE_frame;
  if(!goFlag_)
  {
    robotVelocity_BASE_frame.linear.x = 0-odom_vel(0);
    robotVelocity_BASE_frame.linear.y = 0-odom_vel(1);
    robotVelocity_BASE_frame.linear.z = 0;

    robotVelocity_BASE_frame.angular.x = 0;
    robotVelocity_BASE_frame.angular.y = 0;
    robotVelocity_BASE_frame.angular.z = 0;

    pos_vel_pub.publish(robotVelocity_BASE_frame);
    // cout << "velocity: " << robotVelocity_BASE_frame.linear.x << "  " << robotVelocity_BASE_frame.linear.y << "  " << robotVelocity_BASE_frame.angular.z <<endl;
    return ;
  }
  

  ros::Time time_now = ros::Time::now();
  // double t_cur = (time_now - start_time_).toSec();
  double t_cur = 1.3;

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0); //FIXME:why +2
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.yaw = yaw_yawdot.first;
 
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.yaw_dot = yaw_yawdot.second;

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);

  // ------------ estimation q from traj ------------- // 

  control_point_state.header.frame_id = "world";
  double yaw_estimate_from_traj = 0, PI = 3.14159;
  if (vel(0)<0 && vel(1)>0) yaw_estimate_from_traj = atan(vel(1)/(vel(0)+1e-6)) + PI;
  else if (vel(0)<0 && vel(1)<0) yaw_estimate_from_traj = atan(vel(1)/(vel(0)+1e-6)) - PI;
  else yaw_estimate_from_traj = atan(vel(1)/(vel(0)+1e-6));
  Eigen::Vector3d eulerAngle(yaw_estimate_from_traj,0,0);
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
  
  Eigen::Quaterniond q2;
  q2=yawAngle*pitchAngle*rollAngle;

  control_point_state.pose.pose.position.x = pos(0);
  control_point_state.pose.pose.position.y = pos(1);
  control_point_state.pose.pose.position.z = pos(2);
  control_point_state.pose.pose.orientation.x = q2.x();
  control_point_state.pose.pose.orientation.y = q2.y();
  control_point_state.pose.pose.orientation.z = q2.z();
  control_point_state.pose.pose.orientation.w = q2.w();
    
  control_point_state_pub_.publish(control_point_state);
  // ------------ trans vel ------------- // 

  geometry_msgs::Twist velWorld;
  velWorld.linear.x = vel(0);
  velWorld.linear.y = vel(1);
  velWorld.linear.z = 0;
  velWorld.angular.x = 0;
  velWorld.angular.y = 0;
  velWorld.angular.z = yaw_yawdot.second;

  // transform velocity to base
  Eigen::Matrix<double, 3, 1> basePosWorldCur;
  Eigen::Matrix<double, 3, 1> baseLinearVelWorldCur;
  Eigen::Matrix<double, 3, 1> baseLinearVelBodyCur;
  Eigen::Quaterniond baseOriWorldCur;
  Eigen::Matrix<double, 3, 1> baseAngularVelWorldCur;
  Eigen::Matrix<double, 3, 1> baseAngularVelBodyCur;
  
  basePosWorldCur[0] =  odom_Pose.position.x;
  basePosWorldCur[1] =  odom_Pose.position.y;
  basePosWorldCur[2] =  odom_Pose.position.z;
  baseOriWorldCur.w() = odom_Pose.orientation.w;
  baseOriWorldCur.x() = odom_Pose.orientation.x;
  baseOriWorldCur.y() = odom_Pose.orientation.y;
  baseOriWorldCur.z() = odom_Pose.orientation.z;

  baseLinearVelWorldCur[0]  = velWorld.linear.x;
  baseLinearVelWorldCur[1]  = velWorld.linear.y;
  baseLinearVelWorldCur[2]  = velWorld.linear.z;
  baseAngularVelWorldCur[0] = velWorld.angular.x;
  baseAngularVelWorldCur[1] = velWorld.angular.y;
  baseAngularVelWorldCur[2] = velWorld.angular.z;

  // cout<< baseOriWorldCur.x() << " " <<baseOriWorldCur.y() << " " <<  baseOriWorldCur.z() << " " <<baseOriWorldCur.w() << endl;
  baseLinearVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseLinearVelWorldCur;
  baseAngularVelBodyCur = baseOriWorldCur.toRotationMatrix().transpose() * baseAngularVelWorldCur;

  // write to cmd vel
  // geometry_msgs::Twist robotVelocity_BASE_frame;
  // robotVelocity_BASE_frame.linear.x = baseLinearVelBodyCur[0]-odom_vel(0);
  // robotVelocity_BASE_frame.linear.y = baseLinearVelBodyCur[1]-odom_vel(1);
  // robotVelocity_BASE_frame.linear.z = 0;

  // robotVelocity_BASE_frame.angular.x = 0;
  // robotVelocity_BASE_frame.angular.y = 0;
  // robotVelocity_BASE_frame.angular.z = baseAngularVelBodyCur[2]-odom_vel(2);

  
  // robotVelocity_BASE_frame.linear.x = LimitSpeed(baseLinearVelBodyCur[0],0.2,-0.2);
  // robotVelocity_BASE_frame.linear.y = LimitSpeed(baseLinearVelBodyCur[1],0.2,-0.2);
  // robotVelocity_BASE_frame.linear.z = 0;

  // robotVelocity_BASE_frame.angular.x = 0;
  // robotVelocity_BASE_frame.angular.y = 0;
  // robotVelocity_BASE_frame.angular.z = LimitSpeed(baseAngularVelBodyCur[2],0.1,-0.1);

  robotVelocity_BASE_frame.linear.x = baseLinearVelBodyCur[0];
  robotVelocity_BASE_frame.linear.y = baseLinearVelBodyCur[1];
  robotVelocity_BASE_frame.linear.z = 0;

  robotVelocity_BASE_frame.angular.x = 0;
  robotVelocity_BASE_frame.angular.y = 0;
  robotVelocity_BASE_frame.angular.z = baseAngularVelBodyCur[2];


  // robotVelocity_BASE_frame.linear.x = 0;
  // robotVelocity_BASE_frame.linear.y = 0;
  // robotVelocity_BASE_frame.linear.z = 0;

  // robotVelocity_BASE_frame.angular.x = 0;
  // robotVelocity_BASE_frame.angular.y = 0;
  // robotVelocity_BASE_frame.angular.z = 0;

  // if(goFlag_==-1)
  // {
  // cout << "no new message flag come" << endl;
  // robotVelocity_BASE_frame.linear.x = 0;
  // robotVelocity_BASE_frame.linear.y = 0;
  // robotVelocity_BASE_frame.linear.z = 0;

  // robotVelocity_BASE_frame.angular.x = 0;
  // robotVelocity_BASE_frame.angular.y = 0;
  // robotVelocity_BASE_frame.angular.z = 0;

  // }

  cout << "velocity: " << robotVelocity_BASE_frame.linear.x << "  " << robotVelocity_BASE_frame.linear.y << "  " << robotVelocity_BASE_frame.angular.z <<endl;

  pos_vel_pub.publish(robotVelocity_BASE_frame);
  // goFlag_ = -1;

}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_Pose.position.x = msg->pose.pose.position.x;
  odom_Pose.position.y = msg->pose.pose.position.y;
  odom_Pose.position.z = msg->pose.pose.position.z;

  odom_Pose.orientation.x = msg->pose.pose.orientation.x;
  odom_Pose.orientation.y = msg->pose.pose.orientation.y;
  odom_Pose.orientation.z = msg->pose.pose.orientation.z;
  odom_Pose.orientation.w = msg->pose.pose.orientation.w;

  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z);

  robot_yaw_world_ =q.matrix().eulerAngles(2,1,0)(0);

  // odom_vel(0) = msg->twist.twist.linear.x;
  // odom_vel(1) = msg->twist.twist.linear.y;
  // odom_vel(2) = msg->twist.twist.angular.z;
  odom_vel(0) = -0.02;
  odom_vel(1) = -0.008;
  odom_vel(2) = 0;
}

double LimitSpeed(const double vel_input,const double upper,const double lower)
{
  double vel_output;
  if(vel_input > upper) vel_output = upper;
  else if(vel_input < lower) vel_output = lower;
  else vel_output = vel_input;

  return vel_output;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

control_point_state.pose.pose.orientation.w=1;

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  ros::Subscriber legOdom_sub = node.subscribe("legOdom", 10, odometryCallback);

  ros::Subscriber goFlagSub = node.subscribe("ego_planner_node/go_flag", 10, goFlagCallback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  pos_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.1), cmdCallback);

  control_point_state_pub_ = node.advertise<nav_msgs::Odometry>("/control_point_state_", 10);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}