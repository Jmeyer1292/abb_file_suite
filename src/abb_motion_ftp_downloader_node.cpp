#include "abb_file_suite/abb_motion_ftp_downloader.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_motion_ftp_donwloader");

  ros::NodeHandle nh, pnh("~");

  // load parameters
  std::string ip, topic;
  bool j23_coupled;
  pnh.param<std::string>("controller_ip", ip, "");
  pnh.param<std::string>("topic", topic, "joint_path_command");
  nh.param<bool>("J23_coupled", j23_coupled, false);

  abb_file_suite::AbbMotionFtpDownloader ftp(ip, topic, nh, j23_coupled);

  ROS_INFO("ABB FTP-Trajectory-Downloader (ip %s) initialized. Listening on topic %s.", ip.c_str(), topic.c_str());
  ros::spin();

  return 0;
}
