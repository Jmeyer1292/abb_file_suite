#ifndef ABB_MOTION_FTP_DOWNLOADER_H
#define ABB_MOTION_FTP_DOWNLOADER_H

#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "abb_file_suite/ExecuteProgram.h"

namespace abb_file_suite
{

class AbbMotionFtpDownloader
{
public:
  AbbMotionFtpDownloader(const std::string& ip,
                         const std::string& listen_topic,
                         ros::NodeHandle& nh,
                         bool j23_coupled = false);

  void handleJointTrajectory(const trajectory_msgs::JointTrajectory& traj);

  bool handleServiceCall(abb_file_suite::ExecuteProgram::Request& req,
                         abb_file_suite::ExecuteProgram::Response& res);

private:
  ros::Subscriber trajectory_sub_;
  ros::ServiceServer server_;
  const std::string ip_;
  bool j23_coupled_; /** joints 2 and 3 are coupled (as in ABB IRB2400) */
};

}

#endif
