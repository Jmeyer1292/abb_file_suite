#include "abb_file_suite/abb_motion_ftp_downloader.h"

#include <fstream>

#include <ros/ros.h>

#include "rapid_generator/rapid_emitter.h"
#include "ftp_upload.h"

static inline double toDegrees(const double radians) { return radians * 180.0 / M_PI; }
static inline std::vector<double> toDegrees(const std::vector<double>& radians)
{
  std::vector<double> result;
  result.reserve(radians.size());
  for (std::size_t i = 0; i < radians.size(); ++i) result.push_back(toDegrees(radians[i]));
  return result;
}

static void linkageAdjust(std::vector<double>& joints)
{
  joints[2] += joints[1];
}

abb_file_suite::AbbMotionFtpDownloader::AbbMotionFtpDownloader(const std::string &ip,
                                                               const std::string &listen_topic,
                                                               ros::NodeHandle &nh,
                                                               bool j23_coupled)
  : ip_(ip)
  , j23_coupled_(j23_coupled)
{
  trajectory_sub_ = nh.subscribe(
        listen_topic,
        10,
        &AbbMotionFtpDownloader::handleJointTrajectory,
        this);

  server_ = nh.advertiseService("execute_program", &AbbMotionFtpDownloader::handleServiceCall, this);
}

void abb_file_suite::AbbMotionFtpDownloader::handleJointTrajectory(const trajectory_msgs::JointTrajectory &traj)
{
  ROS_INFO_STREAM("Trajectory received");
  // generate temporary file with appropriate rapid code
  std::ofstream ofh ("/tmp/mGodel_blend.mod");

  if (!ofh)
  {
    ROS_WARN_STREAM("Could not create file");
    return;
  }

  std::vector<rapid_emitter::TrajectoryPt> pts;
  pts.reserve(traj.points.size());
  for (std::size_t i = 0; i < traj.points.size(); ++i)
  {
    std::vector<double> tmp = toDegrees(traj.points[i].positions);
    if (j23_coupled_) linkageAdjust(tmp);
    pts.push_back(tmp);
  }

  rapid_emitter::ProcessParams params;
  params.wolf = false;
  rapid_emitter::emitJointTrajectoryFile(ofh, pts, params);
  ofh.close();

  // send to controller
  if (!uploadFile(ip_ + "/PARTMODULES", "/tmp/mGodel_blend.mod"))
  {
    ROS_WARN("Could not upload joint trajectory to remote ftp server");
  }
}

bool abb_file_suite::AbbMotionFtpDownloader::handleServiceCall(abb_file_suite::ExecuteProgram::Request &req,
                                                               abb_file_suite::ExecuteProgram::Response &res)
{
  ROS_INFO("Handling program execution request");
  // Check for existence
  std::ifstream ifh (req.file_path.c_str());
  if (!ifh)
  {
    ROS_WARN("Could not open file '%s'.", req.file_path.c_str());
    return false;
  }
  ifh.close();

  return uploadFile(ip_ + "/PARTMODULES", req.file_path.c_str());
}
