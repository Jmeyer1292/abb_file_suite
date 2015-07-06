#ifndef RAPID_DATA_STRUCTURES_H
#define RAPID_DATA_STRUCTURES_H

#include <vector>
#include <string>

namespace rapid_emitter
{

struct TrajectoryPt
{
  typedef std::vector<double> value_type;

  TrajectoryPt(const std::vector<double>& positions)
    : positions_(positions)
  {}

  value_type positions_;
};

struct ProcessParams
{
  double spindle_speed;
  double tcp_speed;
  double force;
  std::string output_name;
  bool wolf;
  double slide_force;
};

}

#endif // RAPID_DATA_STRUCTURES_H

