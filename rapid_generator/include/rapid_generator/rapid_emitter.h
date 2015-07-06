#ifndef RAPID_EMITTER_H
#define RAPID_EMITTER_H

#include "rapid_generator/rapid_data_structures.h"

#include <iosfwd>
#include <string>

namespace rapid_emitter
{
  bool emitJointPosition(std::ostream& os, const TrajectoryPt& pt, size_t n);
  bool emitGrindMotion(std::ostream& os, const ProcessParams& params, size_t n, bool start = false, bool end = false);
  bool emitFreeMotion(std::ostream& os, const ProcessParams& params, size_t n, bool start = false, bool end = false);
  bool emitSetOutput(std::ostream& os, const ProcessParams& params, size_t value);
  bool emitProcessDeclarations(std::ostream& os, const ProcessParams& params, size_t value);

  bool emitRapidFile(std::ostream& os,
                     const std::vector<TrajectoryPt>& points,
                     size_t startProcessMotion,
                     size_t endProcessMotion,
                     const ProcessParams& params);

  bool emitJointTrajectoryFile(std::ostream& os,
                               const std::vector<TrajectoryPt>& points,
                               const ProcessParams& params);
}

#endif
