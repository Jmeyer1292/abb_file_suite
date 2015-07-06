#include "rapid_generator/rapid_emitter.h"

#include <iostream>

bool rapid_emitter::emitRapidFile(std::ostream& os,
                                  const std::vector<TrajectoryPt>& points,
                                  size_t startProcessMotion,
                                  size_t endProcessMotion,
                                  const ProcessParams& params)
{
  // Write header
  os << "MODULE mGodel_Blend\n\n";
  // Emit all of the joint points
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    emitJointPosition(os, points[i], i);
  }
  // Emit Process Declarations
  emitProcessDeclarations(os, params, 1);
  // Write beginning of procedure
  os << "\nPROC TestProc()\n";
  // For 0 to lengthFreeMotion, emit free moves
  for (std::size_t i = 0; i < startProcessMotion; ++i)
  {
    if (i == 0)
    {
      emitFreeMotion(os, params, i, true, false);
    }
    else
    {
      emitFreeMotion(os, params, i);
    }
  }
  emitSetOutput(os, params, 0);
  // for lengthFreeMotion to end of points, emit grind moves
  for (std::size_t i = startProcessMotion; i < endProcessMotion; ++i)
  {
    if (i == startProcessMotion)
    {
      emitGrindMotion(os, params, i, true, false);
    }
    else if (i == endProcessMotion-1)
    {
      emitGrindMotion(os, params, i, false, true);
    }
    else
    {
      emitGrindMotion(os, params, i);
    }
  }
  emitSetOutput(os, params, 1);
  // for lengthFreeMotion to end of points, emit grind moves
  for (std::size_t i = endProcessMotion; i < points.size(); ++i)
  {
    if (i == points.size() - 1)
    {
      emitFreeMotion(os, params, i, false, true);
    }
    else
    {
      emitFreeMotion(os, params, i);
    }
    
  }

  os << "EndProc\n";
  // write any footers including main procedure calling the above
  os << "ENDMODULE\n";
  return true;
}


bool rapid_emitter::emitGrindMotion(std::ostream& os, const ProcessParams& params, size_t n, bool start, bool end)
{
  if (params.wolf)
  {
    if (start)
    {
      os << "GrindLStart CalcRobT(jTarget_" << n << ",tool0), v100, gr1, fine, tool0;\n";
    }
    else if (end)
    {
      os << "GrindLEnd CalcRobT(jTarget_" << n << ",tool0), v100, fine, tool0;\n";
    }
    else
    {
      os << "GrindL CalcRobT(jTarget_" << n << ",tool0), v100, z40, tool0;\n";
    }
  }
  else
  {
    os << "MoveL CalcRobT(jTarget_" << n << ",tool0), vProcessSpeed, z5, tool0;\n";
  }
  return os.good();
}

bool rapid_emitter::emitFreeMotion(std::ostream& os, const ProcessParams& params, size_t n, bool start, bool end)
{
  if (start || end)
  {
    os << "MoveJ CalcRobT(jTarget_" << n << ",tool0), vMotionSpeed, fine, tool0;\n";
  }
  else
  {
    os << "MoveL CalcRobT(jTarget_" << n << ",tool0), vMotionSpeed, z40, tool0;\n";
  }
  
  return os.good();
}
bool rapid_emitter::emitJointPosition(std::ostream& os, const TrajectoryPt& pt, size_t n)
{
  os << "TASK PERS jointtarget jTarget_" << n << ":=[[";
    for (size_t i = 0; i < pt.positions_.size() ; i++)
    {
      os << pt.positions_[i];
      if (i < pt.positions_.size()-1)
      {
        os << ",";
      }
    }
    
  os << "],[9E9,9E9,9E9,9E9,9E9,9E9]];\n";
  return true;
}

bool rapid_emitter::emitSetOutput(std::ostream& os, const ProcessParams& params, size_t value)
{
  if (params.wolf == false)
  {
    os << "WaitTime\\InPos, 0.01;\n";
    os << "SETDO " << params.output_name << ", " << value << ";\n";
  }
  return os.good();
} 

bool rapid_emitter::emitProcessDeclarations(std::ostream& os, const ProcessParams& params, size_t value)
{
  if (params.wolf)
  {
    os << "TASK PERS grinddata gr1:=[" << params.tcp_speed << "," << params.spindle_speed << "," << params.slide_force << ",FALSE,FALSE,FALSE,0,0];\n";
  }
  else
  {
    os << "CONST speeddata vProcessSpeed:=[" << params.tcp_speed << "," << params.tcp_speed << ",50,50];\n";
  }
   os << "CONST speeddata vMotionSpeed:=[" << "200" << "," << "30" << ",500,500];\n";
  return os.good();
}


bool rapid_emitter::emitJointTrajectoryFile(std::ostream &os, const std::vector<TrajectoryPt> &points, const ProcessParams &params)
{
  // Write header
  os << "MODULE mGodel_Blend\n\n";
  // Emit all of the joint points
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    emitJointPosition(os, points[i], i);
  }
  // Emit Process Declarations
  emitProcessDeclarations(os, params, 1);
  // Write beginning of procedure
  os << "\nPROC Godel_Blend()\n";
  // For 0 to lengthFreeMotion, emit free moves
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    emitFreeMotion(os, params, i, true);
  }

  os << "EndProc\n";
  // write any footers including main procedure calling the above
  os << "ENDMODULE\n";

  return os.good();
}
