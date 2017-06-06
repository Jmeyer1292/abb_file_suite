#include "rapid_generator/rapid_postprocessor.h"
#include <stdexcept>
#include <cmath>

// Program
rapid_generator::Program::Program()
{

}

std::string rapid_generator::Program::generate() const
{
  ProgramInterface iface;
  for (const auto& inst : instructions_)
  {
    inst->generate(iface);
  }

  std::ostringstream whole_program;
  whole_program << "MODULE mGodel_Blend\n\n";

  whole_program << iface.constants.str();

  whole_program << "\nPROC Godel_Blend()\n";

  whole_program <<  iface.body.str();

  whole_program << "EndProc\n";

  whole_program << "ENDMODULE\n";

  return whole_program.str();
}

void rapid_generator::Program::add(std::unique_ptr<rapid_generator::Instruction> instruction)
{
  if (!instruction)
  {
    throw std::invalid_argument("instructions may not be null");
  }

  instructions_.push_back(std::move(instruction));
}

// MoveJ
static double toDegrees(const double radians) { return radians * 180.0 / M_PI; }

rapid_generator::MoveJInstruction::MoveJInstruction(const std::vector<double> &joints, double duration)
  : joints_(joints), duration_(duration)
{
  for (auto& v : joints_)
    v = toDegrees(v);
}

void rapid_generator::MoveJInstruction::generate(rapid_generator::ProgramInterface &interface) const
{
  const auto n = interface.nextIndex();

  // write constant
  interface.constants << "TASK PERS jointtarget jTarget_" << n << ":=[[";
  for (size_t i = 0; i < joints_.size(); i++)
  {
    interface.constants << joints_[i];
    if (i < joints_.size() - 1) interface.constants << ",";
  }

  // We assume a six axis robot here.
  interface.constants << "],[9E9,9E9,9E9,9E9,9E9,9E9]];\n";

  // write body
  const static auto zone = "z20";
  interface.body << "MoveAbsJ jTarget_" << n << ", v200, \\T:=" << duration_ << ", " << zone << ", tool1;\n";
}

// Move
rapid_generator::SetIOInstruction::SetIOInstruction(const std::string &io_name, bool value)
  : io_name_(io_name), value_(value)
{
}

void rapid_generator::SetIOInstruction::generate(rapid_generator::ProgramInterface &interface) const
{
  const char* const text_value = value_ ? "1" : "0";
  interface.body << "SETDO " << io_name_ << ", " << text_value << ";\n";
}

rapid_generator::WaitInstruction::WaitInstruction(double duration)
  : duration_(duration)
{
}

void rapid_generator::WaitInstruction::generate(rapid_generator::ProgramInterface &interface) const
{
  interface.body << "WaitTime\\InPos," << duration_ << ";\n";
}
