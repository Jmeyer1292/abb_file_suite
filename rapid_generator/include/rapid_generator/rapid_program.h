#ifndef RAPID_PROGRAM_H
#define RAPID_PROGRAM_H

#include <string>

namespace rapid_generator
{

class RapidProgram
{
public:
  RapidProgram(const std::string& module_name);


};

}

#endif // RAPID_PROGRAM_H

int main()
{
  rapid_generator::RapidProgram pgm;

  pgm << Module("mGodel_blend") << ""
}
