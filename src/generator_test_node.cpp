#include <ros/ros.h>

#include "rapid_generator/rapid_postprocessor.h"
#include <iostream>

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

int main()
{
  rapid_generator::Program program;

  std::vector<double> joints = {3.14, 0, 0, 0, 0, 0};

  program.add( make_unique<rapid_generator::MoveJInstruction>(joints, 0.0) );
  program.add( make_unique<rapid_generator::MoveJInstruction>(joints, 1.0) );
  program.add( make_unique<rapid_generator::MoveJInstruction>(joints, 2.0) );
  program.add( make_unique<rapid_generator::MoveJInstruction>(joints, 3.0) );

  program.add( make_unique<rapid_generator::WaitInstruction>(0.5) );
  program.add( make_unique<rapid_generator::SetIOInstruction>("dio8", true) );

  auto string = program.generate();

  std::cout << "Program:\n" << string << "\n";
}
