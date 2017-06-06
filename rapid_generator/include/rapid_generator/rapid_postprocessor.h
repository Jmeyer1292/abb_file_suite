#ifndef RAPID_POSTPROCESSOR_H
#define RAPID_POSTPROCESSOR_H

#include <vector>
#include <string>
#include <memory>
#include <sstream>

namespace rapid_generator
{

class ProgramInterface
{
public:
  std::ostringstream constants;
  std::ostringstream body;

  std::size_t nextIndex() { return idx++; }

private:
  std::size_t idx = 0;
};

class Instruction
{
public:
  virtual ~Instruction() {}
  virtual void generate(ProgramInterface& interface) const = 0;
};

class Program
{
public:
  Program();

  std::string generate() const;

  void add(std::unique_ptr<Instruction> instruction);

private:
  std::vector<std::unique_ptr<Instruction>> instructions_;
};

// Impl

class MoveJInstruction : public Instruction
{
public:
  // Radians
  MoveJInstruction(const std::vector<double>& joints, double duration);

  void generate(ProgramInterface& interface) const override;

private:
  std::vector<double> joints_;
  double duration_;
};

class SetIOInstruction : public Instruction
{
public:
  SetIOInstruction(const std::string& io_name, bool value);

  void generate(ProgramInterface& interface) const override;

private:
  std::string io_name_;
  bool value_;
};

class WaitInstruction : public Instruction
{
public:
  WaitInstruction(double duration); // seconds

  void generate(ProgramInterface& interface) const override;

private:
  double duration_;
};

}

#endif // RAPID_POSTPROCESSOR_H
