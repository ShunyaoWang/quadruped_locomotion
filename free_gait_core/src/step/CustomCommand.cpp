/*
 * CustomCommand.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_core/step/CustomCommand.hpp"

// STD ss
#include <iostream>

namespace free_gait {

CustomCommand::CustomCommand()
    : duration_(0.0)
{
}

CustomCommand::~CustomCommand()
{
}

std::unique_ptr<CustomCommand> CustomCommand::clone() const
{
  std::unique_ptr<CustomCommand> pointer(new CustomCommand(*this));
  return pointer;
}

void CustomCommand::setType(const std::string& type)
{
  type_ = type;//string
}

void CustomCommand::setDuration(const double duration)
{
  duration_ = duration;//double
}

void CustomCommand::setCommand(const std::string& command)
{
  command_ = command;//string
}

const std::string& CustomCommand::getType() const
{
  return type_;
}

const std::string& CustomCommand::getCommand() const
{
  return command_;
}

double CustomCommand::getDuration() const
{
  return duration_;
}

std::ostream& operator<<(std::ostream& out, const CustomCommand& customCommand)
{
  out << "Type: " << customCommand.type_ << std::endl;
  out << "Duration: " << customCommand.duration_ << std::endl;
  out << "Command: " << std::endl << customCommand.command_ << std::endl;
  return out;
}

} /* namespace */
