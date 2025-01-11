// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/HomePosition.h"

HomePosition::HomePosition(ElevatorSubsystem* subsystem) : m_elevator(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void HomePosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HomePosition::Execute() {
  m_elevator->HomePosition();
}

// Called once the command ends or is interrupted.
void HomePosition::End(bool interrupted) {}

// Returns true when the command should end.
bool HomePosition::IsFinished() {
  return false;
}
