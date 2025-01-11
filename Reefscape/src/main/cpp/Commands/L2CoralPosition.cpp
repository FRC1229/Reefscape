// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/L2CoralPosition.h"

L2CoralPosition::L2CoralPosition(ElevatorSubsystem* subsystem) : m_elevator(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void L2CoralPosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void L2CoralPosition::Execute() {
  m_elevator->L2CoralPosition();
}

// Called once the command ends or is interrupted.
void L2CoralPosition::End(bool interrupted) {}

// Returns true when the command should end.
bool L2CoralPosition::IsFinished() {
  return false;
}
