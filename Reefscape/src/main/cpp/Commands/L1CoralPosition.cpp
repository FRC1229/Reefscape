// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/L1CoralPosition.h"

L1CoralPosition::L1CoralPosition(ElevatorSubsystem* subsystem) : m_elevator(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void L1CoralPosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void L1CoralPosition::Execute() {
  m_elevator->L1CoralPosition();
}

// Called once the command ends or is interrupted.
void L1CoralPosition::End(bool interrupted) {}

// Returns true when the command should end.
bool L1CoralPosition::IsFinished() {
  return false;
}
