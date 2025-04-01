// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoL1Command.h"

AutoL1Command::AutoL1Command(L1Subsystem* l1, double angle ) :  m_l1(l1), m_angle(angle)  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_l1);
}

// Called when the command is initially scheduled.
void AutoL1Command::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoL1Command::Execute() {
  m_l1->SetAngle(m_angle);
}

// Called once the command ends or is interrupted.
void AutoL1Command::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoL1Command::IsFinished() {
  return false;
}
