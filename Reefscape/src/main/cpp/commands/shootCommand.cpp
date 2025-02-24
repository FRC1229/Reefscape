// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shootCommand.h"

shootCommand::shootCommand(AlgaeSubsystem* algae, double speed): m_algae(algae), m_speed(speed){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_algae);
}

// Called when the command is initially scheduled.
void shootCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void shootCommand::Execute() {

  m_algae->run(m_speed);

}

// Called once the command ends or is interrupted.
void shootCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool shootCommand::IsFinished() {
  return false;
}
