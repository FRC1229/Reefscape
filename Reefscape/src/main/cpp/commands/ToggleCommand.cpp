// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ToggleCommand.h"

ToggleCommand::ToggleCommand(CoralSubsystem* coral):m_coral(coral) {
  AddRequirements(m_coral);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ToggleCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ToggleCommand::Execute() {
  
  if(m_coral->shot){
     m_coral->m_servoL.Set(0.2);
     m_coral->m_servoR.Set(0.2);
     m_coral->shot = false;
  }
  else{
    m_coral->m_servoL.Set(0);
    m_coral->m_servoR.Set(0);
  }

}

// Called once the command ends or is interrupted.
void ToggleCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ToggleCommand::IsFinished() {
  if(m_coral->shot){
    return m_coral->m_servoL.GetAngle() == 0.05 && m_coral->m_servoR.GetAngle() == 0.05;
  }
  else{
    return m_coral->m_servoL.GetAngle() == 0 && m_coral->m_servoR.GetAngle() == 0;
  }
  return false;
}
