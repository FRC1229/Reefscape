// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetCoralPosition.h"
  #include <frc/smartdashboard/SmartDashboard.h>

SetCoralPosition::SetCoralPosition(CoralSubsystem* coral, double angle): m_coral(coral), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_coral);
}

// Called when the command is initially scheduled.
void SetCoralPosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetCoralPosition::Execute() {

  


  m_coral->MoveToAngle(m_angle);
  // m_coral->MoveToAngle(m_angle);

}

// Called once the command ends or is interrupted.
void SetCoralPosition::End(bool interrupted) {
  m_coral->m_CoralTilt.Set(0);
  frc::SmartDashboard::PutNumber("I'M CORAL AND I HAVE CANCELED",20);
}

// Returns true when the command should end.
bool SetCoralPosition::IsFinished() {
  double error = 2;
  return (m_angle + error) > m_coral->GetAngle() && m_coral->GetAngle() > (m_angle - error);
}
