// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetCoralPosition.h"

/*Initializes the SetCoralPosition command, which sets the CoralSubsystem to a specified angle.

Parameters
coral - Pointer to the CoralSubsystem, which manages the coral mechanism.
angle - The target angle (in degrees) to which the coral mechanism should be set.*/

SetCoralPosition::SetCoralPosition(CoralSubsystem* coral, double angle): m_coral(coral), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetCoralPosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetCoralPosition::Execute() {

  if((m_angle + 1) > m_coral->GetAngle() && m_coral->GetAngle() > (m_angle - 1)){
    Cancel();
  }
  else{
    m_coral->MoveToAngle(m_angle);
  }
  // m_coral->MoveToAngle(m_angle);

}

// Called once the command ends or is interrupted.
void SetCoralPosition::End(bool interrupted) {}

// Returns true when the command should end.
bool SetCoralPosition::IsFinished() {
  return false;
}
