// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualCoral.h"
// #include "frc/sm"

ManualCoral::ManualCoral(CoralSubsystem* coral, frc::Joystick* joy): m_coral(coral), m_CoController(joy) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_coral);
}

// Called when the command is initially scheduled.
void ManualCoral::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualCoral::Execute() {


    if(m_CoController->GetRawAxis(1) > 0.05){
      m_coral->m_CoralTilt.Set(-m_CoController->GetRawAxis(1)*0.1);
    }
    else if(m_CoController->GetRawAxis(1) < -0.05){
      m_coral->m_CoralTilt.Set(-m_CoController->GetRawAxis(1)*0.1);
    }
    else{
      m_coral->m_CoralTilt.Set(0);
    }

    



}

// Called once the command ends or is interrupted.
void ManualCoral::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualCoral::IsFinished() {
  return false;
}
