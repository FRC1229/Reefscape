// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualAlgae.h"

ManualAlgae::ManualAlgae(AlgaeSubsystem* algae, frc::Joystick* joystick): m_algae(algae), m_CoController(joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_algae);
}

// Called when the command is initially scheduled.
void ManualAlgae::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualAlgae::Execute() {

    m_algae->m_AlgaeTiltMotor.Set(0);
    if(m_CoController->GetPOV() == 0){
      m_algae->m_AlgaeTiltMotor.Set(0.1);
    }
    else if(m_CoController->GetPOV() == 180){
      m_algae->m_AlgaeTiltMotor.Set(-0.1);
    }
    else{
      m_algae->m_AlgaeTiltMotor.Set(0);
    }


}

// Called once the command ends or is interrupted.
void ManualAlgae::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualAlgae::IsFinished() {
  return false;
}
