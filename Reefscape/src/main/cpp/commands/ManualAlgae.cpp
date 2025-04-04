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
    if(m_CoController->GetRawAxis(1) > 0.05){
      m_algae->m_AlgaeTiltMotor.Set(0.1);
      m_algae->lastPose = m_algae->GetAngle();
    }
    else if(m_CoController->GetRawAxis(1) < -0.05){
      m_algae->m_AlgaeTiltMotor.Set(-0.1);
      m_algae->lastPose = m_algae->GetAngle();
    }
    else{
      double holdVolt = m_algae->stayPid.Calculate(m_algae->GetAngle(), m_algae->lastPose);
      m_algae->m_AlgaeTiltMotor.SetVoltage(units::volt_t{holdVolt}); 
    }
     

}

// Called once the command ends or is interrupted.
void ManualAlgae::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualAlgae::IsFinished() {
  return false;
}
