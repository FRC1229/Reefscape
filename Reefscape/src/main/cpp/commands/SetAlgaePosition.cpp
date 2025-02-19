// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetAlgaePosition.h"
#include "frc/smartdashboard/SmartDashboard.h"


/*Initializes the SetAlgaePosition command, which sets the AlgaeSubsystem to a specified angle.

Parameters
algae - Pointer to the AlgaeSubsystem, which manages the algae mechanism.
angle - The target angle (in degrees) to which the algae mechanism should be set.*/

SetAlgaePosition::SetAlgaePosition(AlgaeSubsystem* algae, double angle): m_algae(algae), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetAlgaePosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetAlgaePosition::Execute() {
  if(m_algae->GetAngle() >= (m_angle-1) && m_algae->GetAngle() <= (m_angle+1)){
    Cancel();
  }
  else{
    double volt = m_algae->m_AlgaeController.Calculate(m_algae->GetAngle(), m_angle);
    m_algae->m_AlgaeTiltMotor.SetVoltage(units::volt_t{volt});
  }

}

// Called once the command ends or is interrupted.
void SetAlgaePosition::End(bool interrupted) {
  frc::SmartDashboard::PutNumber("Hey IM CANCELED",12);

}

// Returns true when the command should end.
bool SetAlgaePosition::IsFinished() {
  return false;
}
