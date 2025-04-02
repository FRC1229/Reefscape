// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetAlgaePosition.h"
#include "frc/smartdashboard/SmartDashboard.h"

SetAlgaePosition::SetAlgaePosition(AlgaeSubsystem* algae, double angle): m_algae(algae), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_algae);
}

// Called when the command is initially scheduled.
void SetAlgaePosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetAlgaePosition::Execute() {

  double volt = m_algae->m_AlgaeController.Calculate(m_algae->GetAngle(), m_angle);
  m_algae->m_AlgaeTiltMotor.SetVoltage(frc::ApplyDeadband(units::volt_t{volt},0_V,0.75_V));
  
}

// Called once the command ends or is interrupted.
void SetAlgaePosition::End(bool interrupted) {
  frc::SmartDashboard::PutNumber("Hey IM CANCELED",12);

}

// Returns true when the command should end.
bool SetAlgaePosition::IsFinished() {
  double error = 0.2;
  // return true;
  return (m_algae->GetAngle() >= (m_angle-error) && m_algae->GetAngle() <= (m_angle+error));
}
