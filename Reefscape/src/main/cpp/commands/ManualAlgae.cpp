// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualAlgae.h"
#include <cmath> // For std::fabs

ManualAlgae::ManualAlgae(AlgaeSubsystem* algae, frc::Joystick* joystick): m_algae(algae), m_CoController(joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_algae);
}

// Called when the command is initially scheduled.
void ManualAlgae::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualAlgae::Execute() {
  double joystickValue = m_CoController->GetRawAxis(m_TiltAxis);

  // Apply deadband
  if (std::fabs(joystickValue) < m_Deadband) {
    joystickValue = 0.0;
  }

  // Scale joystick input
  double motorSpeed = joystickValue * m_SpeedMultiplier;

  // Set motor speed
  m_algae->SetTiltSpeed(motorSpeed);

}

// Called once the command ends or is interrupted.
void ManualAlgae::End(bool interrupted) {
    // Stop the tilt motor when the command ends
  m_algae->SetTiltSpeed(0.0);
}

// Returns true when the command should end.
bool ManualAlgae::IsFinished() {
  return false;
}
