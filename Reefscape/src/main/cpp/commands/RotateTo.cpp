// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateTo.h"

RotateTo::RotateTo(DriveSubsystem* drive, frc::Joystick* joystick, double a): m_drive(drive), m_joystick(joystick), angle(a) {
  AddRequirements(m_drive);
  rotationController.EnableContinuousInput(0,360);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RotateTo::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RotateTo::Execute() {
  double currentAngle =  m_drive->getRotation2D().Degrees().value();
  double rotationCalc = rotationController.Calculate(m_drive->getRotation2D().Degrees().value(), angle);
  
  double xJoy = -m_joystick->GetRawAxis(1);
  double yJoy = m_joystick->GetRawAxis(0);

  xJoy = m_drive->x_speedLimiter.Calculate(frc::ApplyDeadband(xJoy,0.08)*AutoConstants::kMaxSpeed.value());
  yJoy = m_drive->y_speedLimiter.Calculate(frc::ApplyDeadband(yJoy,0.08)*AutoConstants::kMaxSpeed.value());

  m_drive->Drive(units::velocity::meters_per_second_t(xJoy), units::velocity::meters_per_second_t(yJoy), units::angular_velocity::radians_per_second_t(rotationCalc), true);
  
}

// Called once the command ends or is interrupted.
void RotateTo::End(bool interrupted) {
  //Rumble here
}

// Returns true when the command should end.
bool RotateTo::IsFinished() {
  double currentAngle =  m_drive->getRotation2D().Degrees().value();
  return (angle + 0.5) > currentAngle && currentAngle > (angle - 0.5);
}
