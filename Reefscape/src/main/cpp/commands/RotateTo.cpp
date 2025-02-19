// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateTo.h"


/*Initializes the RotateTo command, which rotates the robot to a specified angle using the DriveSubsystem. The rotation controller is configured to handle continuous input for angles between 0 and 360 degrees.

Parameters
drive - Pointer to the DriveSubsystem, which controls the robot’s movement.
joystick - Pointer to the frc::Joystick, which may be used for manual control or adjustments.
a - The target angle (in degrees) to which the robot should rotate.*/

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
  if((angle + 1) > currentAngle && currentAngle > (angle - 1)){
    Cancel();
    // m_drive->Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), units::angular_velocity::radians_per_second_t(-rotationCalc), true);
  }
  else{
    
    double xJoy = -m_joystick->GetRawAxis(1);
    double yJoy = m_joystick->GetRawAxis(0);

    xJoy = m_drive->x_speedLimiter.Calculate(frc::ApplyDeadband(xJoy,0.08)*AutoConstants::kMaxSpeed.value());
    yJoy = m_drive->y_speedLimiter.Calculate(frc::ApplyDeadband(yJoy,0.08)*AutoConstants::kMaxSpeed.value());

    m_drive->Drive(units::velocity::meters_per_second_t(xJoy), units::velocity::meters_per_second_t(yJoy), units::angular_velocity::radians_per_second_t(rotationCalc), true);
  }
}

// Called once the command ends or is interrupted.
void RotateTo::End(bool interrupted) {
  //Rumble here
}

// Returns true when the command should end.
bool RotateTo::IsFinished() {
  return false;
}
