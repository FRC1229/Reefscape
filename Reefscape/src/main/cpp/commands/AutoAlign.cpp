// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>

/*This constructor initializes the AutoAlign command, which aligns the robot using vision data and the drivetrain. It sets up dependencies on the DriveSubsystem and VisionSubsystem, ensuring that no other commands interfere while AutoAlign is active.

Parameters
drive - Pointer to the DriveSubsystem, which controls the robot's movement.
vision - Pointer to the VisionSubsystem, responsible for processing vision data.
joystick - Pointer to the frc::Joystick, which can be used for manual override or adjustments.*/

AutoAlign::AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick): m_drive(drive), m_vision(vision), m_joystick(joystick){ 
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  AddRequirements(vision);
}

// Called when the command is initially scheduled.
void AutoAlign::Initialize() {
  double distance = m_vision->getDistance(42.2);
  double xPose = m_drive->GetEstimatedPose().X().value();
  setPoint = xPose + ((distance-5)/39.37);
}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {
  if(m_joystick->GetRawButton(1)){
  
   double distance = m_vision->getDistance(42.2);
   double poseX = m_drive->GetEstimatedPose().X().value();

   setPoint = poseX + ((distance-5)/39.37);
   double speed = alignPid.Calculate(poseX,setPoint);
   double centerSpeed = centerPid.Calculate(m_vision->getTX(), 0);
   double rotationSpeed = rotationPid.Calculate(m_vision->getTX(),0);

   frc::SmartDashboard::PutNumber("setPoint", setPoint);
   frc::SmartDashboard::PutNumber("poseX", poseX);
   frc::SmartDashboard::PutNumber("distance", distance);
   frc::SmartDashboard::PutNumber("speed calc", speed);

   m_drive->Drive(units::velocity::meters_per_second_t{speed},units::velocity::meters_per_second_t{centerSpeed},units::angular_velocity::radians_per_second_t{rotationSpeed}, false);



  
  
  }
  else{
    Cancel();
  }
  
  

}

// Called once the command ends or is interrupted.
void AutoAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAlign::IsFinished() {
  return false;
}
