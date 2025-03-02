// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutoAlign::AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick): m_drive(drive), m_vision(vision), m_joystick(joystick){ 
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  AddRequirements(vision);
}

// Called when the command is initially scheduled.
void AutoAlign::Initialize() {
  double distance = m_vision->getDistance(8.75);
  double xPose = m_drive->GetEstimatedPose().X().value();
  setPoint = xPose + ((distance-5)/39.37);
}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {
  if(m_joystick->GetRawButton(1)){
  
   double distance = m_vision->getDistance(8.75);
   double poseX = m_drive->GetEstimatedPose().X().value();

   setPoint = poseX + ((distance-5)/39.37);
   double speed = alignPid.Calculate(poseX,setPoint);
   double centerSpeed = centerPid.Calculate(m_vision->getTX(), 0);
  //  double rotationSpeed = rotationPid.Calculate(m_vision->getTX(),0);

   frc::SmartDashboard::PutNumber("setPoint", setPoint);
   frc::SmartDashboard::PutNumber("poseX", poseX);
   frc::SmartDashboard::PutNumber("distance", distance);
   frc::SmartDashboard::PutNumber("speed calc", speed);

   m_drive->Drive(units::velocity::meters_per_second_t{0},units::velocity::meters_per_second_t{centerSpeed},units::angular_velocity::radians_per_second_t{0}, false);



  
  
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
