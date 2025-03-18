// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoLastAlign.h"

AutoLastAlign::AutoLastAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick)
    :m_drive(drive), m_vision(vision), m_joystick(joystick) {
    
    AddRequirements(m_drive);
    AddRequirements(m_vision);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoLastAlign::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoLastAlign::Execute() {


    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];

    double Xspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().X().value(), targetPose.X().value());
    double Yspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
    double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());


    // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
    // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);

    double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
    double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

    

  

    m_drive->Drive(units::velocity::meters_per_second_t{Xspeed}, 
                   units::velocity::meters_per_second_t{Yspeed},
                   units::angular_velocity::radians_per_second_t{rotationSpeed}, false);


}

// Called once the command ends or is interrupted.
void AutoLastAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoLastAlign::IsFinished() {
  return false;
}
