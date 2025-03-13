#include "commands/AutoAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>

AutoAlign::AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick)
    :m_drive(drive), m_vision(vision), m_joystick(joystick){
  AddRequirements(drive);
  AddRequirements(vision);
  rotationPid.EnableContinuousInput(0,360);
}

void AutoAlign::Initialize() {
  // double distance = m_vision->getDistance(12);
  // double xPose = m_drive->GetEstimatedPose().X().value();
  // double setPoint = xPose + ((distance - 5) / 39.37);
}

void AutoAlign::Execute() {
  if(m_vision->seeTarget()){
  
    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->getID()];
    lastTag= m_vision->getID();


    double Xspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().X().value(), targetPose.X().value());
    double Yspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
    double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());


    // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
    // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);

  

    m_drive->Drive(units::velocity::meters_per_second_t{Xspeed}, 
                   units::velocity::meters_per_second_t{Yspeed},
                   units::angular_velocity::radians_per_second_t{rotationSpeed}, false);

  

    
  }
  else{
    frc::Pose2d targetPose = m_vision->targetPoses[lastTag];

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
}


void AutoAlign::End(bool interrupted) {}

bool AutoAlign::IsFinished() {
  double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
  double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

  if(m_vision->seeTarget()){
    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->getID()];
  }
  else{
    frc::Pose2d targetPose = m_vision->targetPoses[lastTag];
  }
  


  return false;
}
