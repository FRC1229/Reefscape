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


double MYABS(double value){
  if(value < 0){
    return value * -1;
  }
  else{
    return value;
  }
}

void AutoAlign::Execute() {
  if(m_vision->seeTarget()){
  
    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->ClosestTarget().GetFiducialId()];
    m_vision->lastTag = m_vision->ClosestTarget().GetFiducialId();


    if(!(MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error)){

      
      double Xspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().X().value(), targetPose.X().value());
      double Yspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
      double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());


      // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
      // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);

    

      m_drive->Drive(units::velocity::meters_per_second_t{Xspeed}, 
                    units::velocity::meters_per_second_t{Yspeed},
                    units::angular_velocity::radians_per_second_t{rotationSpeed}, true);

    }

    frc::SmartDashboard::PutNumber("hi",targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value());
    frc::SmartDashboard::PutNumber("Hi Y", MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()));
    frc::SmartDashboard::PutBoolean("ISDONE????CHALLENGEIMPOSSIBLE", MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error);
  }
  else{


   

    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];

    if(!(MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error)){

      double Xspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().X().value(), targetPose.X().value());
      double Yspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
      double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());


      // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
      // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);

      double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
      double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

      

    

      m_drive->Drive(units::velocity::meters_per_second_t{Xspeed}, 
                    units::velocity::meters_per_second_t{Yspeed},
                    units::angular_velocity::radians_per_second_t{rotationSpeed}, true);
    }


    
    frc::SmartDashboard::PutNumber("hi",targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value());
    frc::SmartDashboard::PutNumber("Hi Y", targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value());
    frc::SmartDashboard::PutBoolean("ISDONE????CHALLENGEIMPOSSIBLE", MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error);

  }
}


void AutoAlign::End(bool interrupted) {}




bool AutoAlign::IsFinished() {
  frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];
  ///CANCEL LOGIC
  double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
  double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

  

  if(m_vision->seeTarget()){
    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->ClosestTarget().GetFiducialId()];
  }
  else{
    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];
  }
  

  return MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error;





}
