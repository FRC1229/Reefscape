#include "commands/AutoAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <Constants.h>

AutoAlign::AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick, frc::Timer* rumbletimer)
    :m_drive(drive), m_vision(vision), m_joystick(joystick),m_rumbletimer(rumbletimer){
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
      double Yspeed = alignPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
      double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());


      // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
      // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
      // double xClamp = 0.8;
      // double yClamp = 0.8;
      // double rotClamp = 0.5;
      // if(Xspeed > xClamp){
      //   Xspeed=xClamp;
      // }
      // else if(Xspeed < -xClamp){
      //   Xspeed=-xClamp;
      // }

      // if(Yspeed > yClamp){
      //   Yspeed=yClamp;
      // }
      // else if(Yspeed < -yClamp){
      //   Yspeed=-yClamp;
      // }

      // if(rotationSpeed > rotClamp){
      //   rotationSpeed=rotClamp;
      // }
      // else if(rotationSpeed < -rotClamp){
      //   rotationSpeed=-rotClamp;
      // }

    frc::SmartDashboard::PutNumber("Xspeed", Xspeed);
    frc::SmartDashboard::PutNumber("Yspeed", Yspeed);
    frc::SmartDashboard::PutNumber("Rotspeed", rotationSpeed);

      m_drive->DriveOdo(units::velocity::meters_per_second_t{Xspeed}, 
                    units::velocity::meters_per_second_t{Yspeed},
                    units::angular_velocity::radians_per_second_t{rotationSpeed}, true);

    }

    
  }
  else{


   

    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];

    if(!(MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error)){

      double Xspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().X().value(), targetPose.X().value());
      double Yspeed = alignPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
      double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());


      // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
      // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
      // double xClamp = 0.8;
      // double yClamp = 0.8;
      // double rotClamp = 0.5;
      // if(Xspeed > xClamp){
      //   Xspeed=xClamp;
      // }
      // else if(Xspeed < -xClamp){
      //   Xspeed=-xClamp;
      // }

      // if(Yspeed > yClamp){
      //   Yspeed=yClamp;
      // }
      // else if(Yspeed < -yClamp){
      //   Yspeed=-yClamp;
      // }

      // if(rotationSpeed > rotClamp){
      //   rotationSpeed=rotClamp;
      // }
      // else if(rotationSpeed < -rotClamp){
      //   rotationSpeed=-rotClamp;
      // }

      double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
      double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

      

    frc::SmartDashboard::PutNumber("Xspeed", Xspeed);
    frc::SmartDashboard::PutNumber("Yspeed", Yspeed);
    frc::SmartDashboard::PutNumber("Rotspeed", rotationSpeed);

    m_drive->DriveOdo(units::velocity::meters_per_second_t{Xspeed}, 
                    units::velocity::meters_per_second_t{Yspeed},
                    units::angular_velocity::radians_per_second_t{rotationSpeed}, true);
    }


    
    
  }
}


void AutoAlign::End(bool interrupted) {
  m_joystick->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
}




bool AutoAlign::IsFinished() {
  frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];
  ///CANCEL LOGIC
  double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
  double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

  

  if(m_vision->seeTarget()){
    frc::Pose2d targetPose = m_vision->targetPoses[m_vision->ClosestTarget().GetFiducialId()];
    double xError = MYABS(targetPose.X().value() - m_drive->m_odometry.GetEstimatedPosition().X().value());
    double yError = MYABS(targetPose.Y().value() - m_drive->m_odometry.GetEstimatedPosition().Y().value());

    targetPose = m_vision->targetPoses[m_vision->getID()];
    if ((xError < error && yError < error)) {
      m_rumbletimer->Start();

      if(m_rumbletimer->Get().value() < 1){
        m_joystick->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
      } else {
        m_joystick->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
        m_rumbletimer->Stop();
        m_rumbletimer->Reset();
        
      }
    } 
    
  
  return xError < error && yError < error;





}
