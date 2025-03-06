#include "commands/AutoAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutoAlign::AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick)
    : m_drive(drive), m_vision(vision), m_joystick(joystick) {
  AddRequirements(drive);
  AddRequirements(vision);
}

void AutoAlign::Initialize() {
  double distance = m_vision->getDistance(8.75);
  double xPose = m_drive->GetEstimatedPose().X().value();
  double setPoint = xPose + ((distance - 5) / 39.37);
}

void AutoAlign::Execute() {
  if (m_joystick->GetRawButton(1)) {
    double distance = m_vision->getDistance(8.75);
    double poseX = m_drive->GetEstimatedPose().X().value();
    double setPoint = poseX + ((distance - 5) / 39.37);
    
    double speed = alignPid.Calculate(poseX, setPoint);

    double txLeft = m_vision->getTXLeft();
    double txRight = m_vision->getTXRight();

    double centerSpeedLeft = centerPid.Calculate(m_vision->getTXLeft(), 0);
    double centerSpeedRight = centerPid.Calculate(m_vision->getTXRight(), 0);
    double rotationSpeed = rotationPid.Calculate((txLeft + txRight) / 2, 0);

    frc::SmartDashboard::PutNumber("setPoint", setPoint);
    frc::SmartDashboard::PutNumber("poseX", poseX);
    frc::SmartDashboard::PutNumber("distance", distance);
    frc::SmartDashboard::PutNumber("speed calc", speed);

    m_drive->Drive(units::velocity::meters_per_second_t{speed}, 
                   units::velocity::meters_per_second_t{(centerSpeedLeft + centerSpeedRight) / 2},
                   units::angular_velocity::radians_per_second_t{rotationSpeed}, false);
  } else {
    Cancel();
  }
}

void AutoAlign::End(bool interrupted) {}

bool AutoAlign::IsFinished() {
  return false;
}
