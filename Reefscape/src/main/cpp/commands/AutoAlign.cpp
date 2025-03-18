#include "commands/AutoAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>  // For std::abs
#include <Constants.h>

AutoAlign::AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick)
    : m_drive(drive), m_vision(vision), m_joystick(joystick) {
  AddRequirements(drive);
  AddRequirements(vision);
  rotationPid.EnableContinuousInput(0, 360);
}

void AutoAlign::Initialize() {
  // Initialization logic if needed
}

void AutoAlign::Execute() {
  frc::Pose2d currentPose = m_drive->m_odometry.GetEstimatedPosition();
  frc::Pose2d targetPose;

  if (m_vision->seeTarget()) {
    int targetId = m_vision->ClosestTarget().GetFiducialId();
    m_vision->lastTag = targetId;
    targetPose = m_vision->targetPoses[targetId];
  } else {
    targetPose = m_vision->targetPoses[m_vision->lastTag];
  }

  double xError = targetPose.X().value() - currentPose.X().value();
  double yError = targetPose.Y().value() - currentPose.Y().value();
  bool isAligned = std::abs(xError) < error && std::abs(yError) < error;

  if (!isAligned) {
    double Xspeed = centerPid.Calculate(currentPose.X().value(), targetPose.X().value());
    double Yspeed = centerPid.Calculate(currentPose.Y().value(), targetPose.Y().value());
    double rotationSpeed = rotationPid.Calculate(currentPose.Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());

    m_drive->Drive(
        units::velocity::meters_per_second_t{Xspeed}, 
        units::velocity::meters_per_second_t{Yspeed},
        units::angular_velocity::radians_per_second_t{rotationSpeed}, true);
  }

  // Debugging output
  frc::SmartDashboard::PutNumber("X Error", xError);
  frc::SmartDashboard::PutNumber("Y Error", yError);
  frc::SmartDashboard::PutBoolean("Alignment Complete", isAligned);
}

void AutoAlign::End(bool interrupted) {
  // Stop movement when the command ends
  m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false);
}

bool AutoAlign::IsFinished() {
  frc::Pose2d currentPose = m_drive->m_odometry.GetEstimatedPosition();
  frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];

  double xError = std::abs(targetPose.X().value() - currentPose.X().value());
  double yError = std::abs(targetPose.Y().value() - currentPose.Y().value());

  return xError < error && yError < error;
}