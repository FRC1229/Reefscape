// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <cameraserver/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>


void Robot::RobotInit() {
    //m_container.m_arm.CalibrateEncoderValue();
    frc::CameraServer::StartAutomaticCapture();

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  


  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutNumber("Gyroo", m_container.m_drive.m_gyro.GetRotation2d().Degrees().value());

  frc::SmartDashboard::PutNumber("distanceYesCool", m_container.m_vision.getDistance(44.5));
  frc::SmartDashboard::PutNumber("FLTurn Encoder", m_container.m_drive.m_frontLeft.m_turningEncoder.GetAbsolutePosition().GetValue().value());
  frc::SmartDashboard::PutNumber("FRTurn Encoder", m_container.m_drive.m_frontRight.m_turningEncoder.GetAbsolutePosition().GetValue().value());
  frc::SmartDashboard::PutNumber("BLTurn Encoder", m_container.m_drive.m_rearLeft.m_turningEncoder.GetAbsolutePosition().GetValue().value());
  frc::SmartDashboard::PutNumber("BRTurn Encoder", m_container.m_drive.m_rearRight.m_turningEncoder.GetAbsolutePosition().GetValue().value());
  frc::SmartDashboard::PutNumber("FLDrive Encoder", m_container.m_drive.m_frontLeft.GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("FRDrive Encoder", m_container.m_drive.m_frontRight.GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("BLDrive Encoder", m_container.m_drive.m_rearLeft.GetPosition().distance.value());
  frc::SmartDashboard::PutNumber("BRDrive Encoder", m_container.m_drive.m_rearRight.GetPosition().distance.value());
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  //m_container.m_arm.CalibrateEncoderValue();
  m_container.m_drive.ZeroHeading();
  m_container.m_drive.ResetEncoders();
  m_container.m_drive.ResetOdometry(frc::Pose2d{2_m,7_m,frc::Rotation2d{0_deg}});
  
  m_autonomousCommand = m_container.getAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_container.m_drive.ZeroHeading();
  m_container.m_drive.ResetEncoders();
  m_container.m_drive.ResetOdometry(frc::Pose2d{0_m,0_m,frc::Rotation2d{0_deg}});

  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  //m_container.m_arm.CalibrateEncoderValue();

  
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
  m_container.m_drive.ZeroHeading();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {

  // frc::SmartDashboard::PutNumber("FL Desired Angle", m_container.m_drive.FrontLeft.angle.Radians().value());
  // frc::SmartDashboard::PutNumber("FL Desired Speed", m_container.m_drive.FrontLeft.speed.value());
  // frc::SmartDashboard::PutNumber("BL calc", m_container.m_drive.RearLeft.angle.Radians().value());
  // frc::SmartDashboard::PutNumber("BR calc", m_container.m_drive.RearRight.angle.Radians().value());
  // frc::SmartDashboard::PutNumber("FR calc", m_container.m_drive.FrontRight.angle.Radians().value());
 
  // frc::SmartDashboard::PutNumber("Desired Speed", m_container.m_drive.FrontLeft.speed.value());
  // frc::SmartDashboard::PutNumber("FLVel Encoder", m_container.m_drive.m_frontLeft.GetState().speed.value());
  // frc::SmartDashboard::PutNumber("FRVel Encoder", m_container.m_drive.m_frontRight.GetState().speed.value());
  // frc::SmartDashboard::PutNumber("BLVel Encoder", m_container.m_drive.m_rearLeft.GetState().speed.value());
  // frc::SmartDashboard::PutNumber("BRVel Encoder", m_container.m_drive.m_rearRight.GetState().speed.value());
  // frc::SmartDashboard::PutNumber("ppp", m_container.m_drive.ppp);
  // frc::SmartDashboard::PutNumber("increment", m_container.m_drive.increment);
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif