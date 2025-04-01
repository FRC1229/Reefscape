// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/dimensionless.h>
#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <LimelightHelpers.h>
#include <frc/smartdashboard/Field2d.h>

using namespace pathplanner;
using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
  //Defining each Swereve Module
    : m_frontLeft{kFrontLeftDriveMotorID,
                  kFrontLeftTurningMotorID,
                  kFrontLeftTurningEncoderID, true , false, true},

      m_rearLeft{
          kRearLeftDriveMotorID,       kRearLeftTurningMotorID,
          kRearLeftTurningEncoderID, true , true, true},

      m_frontRight{
          kFrontRightDriveMotorID,       kFrontRightTurningMotorID,
          kFrontRightTurningEncoderID, true, true, true},

      m_rearRight{
          kRearRightDriveMotorID,       kRearRightTurningMotorID,
          kRearRightTurningEncoderID, true, false, true},

      m_odometry{kDriveKinematics,
                 m_gyro.GetRotation2d(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                  frc::Pose2d{2_m,7_m,frc::Rotation2d{0_deg}}}

      {
        pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

        AutoBuilder::configure(
        [this](){ return GetEstimatedPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ Drive(speeds.vx,speeds.vy,speeds.omega,false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<PPHolonomicDriveController>( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        [this]()->bool{return this ->Fieldflip();},
        this // Reference to this subsystem to set requirements
    );
    
    rotatePID.EnableContinuousInput(-180,180);

      m_fieldflip.SetDefaultOption("false", false);
      m_fieldflip.AddOption("true", true);
      m_fieldflip.AddOption("false", false);

      frc::SmartDashboard::PutData("IsRedAlliance", &m_fieldflip);

      }

bool DriveSubsystem::Fieldflip(){
  return m_fieldflip.GetSelected();
}

frc::Rotation2d DriveSubsystem::getRotation2D(){
  double yaw = m_gyro.GetYaw().GetValue().value();

  yaw =  fmod(yaw,360.0);

  if (yaw < 0){
    yaw+=360;
  }

  return frc::Rotation2d(units::degree_t(yaw));
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});


  if(m_vision.seeTarget()){
    for(auto tar : m_vision.getCameraRobotPoses()){
      m_odometry.AddVisionMeasurement(tar, frc::Timer::GetFPGATimestamp());
    }
  }



  m_field.SetRobotPose(m_odometry.GetEstimatedPosition());


  frc::SmartDashboard::PutNumber("gyro", m_gyro.GetYaw().GetValue().value());
  frc::SmartDashboard::PutNumber("POSEX", m_odometry.GetEstimatedPosition().X().value());
  frc::SmartDashboard::PutNumber("POSEY", m_odometry.GetEstimatedPosition().Y().value());

  frc::SmartDashboard::PutNumber("Vision X", m_vision.getCameraRobotPose().X().value());
  frc::SmartDashboard::PutNumber("Vision Y", m_vision.getCameraRobotPose().Y().value());




  // frc::Pose2d visionPose = LimelightHelpers::toPose2D(LimelightHelpers::getBotpose());




  //m_odometry.AddVisionMeasurement(visionPose, frc::Timer::GetFPGATimestamp());


  //   inline std::vector<double> getBotpose(const std::string &limelightName = "")
  //   {
  //       return getLimelightNTDoubleArray(limelightName, "botpose");
  //   }
  // inline frc::Pose2d toPose2D(const std::vector<double>& inData)
  //   {
  //       if(inData.size() < 6)
  //       {
  //           return frc::Pose2d();
  //       }
  //       return frc::Pose2d(
  //           frc::Translation2d(units::length::meter_t(inData[0]), units::length::meter_t(inData[1])), 
  //           frc::Rotation2d(units::angle::radian_t(inData[5]*(M_PI/180.0))));
  //   }

}

void DriveSubsystem::DriveOdo(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  
  // ****************************
  
  // double gyroAngle = m_gyro.GetAngle();

  // gyroAngle = (double)((int)(gyroAngle * 100) % 36000) / 100.0;

  // double rotCalc = rotatePID.Calculate(gyroAngle,gyroSetpoint);

  // rot = units::radians_per_second_t{frc::ApplyDeadband(-rotCalc,0.01)};

  //***********************************

  // double gyroAngle = m_gyro.GetAngle();

  // gyroAngle = (double)((int)(gyroAngle * 100) % 36000) / 100.0;

  // //frc::SmartDashboard::PutNumber("GyroAnglePID",-gyroAngle);
  // double rotCalc = rotatePID.Calculate(gyroAngle,gyroSetpoint);

  // rot = units::radians_per_second_t{frc::ApplyDeadband(-rotCalc,0.07)};
  // double goal = 180.0;
  // double clamp_var = 1.1;
  // double inv_p = -70.0;
  // double rotclamped = std::clamp(frc::ApplyDeadband((-gyroAngle - goal),1.0) / inv_p,-clamp_var,clamp_var);
  // rot = units::radians_per_second_t{rotclamped};

  // if (gyroAngle < 180){
  //   rot = units::radians_per_second_t{-0.1};
  // }
  // else{
  //   rot = units::radians_per_second_t{0.1};
  // }

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_odometry.GetEstimatedPosition().Rotation())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr); 
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);

  FrontLeft = fl;
  FrontRight = fr;
  RearLeft =  bl;
  RearRight = br;

  frc::SmartDashboard::PutNumber("Desired Vel", FrontLeft.speed.value());
  // frc::SmartDashboard::PutNumber("Rot PID Out",rotCalc);
  frc::SmartDashboard::PutNumber("rot rps", rot.value());
  frc::SmartDashboard::PutNumber("Yaw", m_gyro.GetYaw().GetValue().value());

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  
  // ****************************
  
  // double gyroAngle = m_gyro.GetAngle();

  // gyroAngle = (double)((int)(gyroAngle * 100) % 36000) / 100.0;

  // double rotCalc = rotatePID.Calculate(gyroAngle,gyroSetpoint);

  // rot = units::radians_per_second_t{frc::ApplyDeadband(-rotCalc,0.01)};

  //***********************************

  // double gyroAngle = m_gyro.GetAngle();

  // gyroAngle = (double)((int)(gyroAngle * 100) % 36000) / 100.0;

  // //frc::SmartDashboard::PutNumber("GyroAnglePID",-gyroAngle);
  // double rotCalc = rotatePID.Calculate(gyroAngle,gyroSetpoint);

  // rot = units::radians_per_second_t{frc::ApplyDeadband(-rotCalc,0.07)};
  // double goal = 180.0;
  // double clamp_var = 1.1;
  // double inv_p = -70.0;
  // double rotclamped = std::clamp(frc::ApplyDeadband((-gyroAngle - goal),1.0) / inv_p,-clamp_var,clamp_var);
  // rot = units::radians_per_second_t{rotclamped};

  // if (gyroAngle < 180){
  //   rot = units::radians_per_second_t{-0.1};
  // }
  // else{
  //   rot = units::radians_per_second_t{0.1};
  // }

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr); 
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);

  FrontLeft = fl;
  FrontRight = fr;
  RearLeft =  bl;
  RearRight = br;

  // frc::SmartDashboard::PutNumber("Desired Vel", FrontLeft.speed.value());
  // // frc::SmartDashboard::PutNumber("Rot PID Out",rotCalc);
  // frc::SmartDashboard::PutNumber("rot rps", rot.value());
  // frc::SmartDashboard::PutNumber("Yaw", m_gyro.GetYaw().GetValue().value());

}

void DriveSubsystem::DriveWithJoysticks(double xJoy, double yJoy, double rotJoy, bool fieldRelative, bool halfSpeed){
  if(halfSpeed){
    xJoy = x_speedLimiter.Calculate(frc::ApplyDeadband(xJoy,0.08)*AutoConstants::kMaxSpeed.value()/4);
    yJoy = y_speedLimiter.Calculate(frc::ApplyDeadband(yJoy,0.08)*AutoConstants::kMaxSpeed.value()/4);
    rotJoy = rot_speedLimiter.Calculate(frc::ApplyDeadband(rotJoy,0.08)*AutoConstants::kMaxAngularSpeed.value()/4);
  }
  else{
    xJoy = x_speedLimiter.Calculate(frc::ApplyDeadband(xJoy,0.08)*AutoConstants::kMaxSpeed.value());
    yJoy = y_speedLimiter.Calculate(frc::ApplyDeadband(yJoy,0.08)*AutoConstants::kMaxSpeed.value());
    rotJoy = rot_speedLimiter.Calculate(frc::ApplyDeadband(rotJoy,0.08)*AutoConstants::kMaxAngularSpeed.value());
  }
  // xJoy = frc::ApplyDeadband(xJoy,0.05)*AutoConstants::kMaxSpeed.value();
  // yJoy = frc::ApplyDeadband(yJoy,0.05)*AutoConstants::kMaxSpeed.value();
  // rotJoy = frc::ApplyDeadband(rotJoy,0.05)*AutoConstants::kMaxAngularSpeed.value();

  const auto xSpeed = units::meters_per_second_t{xJoy};
  const auto ySpeed = units::meters_per_second_t{yJoy};
  const auto rot = units::radians_per_second_t{rotJoy};
  Drive(xSpeed,ySpeed,rot,fieldRelative);
}

void DriveSubsystem::AutoAlign(double tx, double x, double y){
  x = x_speedLimiter.Calculate(frc::ApplyDeadband(x,0.05));
  y = y_speedLimiter.Calculate(frc::ApplyDeadband(y,0.05));
  const auto xSpeed = units::meters_per_second_t{x};
  const auto ySpeed = units::meters_per_second_t{y};
  double rotCalc = rotatePID.Calculate(tx,0);
  Drive(xSpeed,ySpeed,units::radians_per_second_t{frc::ApplyDeadband(rotCalc,0.01)},true);
  // std::cout<<"yipppe"<<std::endl;
}

void DriveSubsystem::TurnToAngle(double x, double y, double rot){
  x = x_speedLimiter.Calculate(frc::ApplyDeadband(x,0.05));
  y = y_speedLimiter.Calculate(frc::ApplyDeadband(y,0.05));
  const auto xSpeed = units::meters_per_second_t{x};
  const auto ySpeed = units::meters_per_second_t{y};
  double rotCalc = rotatePID.Calculate(m_gyro.GetYaw().GetValue().value(),rot);
  Drive(xSpeed,ySpeed,units::radians_per_second_t{frc::ApplyDeadband(rotCalc,0.01)},true);
}

void DriveSubsystem::Multiply(){
  increment *= 10.0;
}

void DriveSubsystem::Divide(){
  increment /= 10.0;
}

void DriveSubsystem::Add(){
  ppp += increment;
}

void DriveSubsystem::Subtract(){
  ppp -= increment;
}



void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() {
  return m_gyro.GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return m_gyro.GetAngularVelocityZWorld().GetValue().value();
}

frc::Pose2d DriveSubsystem::GetEstimatedPose() {
  return m_odometry.GetEstimatedPosition();
}

frc2::CommandPtr DriveSubsystem::pathFind(frc::Pose2d target){

  pathplanner::PathConstraints Constraints = pathplanner::PathConstraints(
    units::meters_per_second_t{2}, units::meters_per_second_squared_t{2},
    units::degrees_per_second_t{150},units::degrees_per_second_squared_t{300}

  );

  frc2::CommandPtr pathfindingCommand = pathplanner::AutoBuilder::pathfindToPose(
    target,
    Constraints,
    0.0_mps
  );

  return pathfindingCommand;
}

frc::ChassisSpeeds DriveSubsystem::getRobotRelativeSpeeds() {
  auto fl = m_frontLeft.GetState();
  auto fr = m_frontRight.GetState();
  auto bl = m_rearLeft.GetState();
  auto br = m_rearRight.GetState();
  return kDriveKinematics.ToChassisSpeeds(fl, fr, bl, br);
}
void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
// void DriveSubsystem::GyroStabilize(){
//   gyroSetpoint = 180;
// }