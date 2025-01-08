

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <rev/SparkMax.h>
#include "Constants.h"

class SwerveModule {
 public:
  rev::spark::SparkMax m_driveMotor;
  rev::spark::SparkMax m_turningMotor;

  rev::spark::SparkRelativeEncoder m_driveEncoder;
  ctre::phoenix6::hardware::CANcoder m_turningEncoder;

//   SwerveModule(int driveMotorChannel, int turningMotorChannel,
//                const int driveEncoderPorts[2], const int turningEncoderPorts[2],
//                bool driveEncoderReversed, bool turningEncoderReversed);
  SwerveModule(int driveMotorChannel, 
                int turningMotorChannel, 
                const int turningEncoderPorts, 
                bool running, 
                bool driveMotorInvert, 
                bool turnMotorInvert);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();
  frc::Rotation2d ninety_degree_rotation2d = frc::Rotation2d{0.0,1.0};


  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();
  bool running;
  bool driveMotorInvert;
  bool turnMotorInvert;
  bool driveEncoderInvert;

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
  static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedForward{0.2_V,2.6_V/1_mps};

  frc::PIDController m_drivePIDController{
      0.1, 0.0, 0.00};
  frc::PIDController m_turningPIDController{
      1.9,
      0.0,
      0.0};
  
};