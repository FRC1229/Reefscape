// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include <rev/SparkMax.h>
#include <numbers>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

// SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
//                            const int driveEncoderPorts[],
//                            const int turningEncoderPorts[],
//                            bool driveEncoderReversed,
//                            bool turningEncoderReversed)
//     : m_driveMotor(driveMotorChannel),
//       m_turningMotor(turningMotorChannel),
//       m_driveEncoder(driveEncoderPorts[0], driveEncoderPorts[1]),
//       m_turningEncoder(turningEncoderPorts[0], turningEncoderPorts[1]),
//       m_reverseDriveEncoder(driveEncoderReversed),
//       m_reverseTurningEncoder(turningEncoderReversed) {

//Very simlar code to the swerve templete the m_drivemotor and m_turningmotor had to be changed to rev libs
//SPARK MAX AND FLEX can use the same codes
SwerveModule::SwerveModule(int driveMotorChannel, 
                            int turningMotorChannel, 
                            const int turningEncoderPorts, 
                            bool running = false , 
                            bool driveMotorInvert = false, 
                            bool turnMotorInvert = false)
    : m_driveMotor(driveMotorChannel, rev::spark::SparkLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::spark::SparkLowLevel::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder()),
      m_turningEncoder(turningEncoderPorts),
      running(running),
      driveMotorInvert(driveMotorInvert),
      turnMotorInvert(turnMotorInvert)
      {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  // m_driveEncoder.SetDistancePerPulse(
  //     ModuleConstants::kDriveEncoderDistancePerPulse);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  // m_turningEncoder.SetDistancePerPulse(
  //     ModuleConstants::kTurningEncoderDistancePerPulse);
  
  rev::spark::SparkBaseConfig DriveSparkConfig;
  rev::spark::SparkBase::ResetMode SparkReset {rev::spark::SparkBase::ResetMode::kNoResetSafeParameters};
  rev::spark::SparkBase::PersistMode SparkPersist {rev::spark::SparkBase::PersistMode::kNoPersistParameters};
  rev::spark::SparkBaseConfig TurnSparkConfig;

  DriveSparkConfig.Inverted(driveMotorInvert);
  m_driveMotor.Configure(DriveSparkConfig,SparkReset,SparkPersist);

  TurnSparkConfig.Inverted(turnMotorInvert);
  m_turningMotor.Configure(TurnSparkConfig,SparkReset,SparkPersist);



  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -std::numbers::pi, std::numbers::pi);
}

frc::SwerveModuleState SwerveModule::GetState()  {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()*ModuleConstants::velocityScaleFactor},
          units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValue()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition()  {
  return {units::meter_t{m_driveEncoder.GetPosition()*ModuleConstants::positionScaleFactor},
          units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValue()}};
}

//Make own optimize hate library and it hates me
frc::SwerveModuleState Optimize(const frc::Rotation2d& currentAngle, frc::SwerveModuleState state) {
  auto delta = state.angle - currentAngle;
  if (units::math::abs(delta.Degrees()) > 90_deg) {
    state.speed *= -1;
    state.angle = state.angle + frc::Rotation2d{180_deg};
  }
  return state;
}


void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {

  frc::Rotation2d encoderRotation{
      units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValue()}};
  // Optimize the reference state to avoid spinning further than 90 degrees
  
  
  //REMOVE INTERIM ON MONDAY

  // auto interim = referenceState;
  // interim.angle = -interim.angle;
  auto state = Optimize(encoderRotation,referenceState);
  // Calculate the drive output from the drive PID controller.
  // const auto driveOutput = m_drivePIDController.Calculate(
  //       m_driveEncoder.GetRate(), state.speed.value());
  state.speed *= (state.angle - encoderRotation).Cos();

  
  const auto driveOutput = m_drivePIDController.Calculate(m_driveEncoder.GetVelocity()*2*std::numbers::pi*0.0508/(6.75*60),state.speed.value());
  const auto driveFeedForward = m_driveFeedForward.Calculate(state.speed);
  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      m_turningEncoder.GetAbsolutePosition().GetValue().value()*2*std::numbers::pi, state.angle.Radians().value());

  if(running){
    // Set the motor outputs.
    m_turningMotor.SetVoltage(units::volt_t{turnOutput});
    m_driveMotor.SetVoltage(units::volt_t{driveOutput}+driveFeedForward);
  }
}

void SwerveModule::ResetEncoders() {
  m_driveEncoder.SetPosition(0);
  //m_turningEncoder.Reset();
}