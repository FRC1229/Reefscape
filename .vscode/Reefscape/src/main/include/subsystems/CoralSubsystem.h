// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/Servo.h>

class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem();

  frc::Servo m_servoL;
  frc::Servo m_servoR;

  bool shot = false;

  rev::spark::SparkMax m_CoralTilt;
  rev::spark::SparkRelativeEncoder m_CoralEncoder;

  void ShootToggle();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   * 
   */
  frc::PIDController m_coralController {0.2,0,0};
  void Periodic() override;
  double GetAngle();
  void MoveToAngle(double angle);
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
