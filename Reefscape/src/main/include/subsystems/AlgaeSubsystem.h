// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/Trajectory.h>

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem();
  rev::spark::SparkMax m_AlgaeTiltMotor;
  rev::spark::SparkRelativeEncoder m_AlgaeTiltEncoder;
  
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void run(double speed);

  void ManualTilt();


  double GetAngle();
  void MoveToAngle(double angle);
  frc::PIDController m_AlgaeController {0.275,0.1,0};

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
