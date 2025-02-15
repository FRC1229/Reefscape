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


class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();
    rev::spark::SparkMax m_ElevatorMotorBottom;
    rev::spark::SparkMax m_ElevatorMotorTop;
    rev::spark::SparkRelativeEncoder m_ElevatorEncoderBottom;
    rev::spark::SparkRelativeEncoder m_ElevatorEncoderTop;
    frc::ProfiledPIDController<units::meters> m_controller;



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  double readEncoder();
  void SetElevatorSpeed(double speed);
  void SetElevatorPos(frc::TrapezoidProfile<units::meters>::State setPoint);
  units::meter_t getDistance();
  frc::TrapezoidProfile<units::meters>::State goal {0_m,0_mps};

  //CHANGE THESE PLEASE


  frc::PIDController HomePositionTopElevatorPID{
    0.1,0,0
  };
    frc::PIDController ElevatorPid{
    0.1,0,0
  };
  



 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
