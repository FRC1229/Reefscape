// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>


class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();
    rev::spark::SparkMax m_ElevatorMotorBottom;
    rev::spark::SparkMax m_ElevatorMotorTop;
    rev::spark::SparkAbsoluteEncoder m_ElevatorEncoderBottom;
    rev::spark::SparkAbsoluteEncoder m_ElevatorEncoderTop;


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  double readEncoder(double encodervalue);
  void SetElevatorSpeed(double speed);
  void HomePosition();
  void L1CoralPosition();
  void L2CoralPosition();
  void L3CoralPosition();
  void L4CoralPosition();

  //CHANGE THESE PLEASE
    frc::PIDController HomePositionTopElevatorPID{
    0.1,0,0
  };
    frc::PIDController HomePositionBottomElevatorPID{
    0.1,0,0
  };
    frc::PIDController L1CoralTopElevatorPID{
    0.1,0,0
  };
    frc::PIDController L1CoralBottomElevatorPID{
    0.1,0,0
  };
    frc::PIDController L2CoralTopElevatorPID{
    0.1,0,0
  };
    frc::PIDController L2CoralBottomElevatorPID{
    0.1,0,0
  };
    frc::PIDController L3CoralTopElevatorPID{
    0.1,0,0
  };
    frc::PIDController L3CoralBottomElevatorPID{
    0.1,0,0
  };
    frc::PIDController L4CoralTopElevatorPID{
    0.1,0,0
  };
    frc::PIDController L4CoralBottomElevatorPID{
    0.1,0,0
  };

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
