// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

class RollerSubsystem : public frc2::SubsystemBase {
 public:
  RollerSubsystem();
  rev::spark::SparkMax m_RollerMotor;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void run(double speed);
  void Stop();


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
