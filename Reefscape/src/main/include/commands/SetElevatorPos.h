// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ElevatorSubsystem.h"

class SetElevatorPos
    : public frc2::CommandHelper<frc2::Command, SetElevatorPos> {
 public:
  /**
   * Constructor for SetElevatorPos command.
   *
   * @param elevator The elevator subsystem used by this command.
   * @param targetPosition The desired elevator position in meters.
   */
  SetElevatorPos(ElevatorSubsystem* elevator, double targetPosition);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  ElevatorSubsystem* m_elevator;
  double m_targetPosition;
  static constexpr double kPositionTolerance = 0.001; // Tolerance in meters
};
