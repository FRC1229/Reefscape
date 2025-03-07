// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/L1Subsystem.h>
#include <frc/Joystick.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetServoPosition
    : public frc2::CommandHelper<frc2::Command, SetServoPosition> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  SetServoPosition(L1Subsystem* m_l1, frc::Joystick* joystick);

  L1Subsystem* m_l1;
  frc::Joystick* m_joystick;

  enum AngleState{
    a0,
    a45,
    a90
  };

  AngleState CurrentState;

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
