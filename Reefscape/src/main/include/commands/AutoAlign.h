// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/DriveSubsystem.h>
#include <subsystems/VisionSubsystem.h>
#include <frc/Joystick.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoAlign
    : public frc2::CommandHelper<frc2::Command, AutoAlign> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  DriveSubsystem* m_drive;
  VisionSubsystem* m_vision;
  frc::Joystick* m_joystick;
  double setPoint;
  frc::PIDController alignPid {0.4,0.1,0};
  frc::PIDController centerPid {0.1,0,0};
  frc::PIDController rotationPid {0.075,0.0,0.0};

};
