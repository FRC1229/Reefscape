// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Joystick.h>
#include <subsystems/DriveSubsystem.h>
#include <subsystems/VisionSubsystem.h>

/**
 * Command for auto-aligning the robot to a vision target using the DriveSubsystem and VisionSubsystem.
 */
class AutoAlign : public frc2::CommandHelper<frc2::Command, AutoAlign> {
 public:
  AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  VisionSubsystem* m_vision;
  frc::Joystick* m_joystick;

  static constexpr double error = 0.02;  // Allowable position error in meters

  frc::PIDController centerPid{1.4, 0.0, 0.0};   // PID for center alignment
  frc::PIDController rotationPid{0.1, 0.0, 0.0}; // PID for rotation alignment
};
