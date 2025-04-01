// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/DriveSubsystem.h>
#include <subsystems/VisionSubsystem.h>
#include <subsystems/DriveSubsystem.h>
#include <subsystems/SwerveModule.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/Joystick.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <studica/AHRS.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoLastAlign
    : public frc2::CommandHelper<frc2::Command, AutoLastAlign> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  AutoLastAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick);

  DriveSubsystem* m_drive;
  VisionSubsystem* m_vision;
  frc::Joystick* m_joystick;


  double setPoint;
  frc::PIDController alignPid {0.4,0.1,0};

  frc::PIDController centerPid {0.75,0,0};

  frc::PIDController rotationPid {0.2,0.0,0.00};
  


  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
