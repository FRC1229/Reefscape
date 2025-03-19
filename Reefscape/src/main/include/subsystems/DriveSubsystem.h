// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Encoder.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <units/dimensionless.h>
#include <frc/filter/SlewRateLimiter.h>
#include <vector>
#include "Constants.h"
#include "SwerveModule.h"
#include <studica/AHRS.h>
#include <subsystems/VisionSubsystem.h>
#include <frc/smartdashboard/Field2d.h>


class DriveSubsystem : public frc2::SubsystemBase {
 public:

  frc::SwerveModuleState FrontLeft;
  frc::SwerveModuleState FrontRight;
  frc::SwerveModuleState RearLeft;
  frc::SwerveModuleState RearRight;

  SwerveModule m_frontLeft;
  SwerveModule m_rearLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearRight;

  VisionSubsystem m_vision;

  frc::Field2d m_field;

  frc::SlewRateLimiter<units::scalar> x_speedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> y_speedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> rot_speedLimiter{10 / 1_s};
  bool Fieldflip();

  DriveSubsystem();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

                
  void DriveOdo(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  void DriveWithJoysticks(double xJoy, double yJoy, double rotJoy, bool fieldRelative, bool halfSpeed);
  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  void TurnToAngle(double x, double y, double rot);

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();
  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetEstimatedPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);
  double increment = 0.00001;
  double ppp = 0.05;
  void GyroStabilize();
  void Multiply();
  void Divide();
  void Subtract();
  void Add();
  void AutoAlign(double tx, double x, double y);
  double gyroSetpoint = 15;

  frc::ChassisSpeeds getRobotRelativeSpeeds();

  frc::PIDController rotatePID{
    0.055,0,0.0
  };

  units::meter_t kTrackWidth =
      0.5207_m;  // Distance between centers of right and left wheels on robot
  units::meter_t kWheelBase =
      0.5207_m;  // Distance between centers of front and back wheels on robo

   frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
      frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
      frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
      frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}};

  frc::Rotation2d zero = frc::Rotation2d(0_deg);

  // AHRS m_gyro {frc::SPI::kMXP};
 ctre::phoenix6::hardware::Pigeon2 m_gyro {14};



  frc::Rotation2d getRotation2D();
  frc::SwerveDrivePoseEstimator<4> m_odometry;
  frc::SendableChooser<bool> m_fieldflip;


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.


  // The gyro sensor
   

  // Odometry class for tracking robot pose
  // 4 defines the number of modules

};