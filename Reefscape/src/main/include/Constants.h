// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <pathplanner/lib/config/RobotConfig.h>


#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
constexpr int kFrontLeftDriveMotorID = 6;
constexpr int kRearLeftDriveMotorID = 8;
constexpr int kFrontRightDriveMotorID = 2;
constexpr int kRearRightDriveMotorID = 4;

constexpr int kFrontLeftTurningMotorID = 7;
constexpr int kRearLeftTurningMotorID = 5;
constexpr int kFrontRightTurningMotorID = 1;
constexpr int kRearRightTurningMotorID = 3;

constexpr int kFrontLeftTurningEncoderID = 11;
constexpr int kRearLeftTurningEncoderID = 12;
constexpr int kFrontRightTurningEncoderID = 9;
constexpr int kRearRightTurningEncoderID = 10;

//SET THIS
constexpr int kGyroConstant = 0;

}  // namespace DriveConstants

namespace ModuleConstants {

// We didn't have to use the encoder resolution 
constexpr int kEncoderCPR = 4096;
constexpr double kWheelDiameterMeters = 0.15;

constexpr double velocityScaleFactor = 2*std::numbers::pi*0.0508/(6.75*60);
constexpr double positionScaleFactor = 2*std::numbers::pi*0.0508/(6.75);


}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 1_mps; //4
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 1.2_rad_per_s; //2.142
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

inline constexpr double kPXController = 0.5;
inline constexpr double kPYController = 0.5;
inline constexpr double kPThetaController = 0.5;

// pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();
//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;


}  // namespace AutoConstants

namespace EndEffectorConstants {
    constexpr int kLeftShooterMotorID = 13;
    constexpr int kRightShooterMotorID = 18;
    constexpr int kConveyerMotorID = 19;
    constexpr double kHomePhysicalAngleWrist = 90.0;
    constexpr double ktickToAngleRatioWrist = 90.0/0.238;


}

namespace IntakeConstants {
    constexpr int kIntakeMotorID = 16;
}

//climb is 14

namespace ArmConstants {
    constexpr int kArmMotorID = 17;
    constexpr int kWristMotorID = 15;
    constexpr int kArmEncoderPort = 0;
    constexpr int kWristEncoderPort = 0;
    constexpr double kHomePhysicalAngle = 35.0;
    constexpr double ktickToAngleRatio = 482.64;
}




namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kCoPilotControllerPort = 1;
}  // namespace OIConstants