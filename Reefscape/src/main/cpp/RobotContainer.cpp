// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include <frc2/command/InstantCommand.h>
#include <iostream>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/POVButton.h>
#include "commands/AutoAlign.h"
#include "commands/RotateTo.h"
#include "commands/SetElevatorPos.h"
#include "commands/ManualElevator.h"
#include "commands/shootCommand.h"
#include "commands/ManualAlgae.h"
#include "commands/ManualCoral.h"
#include "commands/SetAlgaePosition.h"
#include "commands/SetCoralPosition.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include "commands/SetServoPosition.h"
#include <commands/SetServoPosition.h>
#include <commands/AutoL1Command.h>



using namespace DriveConstants;
using namespace pathplanner;


RobotContainer::RobotContainer(){





  // Initialize all of your commands and subsystems here
  // Configure the button bindings
  
  // frc2::CommandPtr threeAndHalf = pathplanner::PathPlannerAuto("New Auto(4)").ToPtr();
  // frc2::CommandPtr sideNote = pathplanner::PathPlannerAuto("OneNoteSide").ToPtr();

  // m_chooser.AddOption("3.5 Note", threeAndHalf.get());
  // m_chooser.AddOption("Side Auto", sideNote.get());
  // m_chooser.SetDefaultOption("One Meter", "One meter");   
  // m_chooser.AddOption("One Meter", "One meter");


  // bool isCompetition = false;

  
  // autoChooser = AutoBuilder::buildAutoChooserFilter(
  //   [&isCompetition](const PathPlannerAuto& autoCommand)
  //   {
  //     return isCompetition ? autoCommand.GetName().starts_with("comp") : true;
  //   }
  // );
  NamedCommands::registerCommand("coralTravel",std::move(SetCoralPosition(&m_coral,5).ToPtr())); 
  NamedCommands::registerCommand("coralShootL4",std::move(SetCoralPosition(&m_coral,33).ToPtr()));
  NamedCommands::registerCommand("ElevatorPosL4",std::move(SetElevatorPos(&m_elevator,0.905).ToPtr()));
  NamedCommands::registerCommand("ElevatorPosHome",std::move(SetElevatorPos(&m_elevator,0.005).ToPtr()));
  NamedCommands::registerCommand("L1Shoot",std::move(AutoL1Command(&m_l1,0.5).ToPtr()));
  NamedCommands::registerCommand("L1Intake",std::move(AutoL1Command(&m_l1,0.25).ToPtr()));
  NamedCommands::registerCommand("L1Travel",std::move(AutoL1Command(&m_l1,0.25).ToPtr()));


  


 
  ConfigureButtonBindings();

    
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // m_drive.Drive(
        //     units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetRawAxis(1),0.05)},
        //     units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetRawAxis(0),0.05)},
        //     units::radians_per_second_t{frc::ApplyDeadband(-m_driverController.GetRawAxis(4),0.05)}, true);
        m_drive.DriveWithJoysticks(
         -m_driverController.GetRawAxis(1),-m_driverController.GetRawAxis(0),-m_driverController.GetRawAxis(4),true, m_driverController.GetRawButton(5));
              
      },
      {&m_drive}));
  m_elevator.SetDefaultCommand(ManualElevator(&m_elevator,&m_copilotController).ToPtr());
  m_algae.SetDefaultCommand(ManualAlgae(&m_algae,&m_driverController).ToPtr());
  m_coral.SetDefaultCommand(ManualCoral(&m_coral,&m_copilotController).ToPtr());
  m_l1.SetDefaultCommand(SetServoPosition(&m_l1,&m_driverController));

  


}

// bool RobotContainer::GetTriggerPressed(){
//   if(m_copilotController.GetRawAxis(3) > 0.05){
//     return true;
//   }
//   return false;
// }


void RobotContainer::ConfigureButtonBindings() {

  frc2::Trigger rightTriggerPressed([this] { return m_copilotController.GetRawAxis(3) > 0.05;}); 
  frc2::Trigger leftTriggerPressed([this] { return m_copilotController.GetRawAxis(2) > 0.05;});

  // frc2::JoystickButton(&m_driverController, 2).OnTrue(TurnToAngle(&m_drive, &m_driverController, 90.0).ToPtr());
  // frc2::JoystickButton(&m_driverController, 3).OnTrue(TurnToAngle(&m_drive, &m_driverController, -90.0).ToPtr());
  frc2::JoystickButton(&m_driverController, 4).OnTrue(frc2::cmd::RunOnce([this]{m_drive.ZeroHeading();}));
  frc2::JoystickButton(&m_driverController, 6).OnTrue(frc2::cmd::RunOnce([this]{m_drive.GetCurrentCommand()->Cancel();}));
  // frc2::JoystickButton(&m_driverController,2).OnTrue(RotateTo(&m_drive,&m_driverController,45).ToPtr());
  frc2::JoystickButton(&m_driverController,1).OnTrue(AutoAlign(&m_drive,&m_vision,&m_driverController).ToPtr());

  //Co

  //elevamator
  frc2::JoystickButton(&m_copilotController, 2).WhileTrue(SetElevatorPos(&m_elevator,0.367).ToPtr()); //Home
  frc2::JoystickButton(&m_copilotController, 1).WhileTrue(SetElevatorPos(&m_elevator,0.005).ToPtr()); // L3
  // frc2::JoystickButton(&m_copilotController, 3).WhileTrue(SetElevatorPos(&m_elevator,0.5).ToPtr()); // L2
  frc2::JoystickButton(&m_copilotController, 4).WhileTrue(SetElevatorPos(&m_elevator,0.905).ToPtr()); // L4

  // rAlgae Posistion
  frc2::POVButton(&m_copilotController,0).WhileTrue(SetElevatorPos(&m_elevator,0.770).ToPtr());
  frc2::POVButton(&m_copilotController,180).WhileTrue(SetElevatorPos(&m_elevator,0.414).ToPtr());


  //Algae
  frc2::JoystickButton(&m_copilotController, 7).WhileTrue(SetAlgaePosition(&m_algae,2).ToPtr()); // Home pos
  frc2::JoystickButton(&m_copilotController, 5).WhileTrue( // Figure out triggers
    frc2::cmd::Sequence(
      SetAlgaePosition(&m_algae,20).ToPtr(),
      frc2::cmd::Parallel(
        shootCommand(&m_algae, -0.4).ToPtr(),
        SetAlgaePosition(&m_algae,20).ToPtr()
      )
    )
  );

  leftTriggerPressed.WhileTrue(
      frc2::cmd::Sequence(
        frc2::cmd::Parallel(
          shootCommand(&m_algae, 0.7).ToPtr(),
          SetAlgaePosition(&m_algae,20).ToPtr()
        )
      )
  );



  //Coral
  frc2::JoystickButton(&m_copilotController, 6).WhileTrue(SetCoralPosition(&m_coral,18 ).ToPtr()); // Home pos
  frc2::JoystickButton(&m_copilotController, 8).WhileTrue(SetCoralPosition(&m_coral,5).ToPtr()); // REMEMBER CHANGE ANGLE Shoot pos
  // leftTriggerPressed.WhileTrue(SetCoralPosition(&m_coral,10).ToPtr()); // REMEMBER CHANGE ANGLE Shoot pos
  frc2::JoystickButton(&m_copilotController, 3).WhileTrue(SetCoralPosition(&m_coral, 33).ToPtr());//L4 pos
  rightTriggerPressed.WhileTrue(SetCoralPosition(&m_coral, 29.5).ToPtr());

  // frc2::JoystickButton(&m_copilotController,2).WhileTrue(shootCommand(&m_algae, -0.8).ToPtr());
  

  

    
    
    

    

  // frc2::JoystickButton(&m_copilotController, 5).WhileTrue(RunIntake(&m_intake).ToPtr());
  //   frc2::JoystickButton(&m_copilotController, 6).WhileTrue(RunConveyer(&m_conveyer).ToPtr());
  

  //frc2::JoystickButton(&m_driverController, 8).OnTrue(ShootPosition(&m_arm).ToPtr());


  //frc2::JoystickButton(&m_driverController, 8).WhileTrue(RunConveyer(&m_conveyer).ToPtr());
  //frc2::JoystickButton(&m_driverController, 6).OnTrue(frc2::InstantCommand([this]{},{&m_drive,&m_vision}).ToPtr());

}



//VERY IMPORTANT
// constraints for pathplanner in the app
// max velocity: 3mps
// max angular velocity: 180 deg per second



frc2::CommandPtr RobotContainer::getAutonomousCommand(){
  // frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
  //                              AutoConstants::kMaxAcceleration);
  // // Add kinematics to ensure max speed is actually obeyed
  // config.SetKinematics(m_drive.kDriveKinematics);

  // // An example trajectory to follow.  All units in meters.
  //   auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //     // Start at the origin facing the +X direction
  //     frc::Pose2d{0_m, 0_m, 0_deg},
  //     // Pass through these two interior waypoints, making an 's' curve path
  //     {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
  //     // End 3 meters straight ahead of where we started, facing forward
  //     frc::Pose2d{3_m, 0_m, 0_deg},
  //     // Pass the config
  //     config);

  // frc::ProfiledPIDController<units::radians> thetaController{
  //     AutoConstants::kPThetaController, 0, 0,
  //     AutoConstants::kThetaControllerConstraints};

  // thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
  //                                       units::radian_t{std::numbers::pi});

  // frc2::CommandPtr swerveControllerCommand =
  //     frc2::SwerveControllerCommand<4>(
  //         exampleTrajectory, [this]() { return m_drive.GetEstimatedPose(); },

  //         m_drive.kDriveKinematics,

  //         frc::PIDController{0.5, 0, 0},
  //         frc::PIDController{0.5, 0, 0},
  //         thetaController,

  //         [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

  //         {&m_drive})
  //         .ToPtr();

  // // Reset odometry to the initial pose of the trajectory, run path following
  // // command, then stop at the end.
  // return frc2::cmd::Sequence(
  //     frc2::InstantCommand(
  //         [this, &exampleTrajectory]() {
  //           m_drive.ResetOdometry(exampleTrajectory.InitialPose());
  //         },
  //         {})
  //         .ToPtr(),
  //     std::move(swerveControllerCommand),
  //     frc2::InstantCommand(
  //         [this] { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {})
  //         .ToPtr());

  // std::vector<frc::Pose2d> poses{
  //     frc::Pose2d(1.0_m, 0.0_m, frc::Rotation2d(0_deg)),
  //     frc::Pose2d(1.0_m, 0.0_m, frc::Rotation2d(0_deg))
  // };
  // std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(poses);

  // // Create the path using the bezier points created above
  // // We make a shared pointer here since the path following commands require a shared pointer
  // auto path = std::make_shared<PathPlannerPath>(
  //     bezierPoints,
  //     PathConstraints(1.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
  //     GoalEndState(0.0_mps, frc::Rotation2d(0_deg)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  // );

  // // Prevent the path from being flipped if the coordinates are already correct
  // path->preventFlipping = true;



// std::vector<frc::Pose2d> poses{
//     frc::Pose2d(0_m, 0.0_m, frc::Rotation2d(0_deg)),
//     frc::Pose2d(1_m, 0.0_m, frc::Rotation2d(0_deg)),
//     frc::Pose2d(1_m, 1.0_m, frc::Rotation2d(0_deg)),


// };
// std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(poses);

// // Create the path using the bezier points created above
// // We make a shared pointer here since the path following commands require a shared pointer
// auto path = std::make_shared<PathPlannerPath>(
//     bezierPoints,
//     PathConstraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
//     GoalEndState(0.0_mps, frc::Rotation2d(0_deg)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
// );

// // Prevent the path from being flipped if the coordinates are already correct
// path->preventFlipping = true;

// return AutoBuilder::followPath(path);

  
    // return m_chooser.GetSelected();
    // std::string autonomous = m_chooser.GetSelected();

    return PathPlannerAuto("Left Side L1").ToPtr();
    // return autoChooser.GetSelected();

}