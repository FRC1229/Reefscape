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

using namespace DriveConstants;
using namespace pathplanner;


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Configure the button bindings
  
  // frc2::CommandPtr threeAndHalf = pathplanner::PathPlannerAuto("New Auto(4)").ToPtr();
  // frc2::CommandPtr sideNote = pathplanner::PathPlannerAuto("OneNoteSide").ToPtr();

  // m_chooser.AddOption("3.5 Note", threeAndHalf.get());
  // m_chooser.AddOption("Side Auto", sideNote.get());
  m_chooser.SetDefaultOption("One Meter", "One meter");
  m_chooser.AddOption("One Meter", "One meter");
 
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
}

void RobotContainer::ConfigureButtonBindings() {

  //frc2::JoystickButton(&m_driverController, 2).OnTrue(TurnToAngle(&m_drive, &m_driverController, 90.0).ToPtr());
  //frc2::JoystickButton(&m_driverController, 3).OnTrue(TurnToAngle(&m_drive, &m_driverController, -90.0).ToPtr());
  frc2::JoystickButton(&m_driverController, 4).OnTrue(frc2::cmd::RunOnce([this]{m_drive.ZeroHeading();}));
  frc2::JoystickButton(&m_driverController, 6).OnTrue(frc2::cmd::RunOnce([this]{m_drive.GetCurrentCommand()->Cancel();}));
  //frc2::Trigger([this]{return m_copilotController.GetRawAxis(2)>0.1;}).WhileTrue(frc2::cmd::Run([this]{m_conveyer.RunConveyer();},{&m_conveyer}));
   //frc2::Trigger([this]{return m_copilotController.GetRawAxis(3)>0.1;}).WhileTrue(frc2::cmd::Run([this]{m_conveyer.RunConveyer(true);},{&m_conveyer}));
  //   frc2::cmd::Sequence(
  //   RunConveyer(&m_conveyer).ToPtr().Until([this]{return sensor.Get();}),
  //   frc2::InstantCommand([this]{m_shooter.GetCurrentCommand()->Cancel();}).ToPtr()
  //   )
  //   );

  

    
    
    

    

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

    return PathPlannerAuto("One meter").ToPtr();
    // return PathPlannerAuto(autonomous).ToPtr();

}