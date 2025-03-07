// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>
#include "Constants.h"
#include "frc/DigitalInput.h"
#include "subsystems/DriveSubsystem.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <subsystems/VisionSubsystem.h>
#include <subsystems/ElevatorSubsystem.h>
#include <subsystems/AlgaeSubsystem.h>
#include <subsystems/CoralSubsystem.h>
#include <subsystems/L1Subsystem.h>
#include <subsystems/LEDSubsystem.h>
#include <subsystems/RollerSubsystem.h>



/*
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  //frc2::Command* GetAutonomousCommand();
  
  frc2::CommandPtr getAutonomousCommand();
  frc::SendableChooser<frc2::Command*> autoChooser;

  // frc2::CommandPtr AutonomousCommand();
  DriveSubsystem m_drive;
  VisionSubsystem m_vision;
  ElevatorSubsystem m_elevator;
  AlgaeSubsystem m_algae;
  CoralSubsystem m_coral;
  L1Subsystem m_l1;
  LEDSubsystem m_Led;
  RollerSubsystem m_roller;


  frc::DigitalInput sensor{0};
  //frc::DigitalInput m_limitSwitch{9};

  frc::Timer intake_timer;


  

  
  
  double increment;
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_copilotController{OIConstants::kCoPilotControllerPort};



  
  //frc::SendableChooser<frc2::Command*> m_chooser;
  frc::SendableChooser<std::string> m_chooser;



 private:
  // The driver's controller

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems

  // The chooser for the autonomous routines

  // frc2::CommandPtr m_Auto1 = PathPlannerAuto("90").ToPtr();  


  // m_chooser.SetDefaultCommand("90 degree", m_auto)

  void ConfigureButtonBindings();

};
