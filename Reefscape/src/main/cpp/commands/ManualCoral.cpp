// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualCoral.h"
// #include "frc/sm"
#include <frc/MathUtil.h> // Required for ApplyDeadband


ManualCoral::ManualCoral(CoralSubsystem* coral, frc::Joystick* joy): m_coral(coral), m_CoController(joy) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_coral);
}

// Called when the command is initially scheduled.
void ManualCoral::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualCoral::Execute() {
    // Get joystick input and apply WPILib deadband function
    double input = frc::ApplyDeadband(m_CoController->GetRawAxis(1), 0.05);

    // Apply scaling factor
    double speed = -input * 0.1;

    // Set motor speed
    m_coral->m_CoralTilt.Set(speed);
}

// Called once the command ends or is interrupted.
void ManualCoral::End(bool interrupted) {
    // Ensure the motor stops when the command ends
    m_coral->m_CoralTilt.Set(0);
}

// Returns true when the command should end.
bool ManualCoral::IsFinished() {
    return false; // Keeps running until interrupted
}
