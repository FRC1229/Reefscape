// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualAlgae.h"

ManualAlgae::ManualAlgae(AlgaeSubsystem* algae, frc::Joystick* joystick): m_algae(algae), m_CoController(joystick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_algae);
}

// Called when the command is initially scheduled.
void ManualAlgae::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualAlgae::Execute() {
    double speed = 0.0;

    switch (m_CoController->GetPOV()) {
        case 0:
            speed = 0.1;
            break;
        case 180:
            speed = -0.1;
            break;
        default:
            speed = 0.0;
            break;
    }

    m_algae->SetTiltSpeed(speed); // Use a subsystem method instead of direct motor calls
}


// Called once the command ends or is interrupted.
void ManualAlgae::End(bool interrupted) {
    m_algae->SetTiltSpeed(0.0); // Ensure motor stops when command ends
}

// Returns true when the command should end.
bool ManualAlgae::IsFinished() {
    return false; // Consider adding logic to finish under certain conditions
}

