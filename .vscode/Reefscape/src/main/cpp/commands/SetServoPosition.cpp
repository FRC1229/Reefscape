// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetServoPosition.h"
#include <frc/smartdashboard/SmartDashboard.h>

SetServoPosition::SetServoPosition(L1Subsystem* l1, frc::Joystick* joystick) : m_l1(l1),m_joystick(joystick) {
  CurrentState = a0;
  AddRequirements(m_l1);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetServoPosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetServoPosition::Execute() {

  if(m_joystick->GetRawButtonPressed(2)){

    switch (CurrentState)
    {
    case a45:
      CurrentState=a0;
      break;
    
    case a0:
      CurrentState=a90;
      break;

    case a90:
      CurrentState=a45;
      break;
    }

  }

  switch (CurrentState)
  {
  case a45:
    m_l1->SetAngle(0.15);
    break;
  
  case a0:
    m_l1->SetAngle(0);
    break;

  case a90:
    m_l1->SetAngle(0.5);
    break;
  
  }


  frc::SmartDashboard::PutNumber("servo angle", m_l1->m_servo.GetAngle());
}

// Called once the command ends or is interrupted.
void SetServoPosition::End(bool interrupted) {}

// Returns true when the command should end.
bool SetServoPosition::IsFinished() {
  return false;
}
