// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/SetElevatorPos.h"
#include <frc/smartdashboard/SmartDashboard.h>

SetElevatorPos::SetElevatorPos(ElevatorSubsystem* subsystem, double dis) : m_elevator(subsystem), distance(dis) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void SetElevatorPos::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetElevatorPos::Execute() {
  // if({
  //   Cancel();
  //   // m_drive->Drive(units::velocity::meters_per_second_t(0), units::velocity::meters_per_second_t(0), units::angular_velocity::radians_per_second_t(-rotationCalc), true);
  // }
  // m_elevator->goal = {0.5_m,0_mps};
  // m_elevator->currentPos = distance;
  // m_elevator->currentPos2 = {units::meter_t{m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025},0_mps};
  
  m_elevator->SetElevatorPos(distance);
  
}


// Called once the command ends or is interrupted.
void SetElevatorPos::End(bool interrupted) {
  frc::SmartDashboard::PutNumber("Command Canceled", m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025);
  m_elevator->accelScale = 0;
  // m_elevator->m_ElevatorMotorBottom.Set(0);
  // m_elevator->m_ElevatorMotorTop.Set(0);

}

// Returns true when the command should end.
bool SetElevatorPos::IsFinished() {
  // return false;
  return ((distance + 0.001) > m_elevator->m_ElevatorEncoderBottom.GetPosition() * 0.025 && m_elevator->m_ElevatorEncoderBottom.GetPosition() * 0.025 > (distance - 0.001));
}
