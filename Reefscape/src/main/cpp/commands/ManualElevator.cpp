// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualElevator.h"

/*Initializes the ManualElevator command, allowing manual control of the ElevatorSubsystem using a joystick.

Parameters
elevator - Pointer to the ElevatorSubsystem, responsible for controlling the robot's elevator mechanism.
m_controller - Pointer to the frc::Joystick, used to manually control the elevator movement.*/

ManualElevator::ManualElevator(ElevatorSubsystem* elevator, frc::Joystick* m_controller): m_elevator(elevator),m_CoController(m_controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_elevator);
}

// Called when the command is initially scheduled.
void ManualElevator::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualElevator::Execute() {
  if(m_CoController->GetRawAxis(5) > 0.05){

    // if(m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025 < 0.95){

    if(m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025 > 0){
      
      m_elevator->m_ElevatorMotorTop.Set(-m_CoController->GetRawAxis(5)*0.2);
      m_elevator->m_ElevatorMotorBottom.Set(-m_CoController->GetRawAxis(5)*0.2);

    }
    
    m_elevator->currentPos = {units::meter_t{m_elevator->m_ElevatorEncoderTop.GetPosition()*0.025},0_mps};
    m_elevator->currentPos2 = {units::meter_t{m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025},0_mps};
    // }
    // else{
    //   m_elevator->m_ElevatorMotorTop.Set(0);
    //   m_elevator->m_ElevatorMotorBottom.Set(0);
    // }
  }
  else if(m_CoController->GetRawAxis(5) < -0.05){
   
      m_elevator->m_ElevatorMotorTop.Set(-m_CoController->GetRawAxis(5)*0.2);
      m_elevator->m_ElevatorMotorBottom.Set(-m_CoController->GetRawAxis(5)*0.2);

      m_elevator->currentPos = {units::meter_t{m_elevator->m_ElevatorEncoderTop.GetPosition()*0.025},0_mps};
      m_elevator->currentPos2 = {units::meter_t{m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025},0_mps};
    // }
    // else{
    //   m_elevator->m_ElevatorMotorTop.Set(0);
    //   m_elevator->m_ElevatorMotorBottom.Set(0);
    // }
  }
  else{
    // m_elevator->SetElevatorPos(m_elevator->currentPos, m_elevator->currentPos2);
    m_elevator->m_ElevatorMotorTop.Set(0);
    m_elevator->m_ElevatorMotorBottom.Set(0);
  }

}

// Called once the command ends or is interrupted.
void ManualElevator::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualElevator::IsFinished() {
  return false;
}
