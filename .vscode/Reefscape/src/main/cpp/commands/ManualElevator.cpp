// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualElevator.h"
#include "frc/smartdashboard/SmartDashboard.h"

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
      
      m_elevator->m_ElevatorMotorTop.Set(-m_CoController->GetRawAxis(5)*0.20);
      m_elevator->m_ElevatorMotorBottom.Set(-m_CoController->GetRawAxis(5)*0.20);

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
   
    m_elevator->m_ElevatorMotorTop.Set(-m_CoController->GetRawAxis(5)*0.20);
    m_elevator->m_ElevatorMotorBottom.Set(-m_CoController->GetRawAxis(5)*0.20);

    m_elevator->currentPos = {units::meter_t{m_elevator->m_ElevatorEncoderTop.GetPosition()*0.025},0_mps};
    m_elevator->currentPos2 = {units::meter_t{m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025},0_mps};

  }
  else{
    
    m_elevator->m_controller.SetGoal(m_elevator->currentPos); 

    frc::ElevatorFeedforward m_feedforward(0_V, 0.74_V, 2_V/(0.5_mps), 2_V/(0.5_mps_sq)); 

    units::volt_t feedForwardCalc = m_feedforward.Calculate(m_elevator->m_controller.GetSetpoint().velocity);

    // m_elevator->m_ElevatorMotorTop.SetVoltage(feedForwardCalc);
    // m_elevator->m_ElevatorMotorBottom.SetVoltage(feedForwardCalc);

    frc::SmartDashboard::PutNumber("current pos", m_elevator->currentPos.position.value());
    frc::SmartDashboard::PutNumber("feedFoward volt", feedForwardCalc.value());
    
   
    if(m_elevator->m_ElevatorEncoderBottom.GetPosition()*0.025 > 0.05){
      m_elevator->m_ElevatorMotorTop.SetVoltage(units::volt_t{0.74});
      m_elevator->m_ElevatorMotorBottom.SetVoltage(units::volt_t{0.74});
    }



  }

}

// Called once the command ends or is interrupted.
void ManualElevator::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualElevator::IsFinished() {
  return false;
}
