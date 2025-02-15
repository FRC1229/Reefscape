// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

ElevatorSubsystem::ElevatorSubsystem():
m_ElevatorMotorBottom(ElevatorConstants::kElevatorMotorID, rev::spark::SparkMax::MotorType::kBrushless),
m_ElevatorMotorTop(ElevatorConstants::kElevatorUpperMotorID, rev::spark::SparkMax::MotorType::kBrushless),
m_ElevatorEncoderBottom(m_ElevatorMotorBottom.GetEncoder()),
m_ElevatorEncoderTop(m_ElevatorMotorTop.GetEncoder()),
m_controller(
    2.3,0.1,0.05,
    frc::TrapezoidProfile<units::meters>::Constraints{0.4_mps, 0.4_mps_sq}
    )
{

    m_ElevatorMotorBottom.SetInverted(true);
    m_ElevatorMotorTop.SetInverted(false);  
    
    // frc::TrapezoidProfile<units::meters>::State goal;




}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {

}

units::meter_t ElevatorSubsystem::getDistance(){
    return units::meter_t{m_ElevatorEncoderTop.GetPosition() * 0.025};
}


double ElevatorSubsystem::readEncoder(){
    return (m_ElevatorEncoderTop.GetPosition()/0.786)+2,(m_ElevatorEncoderBottom.GetPosition()/0.786)+2;
    // frc::SmartDashboard::PutNumber("shoulder", shoulder.GetDistance());
}

void ElevatorSubsystem::SetElevatorSpeed(double speed){
    // m_ElevatorMotor.Set(speed);
} 

void ElevatorSubsystem::SetElevatorPos(frc::TrapezoidProfile<units::meters>::State setPoint){
    // Set the SetPoints here
    



}
