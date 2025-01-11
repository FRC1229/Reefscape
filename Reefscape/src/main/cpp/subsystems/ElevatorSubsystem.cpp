// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

ElevatorSubsystem::ElevatorSubsystem():
m_ElevatorMotorBottom(ElevatorConstants::kElevatorMotorBottomID, rev::spark::SparkMax::MotorType::kBrushless),
m_ElevatorMotorTop(ElevatorConstants::kElevatorMotorTopID, rev::spark::SparkMax::MotorType::kBrushless),
m_ElevatorEncoderBottom(m_ElevatorMotorBottom.GetAbsoluteEncoder()),
m_ElevatorEncoderTop(m_ElevatorMotorTop.GetAbsoluteEncoder())
{

}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {

}
double ElevatorSubsystem::readEncoder(double encodervalue){
    return m_ElevatorEncoderTop.GetPosition();
    return m_ElevatorEncoderBottom.GetPosition();
    // frc::SmartDashboard::PutNumber("shoulder", shoulder.GetDistance());
}

void ElevatorSubsystem::SetElevatorSpeed(double speed){
    // m_ElevatorMotor.Set(speed);
} 

void ElevatorSubsystem::HomePosition(){
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = HomePositionTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = HomePositionBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}

void ElevatorSubsystem::L1CoralPosition(){
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L1CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L1CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}

void ElevatorSubsystem::L2CoralPosition(){
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L2CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L2CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}
void ElevatorSubsystem::L3CoralPosition(){
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L3CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L3CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}

void ElevatorSubsystem::L4CoralPosition(){
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L4CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L4CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}