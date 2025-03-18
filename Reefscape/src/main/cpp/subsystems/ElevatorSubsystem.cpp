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
    20,0.0,0.0,
    frc::TrapezoidProfile<units::meters>::Constraints{0.4_mps, 0.4_mps_sq}
    ),
m_controller2(
    1.6,0.1,0.0,
    frc::TrapezoidProfile<units::meters>::Constraints{0.4_mps, 0.4_mps_sq}
),
m_feedforward(0_V, 0.74_V, 2_V/(0.5_mps), 2_V/(0.5_mps_sq)),
m_controllerPid(9,0.20,0)
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

void ElevatorSubsystem::SetElevatorPos(double setpoint){

    // Set the SetPoints here
    if(accelScale <= 1){
        accelScale+=0.1;
    }

    double  e1 = m_ElevatorEncoderTop.GetPosition() * 0.025;
    double  e2 = m_ElevatorEncoderBottom.GetPosition() * 0.025;

    currentPos = {units::meter_t{e1},0_mps};

    frc::SmartDashboard::PutNumber("Encoder 1", e1);
    frc::SmartDashboard::PutNumber("Encoder 2", e2);

    units::volt_t feedForwardCalc = m_feedforward.Calculate(m_controller.GetSetpoint().velocity);

    units::volt_t pidCalc1 = units::volt_t{m_controllerPid.Calculate(e1,setpoint)};
    units::volt_t pidCalc2 = units::volt_t{m_controllerPid.Calculate(e2,setpoint)};

    frc::SmartDashboard::PutNumber("Volt 1", pidCalc1.value());
    frc::SmartDashboard::PutNumber("Volt 2", pidCalc2.value());
    if(setpoint >= 0.005){
        m_ElevatorMotorTop.SetVoltage(frc::ApplyDeadband((pidCalc1*accelScale)+0.74_V,0_V,1.5_V));
        m_ElevatorMotorBottom.SetVoltage(frc::ApplyDeadband((pidCalc1*accelScale)+0.74_V,0_V,1.5_V));
    }
    else{
        m_ElevatorMotorTop.SetVoltage(frc::ApplyDeadband((pidCalc1*accelScale),0_V,1.5_V));
        m_ElevatorMotorBottom.SetVoltage(frc::ApplyDeadband((pidCalc1*accelScale),0_V,1.5_V));
    }
    // frc::SmartDashboard::PutNumber("Setpoint", setPoint.position.value());




    

}
