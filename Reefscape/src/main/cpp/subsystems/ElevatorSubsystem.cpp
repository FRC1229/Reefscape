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
    1.6,0.0,0.0,
    frc::TrapezoidProfile<units::meters>::Constraints{0.4_mps, 0.4_mps_sq}
    ),
m_controller2(
    1.6,0.1,0.0,
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

void ElevatorSubsystem::SetElevatorPos(frc::TrapezoidProfile<units::meters>::State setPoint,frc::TrapezoidProfile<units::meters>::State setPoint2){

    frc::ElevatorFeedforward m_feedforward(0_V, 0_V, 2_V/(0.5_mps), 2_V/(0.5_mps_sq));

    // Set the SetPoints here
    m_controller.SetGoal(setPoint);
    m_controller2.SetGoal(setPoint2);

    units::meter_t  e1 = units::meter_t{m_ElevatorEncoderTop.GetPosition() * 0.025};
    units::meter_t  e2 = units::meter_t{m_ElevatorEncoderBottom.GetPosition() * 0.025};

    frc::SmartDashboard::PutNumber("Encoder 1", e1.value());
    frc::SmartDashboard::PutNumber("Encoder 2", e2.value());

    units::volt_t feedForwardCalc = m_feedforward.Calculate(m_controller.GetSetpoint().velocity);

    units::volt_t pidCalc1 = units::volt_t{m_controller.Calculate(e1)};
    units::volt_t pidCalc2 = units::volt_t{m_controller.Calculate(e2)};

    frc::SmartDashboard::PutNumber("Volt 1", pidCalc1.value());
    frc::SmartDashboard::PutNumber("Volt 2", pidCalc2.value());

    m_ElevatorMotorTop.SetVoltage(pidCalc1+feedForwardCalc);
    m_ElevatorMotorBottom.SetVoltage(pidCalc2+feedForwardCalc);

    frc::SmartDashboard::PutNumber("Setpoint", setPoint.position.value());




    

}
