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
    0.02, 0, 0, 
    frc::TrapezoidProfile<units::meters>::Constraints{0.4_mps, 0.4_mps_sq})
{
    double kP=0.02;
    double kI=0.0;
    double kD=0.00;

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

void ElevatorSubsystem::SetElevatorPos(double setPoint){
    // Set the SetPoints here
    
    // double degrees = 0;
    // // double volt = 0;
    // double voltTop = 0;
    // double degreesTop = 0;

    // units::volt_t kS = 0_V;
    // units::volt_t kG = 0_V;
    // auto kV = 2_V/(0.5_mps);
    // auto kA = 2_V/(0.5_mps_sq);

    
    // frc::ElevatorFeedforward m_feedforward(kS, kG, kV, kA);


    double kP=1;
    double kI=0.0;
    double kD=0.00;
    units::volt_t kS = 0_V;
    units::volt_t kG = 0_V;
    auto kV = 2_V/(0.5_mps);
    auto kA = 2_V/(0.5_mps_sq);
    double manualSpeed;

    double circumference = 0.025; // Pre-calculated circumference in meters
    double revolutions11 = m_ElevatorEncoderBottom.GetPosition(); // Get revolutions from encoder
    double revolutions12 = m_ElevatorEncoderTop.GetPosition(); // Get revolutions from encoder
    double distanceInMeters11 = revolutions11 * circumference; // Convert to meters
    double distanceInMeters12 = revolutions12 * circumference; // Convert to meters


    // Creates a PIDController with gains kP, kI, and kD
    frc::ProfiledPIDController<units::meters> controller(
    kP, kI, kD, 
    frc::TrapezoidProfile<units::meters>::Constraints{1_mps, 0.4_mps_sq});
    
    frc::PIDController pid {2,0,0};

    frc::ElevatorFeedforward m_feedforward(kS, kG, kV, kA);


    frc::TrapezoidProfile<units::meters>::State goal = {units::meter_t(setPoint), 0_mps};
    controller.SetGoal(goal);
    units::meter_t e11Position = units::meter_t{distanceInMeters11};
    units::meter_t e12Position = units::meter_t{distanceInMeters12};
    // units::volt_t pidCalc11 = units::volt_t{controller.Calculate(units::meter_t(e11Position))};
    // units::volt_t pidCalc12 = units::volt_t{controller.Calculate(units::meter_t(e12Position))};

    units::volt_t pidCalc11 = units::volt_t{pid.Calculate(distanceInMeters11,setPoint)};
    units::volt_t pidCalc12 = units::volt_t{pid.Calculate(distanceInMeters12,setPoint)};
    
    units::volt_t feedForwardCalc = m_feedforward.Calculate(m_controller.GetSetpoint().velocity);
    m_ElevatorMotorBottom.SetVoltage(pidCalc11+feedForwardCalc);
    m_ElevatorMotorTop.SetVoltage(pidCalc12+feedForwardCalc);

    frc::SmartDashboard::PutNumber("Volt Top", pidCalc11.value());
    frc::SmartDashboard::PutNumber("Volt Bot", pidCalc12.value());


    frc::SmartDashboard::PutNumber("bot Dis", distanceInMeters11);
    frc::SmartDashboard::PutNumber("upper Dis",distanceInMeters12);
    frc::SmartDashboard::PutNumber("SetPoint", goal.position.value());


    // m_ElevatorMotorBottom.SetVoltage(voltCalcLower);
    // m_ElevatorMotorTop.SetVoltage(voltCalcUpper);
    

    // m_ElevatorMotorBottom.Set(0.1);

    

    // degreesTop = readEncoder(m_ElevatorEncoderTop.GetPosition());
    // voltTop = HomePositionTopElevatorPID.Calculate(degreesTop, TopElevatorSetPoint);
    // frc::SmartDashboard::PutNumber("volt calc", voltTop);
    // m_ElevatorMotorTop.SetVoltage(units::volt_t{voltTop});

    // degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    // volt = HomePositionBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    // frc::SmartDashboard::PutNumber("volt calc", volt);
    // m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});



}
