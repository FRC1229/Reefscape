// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeSubsystem.h"
#include <frc/controller/PIDController.h>

AlgaeSubsystem::AlgaeSubsystem():
m_AlgaeTiltMotor(23,rev::spark::SparkMax::MotorType::kBrushless),
m_AlgaeTiltEncoder(m_AlgaeTiltMotor.GetEncoder()),
m_AlgaeController(0.05, 0.0, 0.0) // PID constants: P=0.05, I=0.0, D=0.0
{
    m_AlgaeTiltMotor.SetInverted(true); // Inverting the motor if necessary
}

// This method will be called once per scheduler run
void AlgaeSubsystem::Periodic() {}


// Placeholder for manual tilt, not modified
void AlgaeSubsystem::ManualTilt(){

}

// Get the current angle from the encoder
double AlgaeSubsystem::GetAngle(){
    return m_AlgaeTiltEncoder.GetPosition();
}

// Move to a specific angle using PID
void AlgaeSubsystem::MoveToAngle(double angle) {
    // Calculate the PID output
    double pidOutput = m_AlgaeController.Calculate(GetAngle(), angle);
    // Apply the PID output as voltage to the motor
    m_AlgaeTiltMotor.SetVoltage(units::volt_t{pidOutput});
}
