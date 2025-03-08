// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"
#include <frc/controller/PIDController.h>

CoralSubsystem::CoralSubsystem(): 
m_CoralTilt(24,rev::spark::SparkMax::MotorType::kBrushless), 
m_CoralEncoder(m_CoralTilt.GetEncoder()),
m_coralController(0.05, 0.0, 0.0) // PID constants: P=0.05, I=0.0, D=0.0
{
    m_CoralTilt.SetInverted(true);
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {}

double CoralSubsystem::GetAngle(){
    return m_CoralEncoder.GetPosition();
}

// Move to a specific angle using PID
void CoralSubsystem::MoveToAngle(double angle) {
    // Calculate the PID output based on the target angle
    double pidOutput = m_coralController.Calculate(GetAngle(), angle);
    // Apply the PID output as voltage to the motor
    m_CoralTilt.SetVoltage(units::volt_t{pidOutput});
}
