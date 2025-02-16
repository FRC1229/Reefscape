// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeSubsystem.h"

AlgaeSubsystem::AlgaeSubsystem():
m_AlgaeMotor(22, rev::spark::SparkMax::MotorType::kBrushless),
m_AlgaeTiltMotor(23,rev::spark::SparkMax::MotorType::kBrushless)
{

}

// This method will be called once per scheduler run
void AlgaeSubsystem::Periodic() {}

void AlgaeSubsystem::run(double speed){
    m_AlgaeMotor.Set(speed);
}

void AlgaeSubsystem::ManualTilt(){

}

