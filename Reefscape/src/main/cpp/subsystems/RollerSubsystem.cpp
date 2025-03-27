// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RollerSubsystem.h"

RollerSubsystem::RollerSubsystem()
: m_RollerMotor(22, rev::spark::SparkMax::MotorType::kBrushless)
{
    }

// This method will be called once per scheduler run
void RollerSubsystem::Periodic() {

}

void RollerSubsystem::run(double speed){
    m_RollerMotor.Set(speed);
}   

// Stops the roller
void RollerSubsystem::Stop() {
    m_RollerMotor.Set(0);
}