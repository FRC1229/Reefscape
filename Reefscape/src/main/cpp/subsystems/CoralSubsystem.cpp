// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"

CoralSubsystem::CoralSubsystem(): 
m_CoralTilt(24,rev::spark::SparkMax::MotorType::kBrushless), 
m_CoralEncoder(m_CoralTilt.GetEncoder()),
m_servoL {7},
m_servoR {6}
{
    m_CoralTilt.SetInverted(true);
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {}

double CoralSubsystem::GetAngle(){
    return m_CoralEncoder.GetPosition();
}

void CoralSubsystem::ShootToggle(){
    if(shot){
        m_servoL.Set(0);
        m_servoR.Set(0);
    }
    else{
        m_servoL.Set(0.1);
        m_servoR.Set(0.1);
    }
}

void CoralSubsystem::MoveToAngle(double angle){
    double volt = m_coralController.Calculate(GetAngle(), angle);
    m_CoralTilt.SetVoltage(units::volt_t{volt});
}
