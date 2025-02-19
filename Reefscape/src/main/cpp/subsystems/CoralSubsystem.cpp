// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"

/*
The constructor initializes the CoralSubsystem by setting up the tilt motor and encoder. It uses a brushless SparkMax motor (m_CoralTilt) for controlling the coral mechanism's tilt and initializes the encoder (m_CoralEncoder) to track the tilt angle.

Members Initialized
m_CoralTilt - The motor responsible for tilting the coral mechanism. It is set as a brushless motor connected to port 24.
m_CoralEncoder - The encoder associated with the m_CoralTilt motor to monitor its position (i.e., the tilt angle).
*/


CoralSubsystem::CoralSubsystem(): 
m_CoralTilt(24,rev::spark::SparkMax::MotorType::kBrushless), 
m_CoralEncoder(m_CoralTilt.GetEncoder())
{

}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {}

double CoralSubsystem::GetAngle(){
    return m_CoralEncoder.GetPosition();
}

void CoralSubsystem::MoveToAngle(double angle){
    double volt = m_coralController.Calculate(GetAngle(), angle);
    m_CoralTilt.SetVoltage(units::volt_t{volt});
}
