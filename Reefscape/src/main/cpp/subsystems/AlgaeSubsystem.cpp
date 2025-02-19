// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeSubsystem.h"

/*
The constructor initializes the AlgaeSubsystem by configuring the motors and encoder. It sets up the m_AlgaeMotor and m_AlgaeTiltMotor as brushless SparkMax motors, and initializes the m_AlgaeTiltEncoder with the encoder from the tilt motor.

Members Initialized
m_AlgaeMotor - The motor responsible for controlling the algae mechanism's primary motion.
m_AlgaeTiltMotor - The motor responsible for tilting the algae mechanism.
m_AlgaeTiltEncoder - The encoder associated with the tilt motor to track the current tilt angle.
*/

AlgaeSubsystem::AlgaeSubsystem():
m_AlgaeMotor(22, rev::spark::SparkMax::MotorType::kBrushless),
m_AlgaeTiltMotor(23,rev::spark::SparkMax::MotorType::kBrushless),
m_AlgaeTiltEncoder(m_AlgaeTiltMotor.GetEncoder())
{

}

// This method will be called once per scheduler run
void AlgaeSubsystem::Periodic() {}

void AlgaeSubsystem::run(double speed){
    m_AlgaeMotor.Set(speed);
}

void AlgaeSubsystem::ManualTilt(){

}

double AlgaeSubsystem::GetAngle(){
    return m_AlgaeTiltEncoder.GetPosition();
}

void AlgaeSubsystem::MoveToAngle(double angle){
    if(!(GetAngle() >= angle-1 && GetAngle() <= angle+1)){
        double volt = m_AlgaeController.Calculate(GetAngle(), angle);
        m_AlgaeTiltMotor.SetVoltage(units::volt_t{volt});
    }
}
