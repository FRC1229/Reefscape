// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeSubsystem.h"

AlgaeSubsystem::AlgaeSubsystem():
m_AlgaeTiltMotor(23,rev::spark::SparkMax::MotorType::kBrushless),
m_AlgaeTiltEncoder(m_AlgaeTiltMotor.GetEncoder())
{
    m_AlgaeTiltMotor.SetInverted(true);
}

// This method will be called once per scheduler run
void AlgaeSubsystem::Periodic() {}

void AlgaeSubsystem::ManualTilt(){

}

double AlgaeSubsystem::GetAngle(){
    return m_AlgaeTiltEncoder.GetPosition();
}

void AlgaeSubsystem::MoveToAngle(double angle){
    double volt = m_AlgaeController.Calculate(GetAngle(), angle);
    m_AlgaeTiltMotor.SetVoltage(units::volt_t{volt});

}

void AlgaeSubsystem::SetTiltSpeed(double speed) {
  double currentAngle = GetAngle();

  // Check if the desired movement is within the allowable range
  if ((currentAngle <= m_MinAngle && speed < 0) ||
      (currentAngle >= m_MaxAngle && speed > 0)) {
    // Stop the motor if it's about to move beyond the limits
    m_AlgaeTiltMotor.Set(0.0);
  } else {
    m_AlgaeTiltMotor.Set(speed);
  }
}

