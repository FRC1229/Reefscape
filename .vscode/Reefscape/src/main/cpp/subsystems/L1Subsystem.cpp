// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/L1Subsystem.h"

L1Subsystem::L1Subsystem(): m_servo {9}
{

}

// This method will be called once per scheduler run
void L1Subsystem::Periodic() {}

void L1Subsystem::SetAngle(double angle){
    m_servo.SetPosition(angle);
}

