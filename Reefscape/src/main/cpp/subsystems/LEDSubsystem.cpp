// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSubsystem.h"


LEDSubsystem::LEDSubsystem()
{
    m_led.SetLength(121);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
    
};

// This method will be called once per scheduler run
void LEDSubsystem::Periodic() {

}

void LEDSubsystem::SetLedColor(int r, int g, int b, int length){
   for(int i = 0; i<length;i++){
        m_ledBuffer[i].SetRGB(r,g,b);
    }

    m_led.SetData(m_ledBuffer);


}