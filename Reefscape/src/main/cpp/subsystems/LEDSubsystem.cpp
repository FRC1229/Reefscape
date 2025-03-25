// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSubsystem.h"
#include <frc/LEDPattern.h>


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

void LEDSubsystem::Rainbow(int sat, int val){
    frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);
    frc::LEDPattern m_scrollingRainbow =
    m_rainbow.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);
    m_scrollingRainbow.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}

void LEDSubsystem::Scroll(frc::Color col) {
    std::array<std::pair<double, frc::Color>, 2> maskSteps{std::pair{0.0, col},
                                                  std::pair{0.1, frc::Color::kBlack}};
  frc::LEDPattern base = frc::LEDPattern::Rainbow(255, 255);
  frc::LEDPattern mask = frc::LEDPattern::Steps(maskSteps).ScrollAtRelativeSpeed(units::hertz_t{0.25});

  frc::LEDPattern pattern =  base.Mask(mask);
  pattern.ApplyTo(m_ledBuffer);  
  m_led.SetData(m_ledBuffer);
}

void LEDSubsystem::sideLed(std::string sideChoose){
    std::array<frc::AddressableLED::LEDData, 120> m_buffer;
    frc::LEDPattern color = frc::LEDPattern::Solid(frc::Color::kRed);
    color.ApplyTo(m_ledBuffer);
    if (sideChoose == "left") {
       for(int i = 0; i < 60; i++){
        m_buffer[i] = m_ledBuffer[i];
        } 
    } else if (sideChoose == "right"){
        for(int i = 0; i < 60; i++){
        m_buffer[i+60] = m_ledBuffer[i];
        }

    }
    m_led.SetData(m_buffer);
    
}