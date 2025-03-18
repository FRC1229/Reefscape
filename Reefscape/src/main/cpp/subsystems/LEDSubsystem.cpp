// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSubsystem.h"
#include <frc/LEDPattern.h>

// LED setup
frc::AddressableLED m_led{8};
std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

// Constructor
LEDSubsystem::LEDSubsystem() {
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

// Periodic: Automatically updates LED feedback based on field conditions
void LEDSubsystem::Periodic() {
   /* if (/* Condition: Vision target locked ) {
        SetLedColor(0, 255, 0, kLength);  // Green for success
    } else if (/* Condition: Auto-aligning ) {
        ScrollEffect();  // Scrolling effect while aligning
    } else if (/* Condition: Error detected ) {
        SetLedColor(255, 0, 0, kLength);  // Red for error
    } else {
        Rainbow(255, 128);  // Default to rainbow effect
    }     
    
    */
}

// Sets the entire LED strip to a solid color
void LEDSubsystem::SetLedColor(int r, int g, int b, int length) {
    for (int i = 0; i < length; i++) {
        m_ledBuffer[i].SetRGB(r, g, b);
    }
    m_led.SetData(m_ledBuffer);
}

// Creates a scrolling rainbow effect
void LEDSubsystem::Rainbow(int saturation, int value) {
    units::meter_t kLedSpacing{1 / 120.0};  // LED spacing
    frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(saturation, value);
    frc::LEDPattern m_scrollingRainbow = m_rainbow.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);
    
    m_scrollingRainbow.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}

// Creates a scrolling mask effect on top of the rainbow
void LEDSubsystem::ScrollEffect() {
    std::array<std::pair<double, frc::Color>, 2> maskSteps{
        std::pair{0.0, frc::Color::kBlue},
        std::pair{0.1, frc::Color::kBlack}
    };

    frc::LEDPattern base = frc::LEDPattern::Rainbow(255, 255);
    frc::LEDPattern mask = frc::LEDPattern::Steps(maskSteps).ScrollAtRelativeSpeed(units::hertz_t{0.25});
    frc::LEDPattern pattern = base.Mask(mask);
    
    pattern.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}