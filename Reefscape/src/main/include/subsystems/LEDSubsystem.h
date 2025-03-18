// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <subsystems/ElevatorSubsystem.h>
#include <subsystems/AlgaeSubsystem.h>
#include <frc/LEDPattern.h>

static constexpr int kLength = 120;

class LEDSubsystem : public frc2::SubsystemBase {
 public:
  LEDSubsystem();
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  void SetLedColor(int r, int g, int b, int length);
  void Rainbow(int saturation, int value);
  void ScrollEffect();

 private:
  frc::AddressableLED m_led{8};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

  ElevatorSubsystem* m_Elevator;
  AlgaeSubsystem* m_algae;
};
