// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <subsystems/ElevatorSubsystem.h>
#include <subsystems/AlgaeSubsystem.h>
#include <frc/Timer.h>


class LEDSubsystem : public frc2::SubsystemBase {
 public:
  LEDSubsystem();
  ElevatorSubsystem* m_Elevator;
  AlgaeSubsystem* m_algae;
  frc::AddressableLED m_led{8};
  frc::Timer m_ledTimer;
  

  
  std::array<frc::AddressableLED::LEDData, 121> m_ledBuffer; 

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetLedColor(int r, int g, int b, int length);
  void Blink();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
