// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ManualElevator.h"
#include "frc/smartdashboard/SmartDashboard.h"

constexpr double kJoystickDeadband = 0.05;
constexpr double kElevatorSpeedFactor = 0.20;
constexpr double kElevatorMaxHeight = 0.95; // meters
constexpr double kElevatorMinHeight = 0.0;  // meters
constexpr units::volt_t kFeedforwardVoltage{0.74};

ManualElevator::ManualElevator(ElevatorSubsystem* elevator, frc::Joystick* controller)
    : m_elevator(elevator), m_controller(controller)  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_elevator);
}

// Called when the command is initially scheduled.
void ManualElevator::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualElevator::Execute() {
    double joystickValue = ApplyDeadband(m_controller->GetRawAxis(5));

  double currentHeight = GetElevatorHeight();

  if (joystickValue > 0 && currentHeight < kElevatorMaxHeight) {
    // Moving up
    SetElevatorSpeed(-joystickValue * kElevatorSpeedFactor);
  } else if (joystickValue < 0 && currentHeight > kElevatorMinHeight) {
    // Moving down
    SetElevatorSpeed(-joystickValue * kElevatorSpeedFactor);
  } else {
    // Hold position with feedforward
    m_elevator->m_controller.SetGoal(m_elevator->currentPos);

    frc::ElevatorFeedforward feedforward{0_V, kFeedforwardVoltage, 2_V / (0.5_mps), 2_V / (0.5_mps_sq)};
    units::volt_t feedforwardCalc = feedforward.Calculate(m_elevator->m_controller.GetSetpoint().velocity);

    SetElevatorVoltage(feedforwardCalc);

    frc::SmartDashboard::PutNumber("Current Position", m_elevator->currentPos.position.value());
    frc::SmartDashboard::PutNumber("Feedforward Voltage", feedforwardCalc.value());
  }
}

// Called once the command ends or is interrupted.
void ManualElevator::End(bool interrupted) {
  SetElevatorSpeed(0.0);
}

// Returns true when the command should end.
bool ManualElevator::IsFinished() {
  return false;
}

double ManualElevator::ApplyDeadband(double value) {
  return (std::abs(value) > kJoystickDeadband) ? value : 0.0;
}

double ManualElevator::GetElevatorHeight() {
  return m_elevator->m_ElevatorEncoderBottom.GetPosition() * 0.025;
}

void ManualElevator::SetElevatorSpeed(double speed) {
  m_elevator->m_ElevatorMotorTop.Set(speed);
  m_elevator->m_ElevatorMotorBottom.Set(speed);
}

void ManualElevator::SetElevatorVoltage(units::volt_t voltage) {
  m_elevator->m_ElevatorMotorTop.SetVoltage(voltage);
  m_elevator->m_ElevatorMotorBottom.SetVoltage(voltage);
}