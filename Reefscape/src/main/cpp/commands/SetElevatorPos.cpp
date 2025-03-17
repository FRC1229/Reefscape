#include "Commands/SetElevatorPos.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

SetElevatorPos::SetElevatorPos(ElevatorSubsystem* subsystem, double targetPosition)
    : m_elevator(subsystem), m_targetPosition(targetPosition),
      m_controller(
          frc::TrapezoidProfile<units::meters>::Constraints{
              units::meters_per_second_t(kMaxVelocity),
              units::meters_per_second_squared_t(kMaxAcceleration)},
          0.0) {
  AddRequirements({subsystem});
}

void SetElevatorPos::Initialize() {
  m_controller.SetGoal(m_targetPosition);
}

void SetElevatorPos::Execute() {
  double currentPosition = m_elevator->GetCurrentHeight();
  auto profileState = m_controller.Calculate(currentPosition);

  double pidOutput = m_controller.Calculate(currentPosition);
  double feedforward = m_feedforward.Calculate(profileState.velocity);

  m_elevator->SetMotorOutput(pidOutput + feedforward);

  frc::SmartDashboard::PutNumber("Elevator Position", currentPosition);
  frc::SmartDashboard::PutNumber("Elevator Velocity", profileState.velocity.to<double>());
}

void SetElevatorPos::End(bool interrupted) {
  m_elevator->SetMotorOutput(0.0);
}

bool SetElevatorPos::IsFinished() {
  return m_controller.AtGoal();
}
