// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "commands/ManualGroundAlgae.h"

// ManualGroundAlgae::ManualGroundAlgae(GroundAlgaeSubsystem* algae, frc::Joystick* joystick): m_algae(algae), m_CoController(joystick) {
//   // Use addRequirements() here to declare subsystem dependencies.
//   AddRequirements(m_algae);
// }

// // Called when the command is initially scheduled.
// void ManualGroundAlgae::Initialize() {}

// // Called repeatedly when this Command is scheduled to run
// void ManualGroundAlgae::Execute() {

//     m_algae->m_AlgaeTiltMotor.Set(0);
//     if(m_CoController->GetPOV(180) > 0.05){
//       m_algae->m_AlgaeTiltMotor.Set(0.1);
//     }
//     else if(m_CoController->GetRawAxis(1) < -0.05){
//       m_algae->m_AlgaeTiltMotor.Set(-0.1);
//     }
//     else{
//       m_algae->m_AlgaeTiltMotor.Set(0);
//     }


// }

// // Called once the command ends or is interrupted.
// void ManualGroundAlgae::End(bool interrupted) {}

// // Returns true when the command should end.
// bool ManualGroundAlgae::IsFinished() {
//   return false;
// }
