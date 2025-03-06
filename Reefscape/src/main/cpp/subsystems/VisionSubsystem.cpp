// #include "subsystems/VisionSubsystem.h"
// #include "networktables/NetworkTable.h"
// #include "networktables/NetworkTableInstance.h"
// #include "networktables/NetworkTableEntry.h"
// #include "networktables/NetworkTableValue.h"
// #include <frc/smartdashboard/SmartDashboard.h>
// #include <vector>

// VisionSubsystem::VisionSubsystem() {
   

//     //Coral Reef Tags and Angles
//     //Blue side
//     aprilTagAngles[17] = 45;
//     aprilTagAngles[18] = 45;
//     aprilTagAngles[19] = 45;
//     aprilTagAngles[20] = 45;
//     aprilTagAngles[21] = 45;
//     aprilTagAngles[22] = 45;

//     //Red Side
//     aprilTagAngles[6] = 45;
//     aprilTagAngles[7] = 45;
//     aprilTagAngles[8] = 45;
//     aprilTagAngles[9] = 45;
//     aprilTagAngles[10] = 45;
//     aprilTagAngles[11] = 45;

//     //Collection Station

//     //Blue Side
//     aprilTagAngles[13] = 45;
//     aprilTagAngles[12] = 45;

//     //Red Side
//     aprilTagAngles[1] = 45;
//     aprilTagAngles[2] = 45;


//     aprilTagDistance[6] = 8.75;
//     aprilTagDistance[7] = 8.75;
//     aprilTagDistance[8] = 8.75;
//     aprilTagDistance[9] = 8.75;
//     aprilTagDistance[10] = 8.75;
//     aprilTagDistance[11] = 8.75;


// }

// void VisionSubsystem::Periodic() {
//     frc::SmartDashboard::PutNumber("TX Left", getTXLeft());
//     frc::SmartDashboard::PutNumber("TX Right", getTXRight());
//     frc::SmartDashboard::PutNumber("TY Left", getTYLeft());
//     frc::SmartDashboard::PutNumber("TY Right", getTYRight());
// }

// void VisionSubsystem::putShuffleboard() {
    
// }

// double VisionSubsystem::getTX() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision").GetEntry("tx").GetDouble(0.0);
// }

// double VisionSubsystem::getTY() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision").GetEntry("ty").GetDouble(0.0);
// }

// int VisionSubsystem::getID() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision").GetEntry("fid").GetInteger(-1);
// }

// std::vector<double> VisionSubsystem::getPose() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision").GetEntry("botpose").GetDoubleArray(std::span<double>{});
// }

// double VisionSubsystem::getDistance(double targetHeight) {
//     double ty = getTY();
//     double mountAngle = 5;
//     double goalHeight = targetHeight;
//     double cameraHeight = 6.5;
    
//     double angleSum = ty + mountAngle;
//     double angleRadians = angleSum * (3.14159 / 180.0);
    
//     return (goalHeight - cameraHeight) / tan(angleRadians);
// }


// double VisionSubsystem::getTXLeft() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision/left").GetEntry("tx").GetDouble(0.0);
// }

// double VisionSubsystem::getTYLeft() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision/left").GetEntry("ty").GetDouble(0.0);
// }

// std::vector<double> VisionSubsystem::getPoseLeft() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision/left").GetEntry("botpose").GetDoubleArray(std::span<double>{});
// }


// double VisionSubsystem::getTXRight() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision/right").GetEntry("tx").GetDouble(0.0);
// }

// double VisionSubsystem::getTYRight() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision/right").GetEntry("ty").GetDouble(0.0);
// }

// std::vector<double> VisionSubsystem::getPoseRight() {
//     return nt::NetworkTableInstance::GetDefault().GetTable("photonvision/right").GetEntry("botpose").GetDoubleArray(std::span<double>{});
// }
