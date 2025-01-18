#include "subsystems/VisionSubsystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "vector"


VisionSubsystem::VisionSubsystem() {}
void VisionSubsystem::Periodic(){
    frc::SmartDashboard::PutNumber("TX", VisionSubsystem::getTX());
}
void VisionSubsystem::putShuffleboard() {
    //frc::SmartDashboard::PutNumber("TX", VisionSubsystem::getTX());
}

double VisionSubsystem::getTX() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("tx").GetDouble(0.0);
}

double VisionSubsystem::getTY() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("ty").GetDouble(0.0);
}

std::vector<double> VisionSubsystem::getPose() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("botpose").GetDoubleArray(std::span<double>{});
}

double VisionSubsystem::getDistance(double targetHeight){
    double ty = VisionSubsystem::getTY();
    double mountAngle = 20;
    double goalHeight = targetHeight;
    double cameraHeight = 16.35;

    double angleSum = ty+mountAngle;

    double angleRadians = angleSum * (3.14159 / 180.0);


    return (goalHeight - cameraHeight)/tan(angleRadians);

}