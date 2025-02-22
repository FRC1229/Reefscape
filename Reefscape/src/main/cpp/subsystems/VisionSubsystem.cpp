#include "subsystems/VisionSubsystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "vector"


VisionSubsystem::VisionSubsystem() {


    //Coral Reef Tags and Angles
    //Blue side
    aprilTagAngles[17] = 45;
    aprilTagAngles[18] = 45;
    aprilTagAngles[19] = 45;
    aprilTagAngles[20] = 45;
    aprilTagAngles[21] = 45;
    aprilTagAngles[22] = 45;

    //Red Side
    aprilTagAngles[6] = 45;
    aprilTagAngles[7] = 45;
    aprilTagAngles[8] = 45;
    aprilTagAngles[9] = 45;
    aprilTagAngles[10] = 45;
    aprilTagAngles[11] = 45;

    //Collection Station

    //Blue Side
    aprilTagAngles[13] = 45;
    aprilTagAngles[12] = 45;

    //Red Side
    aprilTagAngles[1] = 45;
    aprilTagAngles[2] = 45;

}
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

int VisionSubsystem::getID(){
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("fid").GetInteger(-1);
}

std::vector<double> VisionSubsystem::getPose() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("botpose").GetDoubleArray(std::span<double>{});
}

double VisionSubsystem::getDistance(double targetHeight){
    double ty = VisionSubsystem::getTY();
    double mountAngle = 5;
    double goalHeight = targetHeight;
    double cameraHeight = 6.5;

    double angleSum = ty+mountAngle;

    double angleRadians = angleSum * (3.14159 / 180.0);


    return (goalHeight - cameraHeight)/tan(angleRadians);

}