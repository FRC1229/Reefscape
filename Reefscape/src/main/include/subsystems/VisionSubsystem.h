#pragma once

#include <frc2/command/SubsystemBase.h>

#include <vector>

class VisionSubsystem : public frc2::SubsystemBase {
    public:
    
        std::map<int, double> aprilTagAngles;

        VisionSubsystem();
        
        void Periodic() override;
        void putShuffleboard();
        double getTX();
        double getTY();
        int getID();
        double getDistance(double targetHeight);
        std::vector<double> getPose();
};