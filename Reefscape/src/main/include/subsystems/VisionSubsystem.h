#pragma once

#include <frc2/command/SubsystemBase.h>

#include <vector>

class VisionSubsystem : public frc2::SubsystemBase {
    public:
        VisionSubsystem();
        
        void Periodic() override;
        void putShuffleboard();
        double getTX();
        double getTY();
        double getDistance(double targetHeight);
        std::vector<double> getPose();
};