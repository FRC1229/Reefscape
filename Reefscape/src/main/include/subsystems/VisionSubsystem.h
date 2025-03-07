#pragma once

#include <frc2/command/SubsystemBase.h>
#include <vector>
#include <map>

class VisionSubsystem : public frc2::SubsystemBase {
    public:
    
        std::map<int, double> aprilTagAngles;
        std::map<int,double> aprilTagDistance;

        VisionSubsystem();
        
        void Periodic() override;
        void putShuffleboard();
        
        
        double getTX();
        double getTY();
        int getID();
        double getDistance(double targetHeight);
        std::vector<double> getPose();
        
      
        double getTXLeft();
        double getTXRight();
        double getTYLeft();
        double getTYRight();
        std::vector<double> getPoseLeft();
        std::vector<double> getPoseRight();
};

