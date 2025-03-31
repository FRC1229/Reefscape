#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <vector>
#include <map>

class VisionSubsystem : public frc2::SubsystemBase {
    public:

        VisionSubsystem();

        photon::PhotonCamera camera{"1229_Camera"};
        //photon::PhotonCamera cameraRight{"1229_Camera"};
        std::unique_ptr<photon::PhotonPoseEstimator> poseEstimator;
        frc::Transform3d RobotToCamera;
        frc::Pose2d currentPose;
        frc::AprilTagFieldLayout tagLayout;
        int lastTag;


        void Periodic() override;
        void putShuffleboard();
        frc::Pose2d getPose();
        std::map<int, frc::Pose2d> targetPoses;

        frc::Pose2d GetUpdatePose();

        
        photon::PhotonPipelineResult getResult();
        photon::PhotonTrackedTarget BestResult();
        photon::PhotonTrackedTarget ClosestTarget();
        frc::AprilTagFieldLayout layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025ReefscapeAndyMark);

        frc::Transform2d cameraToRobot = frc::Transform2d{
            units::meter_t{-0.36}, //-27
            units::meter_t{0},
            frc::Rotation2d(units::degree_t{0})
        };


        bool seeTarget();
        double getYaw();
        double getTY();
        double getYmeters();
        double getXmeters();
        double getZAngle();
        frc::Pose2d getCameraRobotPose();
        std::vector<frc::Pose2d> getCameraRobotPoses();

        int getID();
        double getDistance(double targetHeight);
        double getYawfromPose();
     
        
      
       
};
