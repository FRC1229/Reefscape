#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <vector>
#include <map>

class Camera{

    public:

        frc::AprilTagFieldLayout layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025ReefscapeAndyMark);

        // frc::Transform2d cameraToRobot = frc::Transform2d{
        //     units::meter_t{-0.27},
        //     units::meter_t{0},
        //     frc::Rotation2d(units::degree_t{0})
        // };

        frc::Transform2d cameraToRobot;
        photon::PhotonCamera cam;
        Camera(std::string c, frc::Transform2d cb);

        photon::PhotonPipelineResult getResult();
        photon::PhotonTrackedTarget BestResult();
        photon::PhotonTrackedTarget ClosestTarget();

        bool seeTarget();
        double getYaw();
        double getTY();
        double getYmeters();
        double getXmeters();
        double getZAngle();
        frc::Pose2d getCameraRobotPose();
        std::vector<frc::Pose2d> getCameraRobotPoses();

        int getID();
        // double getDistance(double targetHeight);
        // double getYawfromPose();
        

};

class VisionSubsystem : public frc2::SubsystemBase {
    public:

        VisionSubsystem();

        Camera camera_Right{
            "1229_Camera",
        
            frc::Transform2d{
                units::meter_t{0.03},
                units::meter_t{0.52},
                frc::Rotation2d(units::degree_t{-35})

        }
        
        };
        Camera camera_Left{
            "1229_Camera_Left",

            frc::Transform2d{
                units::meter_t{-0.27},
                units::meter_t{-0.24},
                frc::Rotation2d(units::degree_t{35})
        }
        };
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

        
        // photon::PhotonPipelineResult getResult();
        // photon::PhotonTrackedTarget BestResult();
        // photon::PhotonTrackedTarget ClosestTarget();

        frc::AprilTagFieldLayout layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025ReefscapeAndyMark);

        // frc::Transform2d cameraToRobot = frc::Transform2d{
        //     units::meter_t{-0.27},
        //     units::meter_t{0},
        //     frc::Rotation2d(units::degree_t{0})
        // };

        frc::Transform2d cameraToRobot = frc::Transform2d{
            units::meter_t{-0.25},
            units::meter_t{-0.32},
            frc::Rotation2d(units::degree_t{-35})
        };


        // bool seeTarget();
        // double getYaw();
        // double getTY();
        // double getYmeters();
        // double getXmeters();
        // double getZAngle();
        // frc::Pose2d getCameraRobotPose();
        // std::vector<frc::Pose2d> getCameraRobotPoses();

        // int getID();
        // double getDistance(double targetHeight);
        // double getYawfromPose();
     
        
      
       
};

