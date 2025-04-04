#include "subsystems/VisionSubsystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <span>
#include <vector>

VisionSubsystem::VisionSubsystem() {
    //Red Side 
    targetPoses[1] = frc::Pose2d(16.44_m, 1.02_m, frc::Rotation2d(306_deg));
    targetPoses[2] = frc::Pose2d(16.44_m, 7.04_m, frc::Rotation2d(54_deg));
    targetPoses[3] = frc::Pose2d(11.56_m, 7.62_m, frc::Rotation2d(90_deg));
    targetPoses[6] = frc::Pose2d(13.77_m, 3.01_m, frc::Rotation2d(120_deg));
    targetPoses[7] = frc::Pose2d(14.33_m, 4.07_m, frc::Rotation2d(180_deg));
    targetPoses[8] = frc::Pose2d(13.65_m, 5.17_m, frc::Rotation2d(240_deg));
    targetPoses[9] = frc::Pose2d(12.42_m, 5.13_m, frc::Rotation2d(300_deg));
    targetPoses[10] = frc::Pose2d(11.79_m, 3.97_m, frc::Rotation2d(0_deg));
    targetPoses[11] = frc::Pose2d(12.46_m, 2.89_m, frc::Rotation2d(60_deg));
    targetPoses[100] = frc::Pose2d(9.81_m,0_m,frc::Rotation2d(0_deg));
    targetPoses[101] = frc::Pose2d(9.61_m,0_m,frc::Rotation2d(0_deg));


    //Blue Side
    targetPoses[12] = frc::Pose2d(1.11_m, 1.02_m, frc::Rotation2d(234_deg));
    targetPoses[13] = frc::Pose2d(1.11_m, 7.04_m, frc::Rotation2d(126_deg));
    targetPoses[16] = frc::Pose2d(6.03_m, 0.40_m, frc::Rotation2d(270_deg));
    targetPoses[17] = frc::Pose2d(3.85_m, 2.906_m, frc::Rotation2d(60_deg));
    targetPoses[18] = frc::Pose2d(3.22_m, 3.99_m, frc::Rotation2d(0_deg));
    targetPoses[19] = frc::Pose2d(3.81_m, 5.09_m, frc::Rotation2d(300_deg));
    targetPoses[20] = frc::Pose2d(5.08_m, 5.17_m, frc::Rotation2d(240_deg));
    targetPoses[21] = frc::Pose2d(5.76_m, 4.07_m, frc::Rotation2d(180_deg));
    targetPoses[22] = frc::Pose2d(5.18_m, 2.99_m, frc::Rotation2d(120_deg));//3.79, 2.81
    // targetPoses[22] = frc::Pose2d(3.79_m, 2.81_m, frc::Rotation2d(60_deg));
    // targetPoses[16] = frc::Pose2d()

   
// RobotToCamera =
//     frc::Transform3d(frc::Translation3d(0.5_m, 0_m, 0.5_m),
//                     frc::Rotation3d(0_rad, 0_rad, 0_rad)),

//     poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(
//     tagLayout, photon::CLOSEST_TO_REFERENCE_POSE, camera,RobotToCamera);

}

void VisionSubsystem::Periodic() {
   
}

frc::Pose2d VisionSubsystem::GetUpdatePose(){
    // std::optional<photon::EstimatedRobotPose> estimatedPose = poseEstimator->Update(getResult());
    // if(estimatedPose.has_value()){
    //     return estimatedPose.value().estimatedPose.ToPose2d();
    // }
}
photon::PhotonPipelineResult VisionSubsystem::getResult(){
    if(camera.GetLatestResult().HasTargets()){
        return camera.GetLatestResult();
    }
}

photon::PhotonTrackedTarget VisionSubsystem::ClosestTarget(){
    if(getResult().HasTargets()){
        std::span<const photon::PhotonTrackedTarget> targets = getResult().GetTargets();
        photon::PhotonTrackedTarget closestTrag = targets[0];
        for(photon::PhotonTrackedTarget targ : targets){
            if(targ.GetBestCameraToTarget().X().value() < closestTrag.GetBestCameraToTarget().X().value()){
                closestTrag = targ;
            }
        }
        return closestTrag;
    }
}

photon::PhotonTrackedTarget VisionSubsystem::BestResult(){
    if(getResult().HasTargets()){
        return getResult().GetBestTarget();
    }
}

bool VisionSubsystem::seeTarget(){
    return getResult().HasTargets();
}
double VisionSubsystem::getYaw() {
    if(getResult().HasTargets()){

        return BestResult().GetYaw();
    }
}

double VisionSubsystem::getTY() {
    if(getResult().HasTargets()){

        return BestResult().GetPitch();
    }
}

double VisionSubsystem::getYmeters() {
    if(getResult().HasTargets()){
        return BestResult().GetBestCameraToTarget().Y().value();
    }

}

frc::Pose2d VisionSubsystem::getCameraRobotPose(){
    if(getResult().HasTargets()){

        auto target = BestResult();
        frc::Transform3d best_camera_to_target = target.GetBestCameraToTarget();

        int tagId = target.GetFiducialId();

        frc::Transform2d best_camera_to_target_2D = frc::Transform2d{
            best_camera_to_target.X(),
            best_camera_to_target.Y(),
            best_camera_to_target.Rotation().ToRotation2d()
        };

        std::optional<frc::Pose3d> field_to_target = layout.GetTagPose(tagId);


        if (!field_to_target.has_value()){
            return frc::Pose2d();
        }

        frc::Pose2d field_to_tag_2d = field_to_target.value().ToPose2d();
        
        return photon::PhotonUtils::EstimateFieldToRobot(
            best_camera_to_target_2D,
            field_to_tag_2d,
            cameraToRobot
        );  
        
    }
}


std::vector<frc::Pose2d> VisionSubsystem::getCameraRobotPoses(){
    std::vector<frc::Pose2d> allPoses;
    if(getResult().HasTargets()){
        for(auto target : getResult().GetTargets()){

            
            frc::Transform3d best_camera_to_target = target.GetBestCameraToTarget();

            int tagId = target.GetFiducialId();

            frc::Transform2d best_camera_to_target_2D = frc::Transform2d{
                best_camera_to_target.X(),
                best_camera_to_target.Y(),
                best_camera_to_target.Rotation().ToRotation2d()
            };

            std::optional<frc::Pose3d> field_to_target = layout.GetTagPose(tagId);


            if (!field_to_target.has_value()){
                allPoses.push_back(frc::Pose2d());
            }

            frc::Pose2d field_to_tag_2d = field_to_target.value().ToPose2d();
            
            allPoses.push_back(photon::PhotonUtils::EstimateFieldToRobot(
                best_camera_to_target_2D,
                field_to_tag_2d,
                cameraToRobot
            )); 

            return allPoses;
        }
    }
}


double VisionSubsystem::getXmeters() {
    if(getResult().HasTargets()){
        return BestResult().GetBestCameraToTarget().X().value();
    }

}

double VisionSubsystem::getZAngle(){
    if(getResult().HasTargets()){
        // return BestResult().
    }
}

int VisionSubsystem::getID() {
    if(getResult().HasTargets()){
        return BestResult().GetFiducialId();
    }
}


double VisionSubsystem::getDistance(double targetHeight) {
    double ty = getTY();
    double mountAngle = 5;
    double goalHeight = targetHeight;
    double cameraHeight = 8.6;
    
    double angleSum = ty + mountAngle;
    double angleRadians = angleSum * (3.14159 / 180.0);
    
    return (goalHeight - cameraHeight) / tan(angleRadians);
}


