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
    targetPoses[6] = frc::Pose2d(13.69_m, 2.93_m, frc::Rotation2d(120_deg));
    targetPoses[7] = frc::Pose2d(14.33_m, 4.03_m, frc::Rotation2d(180_deg));
    targetPoses[8] = frc::Pose2d(13.69_m, 5.13_m, frc::Rotation2d(240_deg));
    targetPoses[9] = frc::Pose2d(12.42_m, 5.13_m, frc::Rotation2d(300_deg));
    targetPoses[10] = frc::Pose2d(11.79_m, 4.03_m, frc::Rotation2d(0_deg));
    targetPoses[11] = frc::Pose2d(12.42_m, 2.93_m, frc::Rotation2d(60_deg));

    //Blue Side
    targetPoses[12] = frc::Pose2d(1.11_m, 1.02_m, frc::Rotation2d(234_deg));
    targetPoses[13] = frc::Pose2d(1.11_m, 7.04_m, frc::Rotation2d(126_deg));
    targetPoses[16] = frc::Pose2d(5.99_m, 0.44_m, frc::Rotation2d(270_deg));
    targetPoses[17] = frc::Pose2d(3.85_m, 2.906_m, frc::Rotation2d(60_deg));
    targetPoses[18] = frc::Pose2d(3.22_m, 4.03_m, frc::Rotation2d(0_deg));
    targetPoses[19] = frc::Pose2d(3.85_m, 5.13_m, frc::Rotation2d(300_deg));
    targetPoses[20] = frc::Pose2d(5.12_m, 5.13_m, frc::Rotation2d(240_deg));
    targetPoses[21] = frc::Pose2d(5.76_m, 4.03_m, frc::Rotation2d(180_deg));
    targetPoses[22] = frc::Pose2d(5.12_m, 2.93_m, frc::Rotation2d(120_deg));//3.79, 2.81
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

Camera::Camera(std::string c, frc::Transform2d t): 
cam{c}, 
cameraToRobot{t}
{}


frc::Pose2d VisionSubsystem::GetUpdatePose(){
    // std::optional<photon::EstimatedRobotPose> estimatedPose = poseEstimator->Update(getResultLeft());
    // if(estimatedPose.has_value()){
    //     return estimatedPose.value().estimatedPose.ToPose2d();
    // }
}
photon::PhotonPipelineResult Camera::getResult(){
    if(cam.GetLatestResult().HasTargets()){
        return cam.GetLatestResult();
    }
}


photon::PhotonTrackedTarget Camera::ClosestTarget(){
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

photon::PhotonTrackedTarget Camera::BestResult(){
    if(getResult().HasTargets()){
        return getResult().GetBestTarget();
    }
}

bool Camera::seeTarget(){
    return getResult().HasTargets();
}
double Camera::getYaw() {
    if(getResult().HasTargets()){

        return BestResult().GetYaw();
    }
}

double Camera::getTY() {
    if(getResult().HasTargets()){

        return BestResult().GetPitch();
    }
}

double Camera::getYmeters() {
    if(getResult().HasTargets()){
        return BestResult().GetBestCameraToTarget().Y().value();
    }

}

frc::Pose2d Camera::getCameraRobotPose(){
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


std::vector<frc::Pose2d> Camera::getCameraRobotPoses(){
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


double Camera::getXmeters() {
    if(getResult().HasTargets()){
        return BestResult().GetBestCameraToTarget().X().value();
    }

}

double Camera::getZAngle(){
    if(getResult().HasTargets()){
        // return BestResult().
    }
}

int Camera::getID() {
    if(getResult().HasTargets()){
        return BestResult().GetFiducialId();
    }
}


// double VisionSubsystem::getDistance(double targetHeight) {
//     double ty = getTY();
//     double mountAngle = 5;
//     double goalHeight = targetHeight;
//     double cameraHeight = 8.6;
    
//     double angleSum = ty + mountAngle;
//     double angleRadians = angleSum * (3.14159 / 180.0);
    
//     return (goalHeight - cameraHeight) / tan(angleRadians);
// }


