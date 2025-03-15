#include "subsystems/VisionSubsystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>

VisionSubsystem::VisionSubsystem() {

    targetPoses[18] = frc::Pose2d(3.19_m, 4.03_m, frc::Rotation2d(0_deg));
    targetPoses[17] = frc::Pose2d(3.963_m, 2.906_m, frc::Rotation2d(60_deg)); //3.79, 2.81
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
    return camera.GetLatestResult();
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


