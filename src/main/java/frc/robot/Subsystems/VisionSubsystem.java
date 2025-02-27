package frc.robot.Subsystems;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("VisionCamera");

    PhotonTrackedTarget target = new PhotonTrackedTarget();

    Rotation3d pRotation3d = new Rotation3d();

    Transform3d pCameraToRobot = new Transform3d(0.0, 0.0, 0.31, pRotation3d);
    
    Pose3d robotPoseEstimate = new Pose3d();

// The field from AprilTagFields will be different depending on the game.
AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Yawwwwl", target.getYaw());
        SmartDashboard.putNumber("Pitch", target.getPitch());
        SmartDashboard.putNumber("Area", target.getArea());
        SmartDashboard.putNumber("Skew", target.getSkew());
        SmartDashboard.putNumber("FiducialID", target.getFiducialId());
        SmartDashboard.putNumber("PoseX", robotPoseEstimate.getX());
        SmartDashboard.putNumber("PoseY", robotPoseEstimate.getY());
    }

//Gets ID of best target Apriltag and displays 
public Command getFiducialID() {
    return run(() -> {
        //System.out.println("Command Running");
        int ID = 0;
        var result = camera.getLatestResult();
        try {
            ID = result.getBestTarget().fiducialId;
        } catch (Exception e) {
            ID = -1;
        }  
       
    });

}



public Command getAllUnreadResults() {
    return run(() -> {
        // Get information from target.
        target = camera.getAllUnreadResults().get(0).getBestTarget();
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            robotPoseEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), pCameraToRobot);
         }
    });}

 
public Command getPose3d() {
    return run(() -> {
        // Calculate robot's field relative pose
    if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
       robotPoseEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), pCameraToRobot);
    }
    
    });}


}

 