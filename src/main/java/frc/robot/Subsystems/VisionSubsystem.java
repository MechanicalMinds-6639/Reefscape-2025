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

    PhotonPipelineResult camResult = new PhotonPipelineResult();

// The field from AprilTagFields will be different depending on the game.
 public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    /* 
    @Override
    public void periodic(){
        getCameraResult();
        getTarget();
        publishYaw();
        publishFiducialID();
        publishPitch();
        publishArea();
        publishSkew();
        publishPoseX();
        publishPoseY();        
    }
*/
private void getCameraResult() {
    try {
        camResult = camera.getAllUnreadResults().get(0);
    } catch (Exception e){
        System.out.println("AAAAAAAAAAAAAAAAAAAAAAA");  
    }
}  
 
public boolean hasTarget() {
    // Check if the latest result has any targets.
    return camResult.hasTargets();    
}

private void getTarget() {
    if (hasTarget()) {
        target = camResult.getBestTarget();
    }
}

public double yawToTarget(){
    try {
        return target.getYaw();
    } catch (Exception e) {
        return 0;
    }  
}





//Gets ID of best target Apriltag and displays
public void publishFiducialID() {
    double FiducialID = 0;
    try {
        FiducialID = target.getFiducialId();
    } catch (Exception e) {
        FiducialID = -1;
    }  
    SmartDashboard.putNumber("FiducialID", FiducialID);
}
//Gets Yaw of best target Apriltag and displays
public void publishYaw() {
    double yaw = 0;
    try {
        yaw = target.getYaw();
    } catch (Exception e) {
        yaw = -1;
    }  
    SmartDashboard.putNumber("Yawwwwl", yaw);
}

//Gets Pitch of best target Apriltag and displays
public void publishPitch() {
    double pitch = 0;
    try {
        pitch = target.getPitch();
    } catch (Exception e) {
        pitch = -1;
    }  
     SmartDashboard.putNumber("Pitch", pitch);
}

//Gets Area of best target Apriltag and displays
public void publishArea() {
    double area = 0;
    try {
        area = target.getArea();
    } catch (Exception e) {
        area = -1;
    }  
     SmartDashboard.putNumber("Area", area);
}

//Gets Skew of best target Apriltag and displays
public void publishSkew() {
    double skew = 0;
    try {
        skew = target.getSkew();
    } catch (Exception e) {
        skew = -1;
    }  
    SmartDashboard.putNumber("Skew", skew);
}

//Gets PoseX of best target Apriltag and displays
public void publishPoseX() {
    double poseX;
    try {
        poseX = robotPoseEstimate.getX();
    } catch (Exception e) {
        poseX = -1;
    }  
    SmartDashboard.putNumber("PoseX", poseX);
}

//Gets PoseY of best target Apriltag and displays
public void publishPoseY() {
    double poseY = 0;
    try {
        poseY = robotPoseEstimate.getY();
    } catch (Exception e) {
        poseY = -1;
    }  
    SmartDashboard.putNumber("PoseY", poseY);
}



public Command getAllUnreadResults() {
    return run(() -> {
        // Get information from target.
        try {
            target = camera.getAllUnreadResults().get(0).getBestTarget();
            if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                robotPoseEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), pCameraToRobot);
             }
        } catch (Exception e) {
            System.out.println("AAAAAAAAAAAAAAAAAAAAAAA");
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

 