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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("VisionCamera");
    PhotonTrackedTarget target = new PhotonTrackedTarget();
    Rotation3d pRotation3d = new Rotation3d();
    Transform3d pCameraToRobot = new Transform3d(0.216, 0.220, 0.505, pRotation3d);  
    //Transform3d pCameraToRobot = new Transform3d();
    Pose3d robotPoseEstimate = new Pose3d();
    PhotonPipelineResult camResult = new PhotonPipelineResult();

    //List target IDs for red alliance
    int[] redTargetIDs = {1,2,6,7,8,9,10,11};

    //List target IDs for blue alliance
    int[] blueTargetIDs = {12,13,17,18,19,20,21,22};

// The field from AprilTagFields will be different depending on the game.
 public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
 
 //Processing steps in periodic - intent is to distribute processing steps across several scans
 private enum processStep {
    kGetResult,
    kGetTarget,
    kGetPose,
    kPublishResults
 }
 processStep step = processStep.kGetResult;

    @Override
    public void periodic(){
        if (camera.isConnected()) {
            switch(step) {
                case kGetResult:
                    if (getCameraResult()) {
                        step = processStep.kGetTarget;                        
                    }
                    break;

                case kGetTarget:
                    if (getTarget()) {
                        step = processStep.kGetPose;
                    } else {step = processStep.kGetResult;}
                    break;

                case kGetPose:
                    getPose3d();
                    step = processStep.kPublishResults;
                    break;

                case kPublishResults:
                    publishFiducialID();
                    publishYaw();
                    publishPitch();
                    publishArea();
                    publishSkew();
                    publishPoseX();
                    publishPoseY();
                    step = processStep.kGetResult;
                    break;
                }
        }
}

private boolean getCameraResult() {
    try {
        camResult = camera.getAllUnreadResults().get(0);
        return true;
    } catch (Exception e){
        return false;  
    }
}

public boolean hasTarget() {
    // Check if the latest result has any targets. 
    return camResult.hasTargets();    
}

private boolean getTarget() {
    if (hasTarget()) {
        target = camResult.getBestTarget();
        return true;
    } else {return false;}
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
        //yaw = target.getYaw();
        yaw = robotPoseEstimate.getRotation().getAngle();
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
    double poseX = 0;
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

private boolean isRedTarget(int tgt) {
    boolean tgtFound = false;
    for (int t : redTargetIDs) {
        if (t == tgt) {
            tgtFound =  true;
        }
    }
    return tgtFound;
}

private boolean isBlueTarget(int tgt) {
    boolean tgtFound = false;
    for (int t : blueTargetIDs) {
        if (t == tgt) {
            tgtFound =  true;
        }
    }
    return tgtFound;
}
 
//Bind as default command for photonvision
//Rumble controller if found target is on correct alliance side
public Command rumbleAtTarget(CommandXboxController ctrl) {
    return run(() -> {
        boolean rumble = false;
        //Check for target presence
        if (hasTarget()) {
            //check whether red or blue alliance
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                //Check if found target belongs to red side of field
                if (isRedTarget(target.getFiducialId())) {
                    rumble = true;
                }
            } else {
                //Check if found target belongs to blue side of field
                if (isBlueTarget(target.getFiducialId())) {
                    rumble = true;
                }
            }

        }
        //camera.setDriverMode(true);
        ctrl.setRumble(RumbleType.kBothRumble, (rumble ? 0.3 : 0.0));
    });
}
 
private void getPose3d() {
        // Calculate robot's field relative pose
    if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
       robotPoseEstimate = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), pCameraToRobot);
    }    
}






}

 