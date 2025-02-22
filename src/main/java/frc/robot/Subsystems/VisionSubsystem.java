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

// The field from AprilTagFields will be different depending on the game.
AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


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
        SmartDashboard.putNumber("FiducialID", ID);
    });

}

public Command getTargetInformation() {
    return run(() -> {
        // Get information from target.
        PhotonTrackedTarget target = camera.getAllUnreadResults().get(0).getBestTarget();
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();

        SmartDashboard.putNumber("Yaw", yaw);
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Area", area);
        SmartDashboard.putNumber("Skew", skew);

    });
}


}

 