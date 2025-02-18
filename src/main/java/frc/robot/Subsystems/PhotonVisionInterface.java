package frc.robot.Subsystems;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionInterface extends SubsystemBase {

PhotonCamera camera = new PhotonCamera("VisionCamera");
//PhotonPipelineResult result = camera.getLatestResult();
//String test = "Test";

public Command testCommand() {
    return run(() -> {
        int ID = 0;
        var result = camera.getLatestResult();
        try {
            ID = result.getBestTarget().fiducialId;
        } catch (Exception e) {
            ID = 99;
        }  
        SmartDashboard.putNumber("FiducialID", ID);
    });
}

}

 