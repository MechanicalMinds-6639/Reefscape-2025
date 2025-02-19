package frc.robot.Subsystems;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

PhotonCamera camera = new PhotonCamera("VisionCamera");


//Gets ID of best target Apriltag and displays 
public Command getFiducialID() {
    return run(() -> {
        //System.out.println("Command Running");
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

 