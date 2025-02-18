package frc.robot.Subsystems;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionInterface extends SubsystemBase {

PhotonCamera camera = new PhotonCamera("VisionCamera");
//PhotonPipelineResult result = camera.getLatestResult();
//String test = "Test";

public Command testCommand() {
    return run(() -> System.out.println(camera.getName()));
}

}
