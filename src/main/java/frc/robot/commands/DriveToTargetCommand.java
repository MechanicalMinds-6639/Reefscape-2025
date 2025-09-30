package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.SwerveDrive.SwerveSubsystem;

public class DriveToTargetCommand extends Command {

    private VisionSubsystem visionSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private int prefferedTargetID = -1;
    private PhotonTrackedTarget target = null;

    public DriveToTargetCommand(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem,
            int prefferedTargetID) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.prefferedTargetID = prefferedTargetID;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        try {
            target = visionSubsystem.getPerfferedTarget(prefferedTargetID);
            if (target == null)
                end(true);

            Translation2d translationToTarget = visionSubsystem.getTranslationToTarget(target);
            System.out.println(translationToTarget);
            swerveSubsystem.drive(translationToTarget, 0, false);
        } catch (RuntimeException e) {
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveToTarget ended.");
        swerveSubsystem.drive(new Translation2d(), 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
