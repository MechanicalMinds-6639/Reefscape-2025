

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you ?
// can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(Constants.Operator.DRIVER);
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final VisionSubsystem photonVision = new VisionSubsystem();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1)
    .withControllerRotationAxis(driverController::getRightX)
    .deadband(Constants.Operator.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
    driverController::getRightX,
    driverController::getRightY)
    .headingWhile(true);
  
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
    .allianceRelativeControl(false);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true); // YOU CAN DO THIS?>>???????
  }

  private void configureBindings() {

    //System.out.println("AJHKSDASHKASJHDHSAKJDHJKADHKJS");

    Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = driveBase.driveFieldOriented(driveRobotOriented);

    if (Robot.isSimulation()) {
      driverController.start().onTrue(Commands.runOnce(() -> driveBase.resetOdometry(new Pose2d(3,3,new Rotation2d()))));
      driverController.button(1).whileTrue(driveBase.sysIdDriveMotorCommand());
    }

    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity); //Change to switch the drive control style, make sure to set heading correction to true in SwerveSubsystem
    driverController.a().whileTrue(driveBase.centerModulesCommand());
    driverController.x().onTrue(Commands.runOnce(driveBase::zeroGyro));
    //driverController.y().onTrue((driveBase.aimAtTarget(photonVision.yawToTarget())));

    //photonVision.setDefaultCommand(photonVision.getAllUnreadResults());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setMotorBrake(boolean brake) {
    driveBase.setMotorBrake(brake);
  }
}
