// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Grabber;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(Constants.Operator.DRIVER);
  private final CommandXboxController copilotController = new CommandXboxController(Constants.Operator.COPILOT);
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final Wrist KCCWrist = new Wrist();
  private final Grabber KCCGrabber = new Grabber();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();

  private final SendableChooser<Command> autoChooser;

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
    //
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true); // YOU CAN DO THIS?>>???????

    NamedCommands.registerCommand("Dumb Wrist Command", KCCWrist.autoWristCommand(90)
      .withTimeout(2));
    NamedCommands.registerCommand("Dumb Arm Command", arm.autoArmCommand(2.75)
      .withTimeout(3));
    NamedCommands.registerCommand("Auto Spit Command", KCCGrabber.Grab(-1)
      .withTimeout(1.5).andThen(KCCGrabber.Grab(0)));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData(autoChooser);
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
    elevator.setDefaultCommand(elevator.ElevatorConverterCommand(copilotController));
    KCCWrist.setDefaultCommand(KCCWrist.GrabberConverterCommand(copilotController));
    climber.setDefaultCommand(climber.climberDefaultCommand(driverController));
    //driverController.y().whileTrue(elevator.setElevatorHeight(.4)).onFalse(elevator.stop());
    arm.setDefaultCommand(arm.ArmConverterCommand(copilotController));
    driverController.a().whileTrue(driveBase.centerModulesCommand());
    driverController.x().onTrue(Commands.runOnce(driveBase::zeroGyro));
    copilotController.leftBumper().whileTrue(KCCGrabber.Grab(1)).onFalse(KCCGrabber.Grab(0));
    copilotController.rightBumper().whileTrue(KCCGrabber.Grab(-1)).onFalse(KCCGrabber.Grab(0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void resetGyro(){
    Commands.runOnce(driveBase::zeroGyro);
  }

  public void resetWrist() {
    Commands.runOnce(KCCWrist::setZeroReferncePoint);
  }

  public void resetArm() {
    Commands.runOnce(arm::setZeroReferncePoint);
  }

  public void setMotorBrake(boolean brake) {
    driveBase.setMotorBrake(brake);
  }
}
