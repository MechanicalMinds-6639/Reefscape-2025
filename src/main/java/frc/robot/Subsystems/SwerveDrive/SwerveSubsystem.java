// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive;
import frc.robot.Subsystems.SwerveDrive.SwerveModule;


import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveDrive swerveDrive;
  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  private final CommandXboxController m_Controller;

    
  
    // private static final AnalogInput frontLeftEncoder = new AnalogInput(Constants.Drive.FL_ABS_ENCODER_ID);
    // private static final AnalogInput frontRightEncoder = new AnalogInput(Constants.Drive.FR_ABS_ENCODER_ID);
    // private static final AnalogInput backLeftEncoder = new AnalogInput(Constants.Drive.BL_ABS_ENCODER_ID);
    // private static final AnalogInput backRightEncoder = new AnalogInput(Constants.Drive.BR_ABS_ENCODER_ID);
  
    public SwerveSubsystem(File directory, CommandXboxController driverController) {
  
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; //TODO LOWER THIS AT COMP, SLOWS COMPUTATION
  
      try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Drive.MAX_SPEED);
      } catch (Exception e) {
        throw new RuntimeException(e); //In case it cannot find the JSON files in the deploy directory
      }
  
      swerveDrive.setHeadingCorrection(false); // true if using setpoint heading, false if using angular velocity, maybe tie to a sendable chooser or constant
      swerveDrive.setCosineCompensator(false);
      swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
      swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
  
      frontLeftModule = new SwerveModule(6, 7, 2, 248.327);
      frontRightModule = new SwerveModule(4, 5, 3, 138.754);
      backLeftModule = new SwerveModule(2, 1, 0, 214.287);
      backRightModule = new SwerveModule(9, 8, 1, 241.974);
  
      m_Controller = driverController;


      setupPathPlanner();
    }
  
    @Override
    public void periodic() { //The dashboard numbers should no longer be needed
      // This method will be called once per scheduler run
      // SmartDashboard.putNumber("Front left encoder", swerveDrive.getModules()[0].getAbsolutePosition());
      // SmartDashboard.putNumber("Front right encoder", swerveDrive.getModules()[1].getAbsolutePosition());
      // SmartDashboard.putNumber("Back left encoder", swerveDrive.getModules()[2].getAbsolutePosition());
      // SmartDashboard.putNumber("Back right encoder", swerveDrive.getModules()[3].getAbsolutePosition());
    }
  
    public void setupPathPlanner() {
  
      RobotConfig config;
  
      try {
  
        config = RobotConfig.fromGUISettings(); //TODO maybe make this a constant built manually
  
        final boolean enableFeedForward = true;
  
        AutoBuilder.configure(
  
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
  
          (speedsRobotRelative, moduleFeedForwards) -> {
            if(enableFeedForward) {
              swerveDrive.drive(
                speedsRobotRelative,
                swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                moduleFeedForwards.linearForces()
                              );
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
  
          new PPHolonomicDriveController( 
            //TODO Set proper pid values maybe
            new PIDConstants(0.0020645, 0.0, 0.0),
            //translation
            new PIDConstants(0.01, 0.0, 0.0)
            //rotation
          ),
          config,
  
          () -> {
  
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
  
        );

        setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getRobotVelocity(); // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getModuleStates(); // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
        
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    /**
     * This method will take in desired robot-relative chassis speeds,
     * generate a swerve setpoint, then set the target state for each module
     *
     * @param speeds The desired robot-relative speeds
     */
     
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );
        setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
      
    PathfindingCommand.warmupCommand().schedule();
  }
  //End Auto Stuff


  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(
        new Config(), 
        this, swerveDrive, 12, true),
      3.0, 5.0, 3.0);
  }

  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
        new Config(),
        this, swerveDrive),
      3.0, 5.0, 3.0);
  }

  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  /*
   * Command to drive the robot using translative values and angular velocity
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    
    return run(() -> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8), //Unsure of where 0.8 comes from, subject to change
      Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
      true, false);
    });
  }

  /*
   * Command to drive the robot using translative values and heading as a setpoint
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {

    //Make sure swerveDrive.setHeadingCorrection(true) is true when using this command
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), 
        translationY.getAsDouble()), 0.8); //Unsure of where 0.8 comes from, subject to change
      
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), 
        headingX.getAsDouble(), headingY.getAsDouble(),
        swerveDrive.getOdometryHeading().getRadians(),
        swerveDrive.getMaximumChassisVelocity()));
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.setMaximumAttainableSpeeds(swerveDrive.getMaximumChassisVelocity(), 
                                            swerveDrive.getMaximumChassisAngularVelocity());
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    };
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    double maxSpeed = 1.5; //Incase it cannot be updated

    if (m_Controller.getRightTriggerAxis() > 0.75) {
      maxSpeed = 3;
    } else {
      maxSpeed = 1.5;
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeed);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  // Misc. utility methods ---------------------------

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
    System.out.println("Gyroscope YAW zeroed");
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }
}

//Vision Commands

//public Command aimAtTarget()
