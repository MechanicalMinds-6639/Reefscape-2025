// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GrabberConstants;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Grabber extends SubsystemBase {
  //Members
  private final SparkMax GrabberMax = new SparkMax(GrabberConstants.GRABBER_LEFT_ID,MotorType.kBrushless);
  private final SparkMax GrabberTwistMax = new SparkMax(GrabberConstants.GRABBER_TWIST_ID, MotorType.kBrushless);
  private final SparkClosedLoopController GrabberTwistController = GrabberMax.getClosedLoopController();

  /** Creates a new Grabber. */
  public Grabber() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg
        .smartCurrentLimit(GrabberConstants.WRIST_CURRENT_LIMIT) 
        .closedLoopRampRate(GrabberConstants.WRIST_CLOSED_LOOP_RAMP_RATE) 
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.01, 0, 0) // Change meee
        .outputRange(-1, 1);
//        .maxMotion
//        .maxVelocity(GrabberConstants.CORAL_ARM_MAX_VELOCITY_RPM) // Change me
//        .maxAcceleration(GrabberConstants.kCoralArmMaxAccelerationRPMperSecond) // Change me
//        .allowedClosedLoopError(GrabberConstants.kCoralArmAllowedClosedLoopError.in(Rotations)); // Change me
    GrabberTwistMax.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setWristAngle(double angle)
  {
    return run(() -> {
      GrabberTwistController.setReference(Degrees.of(angle).in(Rotations), ControlType.kPosition);
    });
  }



  //Grabber Command Factory
  public Command GrabberConverterCommand(CommandXboxController CraneController){ //double ElevatorSpeed, double ArmSpeed, double TwistSpeed, double GrabberSpeed

    return run(() -> {

      double TwistSpeed = CraneController.getRightX();
      
      // Twister speed control
      GrabberTwistMax.set(TwistSpeed * GrabberConstants.TWIST_MULTIPLIER);

      if (CraneController.leftBumper().getAsBoolean()){
        GrabberMax.set(GrabberConstants.GRABBER_SPEED);
      } else if (CraneController.rightBumper().getAsBoolean()){
        GrabberMax.set(-GrabberConstants.GRABBER_SPEED);
      } else {
        GrabberMax.set(0);
      }

    });
    
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
