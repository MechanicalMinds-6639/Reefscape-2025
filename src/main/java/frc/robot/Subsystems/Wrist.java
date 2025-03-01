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
import frc.robot.Constants.Operator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Wrist extends SubsystemBase {
  //Members
  private final SparkMax GrabberTwistMax = new SparkMax(GrabberConstants.GRABBER_TWIST_ID, MotorType.kBrushless);
  private final SparkClosedLoopController GrabberTwistController = GrabberTwistMax.getClosedLoopController();
  private final RelativeEncoder WristEncoder = GrabberTwistMax.getEncoder();

  /** Creates a new Grabber. */
  public Wrist() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg
        .smartCurrentLimit(GrabberConstants.WRIST_CURRENT_LIMIT)  
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    GrabberTwistMax.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setZeroReferncePoint(){
      WristEncoder.setPosition(0);
  }

  public double getAngleInDegrees(){
    return (WristEncoder.getPosition()/10.0*360.0)%360.0;
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
      System.out.println(getAngleInDegrees());
      if (CraneController.start().getAsBoolean()){
        setZeroReferncePoint();
      }
      
      double TwistSpeed = CraneController.getRightX();
      
      // Twister speed control
      if (Math.abs(TwistSpeed) < Operator.DEADBAND) {
        GrabberTwistMax.set(0.0);
      } else {
        GrabberTwistMax.set(TwistSpeed * GrabberConstants.TWIST_MULTIPLIER);
      }
    });
    
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}