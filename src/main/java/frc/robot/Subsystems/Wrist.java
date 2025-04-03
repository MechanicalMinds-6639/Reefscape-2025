// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;
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
  private double SetPointAngle = 0.0;

  /** Creates a new Grabber. */
  public Wrist() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg
        .smartCurrentLimit(GrabberConstants.WRIST_CURRENT_LIMIT)  
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    cfg.closedLoop.p(GrabberConstants.WRIST_KP)
                  .i(GrabberConstants.WRIST_KI)
                  .d(GrabberConstants.WRIST_KD)
                  .outputRange(-GrabberConstants.WRIST_MAX_OUTPUT, GrabberConstants.WRIST_MAX_OUTPUT);
    GrabberTwistMax.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setZeroReferncePoint(){
      WristEncoder.setPosition(0);
  }

  public double getAngleInDegrees(){
    return Math.abs((WristEncoder.getPosition()/10.0*360.0)%360.0);
  }
  
  public Command setWristAngle(double angle)
  {
    return run(() -> {
      GrabberTwistController.setReference(Degrees.of(angle).in(Rotations)*10, ControlType.kPosition);
    });
  }

  //Grabber Command Factory
  public Command GrabberConverterCommand(CommandXboxController CraneController){ //double ElevatorSpeed, double ArmSpeed, double TwistSpeed, double GrabberSpeed

    return run(() -> {
      if (CraneController.start().getAsBoolean()){
        setZeroReferncePoint();
      }
      
      //double TwistSpeed = CraneController.getRightX();
      //GrabberTwistMax.set(TwistSpeed * GrabberConstants.TWIST_MULTIPLIER);
      //TODO Check which bumper needs negative twist restraint
      // Twister speed control
      
      if (CraneController.getLeftTriggerAxis() > 0.5) {
        GrabberTwistMax.set(-GrabberConstants.TWIST_MULTIPLIER);
      } else if (CraneController.getRightTriggerAxis() > 0.5) {
        GrabberTwistMax.set(GrabberConstants.TWIST_MULTIPLIER);
      }
      else if (CraneController.leftBumper().getAsBoolean()) {
        GrabberTwistMax.set(-GrabberConstants.TWIST_RESTRAINT);
      } else if (CraneController.rightBumper().getAsBoolean()) {
        GrabberTwistMax.set(GrabberConstants.TWIST_RESTRAINT);
      } else {
        GrabberTwistMax.set(0.0);
      }
        
    });
    
  }

  public Command autoWristCommand(double setPointDegrees) {
    return run(() -> {
      if(getAngleInDegrees() > setPointDegrees - 5 && getAngleInDegrees() < setPointDegrees + 5) {
        GrabberTwistMax.set(0.0);
      } else {
        GrabberTwistMax.set(GrabberConstants.WRIST_AUTO_SPEED);
      }
    }); 
  }

  public Command SetWristSpeed (double speed){
    return run(()-> {
      GrabberTwistMax.set(speed);
    });
  }

  
  public Command RunWrist (CommandXboxController HeightController){
    return run (() -> {


       if (HeightController.a().getAsBoolean()){
        SetPointAngle = GrabberConstants.WRIST_INTAKE_ANGLE;
      } else if (HeightController.x().getAsBoolean()) {
        SetPointAngle = GrabberConstants.WRIST_SCORING_ANGLE;
      } else if (HeightController.y().getAsBoolean()) {
        SetPointAngle = GrabberConstants.WRIST_SCORING_ANGLE;
      } else if (HeightController.getLeftTriggerAxis() > Operator.DEADBAND){
        SetPointAngle = SetPointAngle - HeightController.getLeftTriggerAxis() * GrabberConstants.WRIST_SETPOINT_MULTIPLIER;
      } else if (HeightController.getRightTriggerAxis() > Operator.DEADBAND){
        SetPointAngle = SetPointAngle + HeightController.getRightTriggerAxis() * GrabberConstants.WRIST_SETPOINT_MULTIPLIER;
      }

      GrabberTwistController.setReference(Degrees.of(SetPointAngle).in(Rotations)*10, ControlType.kPosition);
    });

 }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(GrabberTwistMax.getAppliedOutput());
    SmartDashboard.putNumber("Wrist Setpoint", SetPointAngle);
    SmartDashboard.putNumber("Wrist Position", getAngleInDegrees());
  }
}