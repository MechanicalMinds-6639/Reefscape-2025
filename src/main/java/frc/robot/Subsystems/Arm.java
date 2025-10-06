// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.Operator;



public class Arm extends SubsystemBase {
  //Arm Members
  private final SparkMax ArmMax = new SparkMax(CraneConstants.ARM_MOTOR_ID, MotorType.kBrushless);

   // Encoders for testing
  private final RelativeEncoder ArmEncoder = ArmMax.getEncoder();
  
  //Limit Switches
  private final DigitalInput Elbow = new DigitalInput(CraneConstants.LIMIT_ELBOW);

 //PID Controllers
   private final ProfiledPIDController ArmController = new ProfiledPIDController(CraneConstants.ARM_KP,
            CraneConstants.ARM_KI,
            CraneConstants.ARM_KD,
            new Constraints(CraneConstants.ARM_MAX_VELOCITY,
                    CraneConstants.ARM_MAX_ACCELERATION));
    private final ArmFeedforward ArmFeedforward = new ArmFeedforward(CraneConstants.ARM_KS,
            CraneConstants.ARM_KG,
            CraneConstants.ARM_KV,
            CraneConstants.ARM_KA);
            
    private double ArmDegreeSetPoint = 0.0;

    private double Multiplier = CraneConstants.ARM_SETPOINT_MULTIPLIER;



  /** Creates a new Arm. */
  public Arm() {

    // Sets PID in Arm Config
    SparkMaxConfig ArmConfig = new SparkMaxConfig();
    ArmConfig.smartCurrentLimit(40);
    ArmMax.configure(ArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ArmDegreeSetPoint = ArmEncoder.getPosition() *360/CraneConstants.ARM_REDUCTION;

  }

    public Command RunArm(CommandXboxController CraneController){

      return new WaitCommand(1).andThen(run (() -> {

        //System.out.println(ArmDegreeSetPoint);


        if (Math.abs(CraneController.getRightY()) > Operator.DEADBAND){
          //make if statement to go up if limit switch is pressed down and not allow down, 
          ArmDegreeSetPoint = ArmDegreeSetPoint - CraneController.getRightY() * Multiplier;
        } else if (CraneController.x().getAsBoolean()){
            ArmDegreeSetPoint = CraneConstants.ARM_L3_DEGREE;
        } else if (CraneController.a().getAsBoolean()){
          ArmDegreeSetPoint = CraneConstants.ARM_CORAL_INTAKE_DEGREE;
      }   else if (CraneController.y().getAsBoolean()){
          ArmDegreeSetPoint = CraneConstants.ARM_L4_SCORING_ANGLE;
      }

       if (CraneController.povUp().getAsBoolean()){
         Multiplier = CraneConstants.ARM_TURBO_SETPOINT_MULTIPLIER;
       } else if (CraneController.povRight().getAsBoolean()){
         Multiplier = CraneConstants.ARM_SETPOINT_MULTIPLIER;
       }

       if (!Elbow.get()){
        reachGoal(ArmDegreeSetPoint);
       } else if (Elbow.get() && ArmDegreeSetPoint>0) {
        reachGoal(ArmDegreeSetPoint);
       } else {
        ArmMax.set(0);
       }


       if (Elbow.get()){
        setZeroReferncePoint();
        ArmDegreeSetPoint = 2;
      }
      }));

    }



    public Command ArmConverterCommand(CommandXboxController CraneController){ //double ElevatorSpeed, double ArmSpeed, double TwistSpeed, double GrabberSpeed

    return run(() -> {


      if (Elbow.get()){
        setZeroReferncePoint();
      }

      double ArmSpeed = -CraneController.getRightY();
     
      if (Math.abs(ArmSpeed) < Operator.ARM_DEADBAND) {
        ArmMax.set(0.0);
      } 
      else if (Elbow.get() && ArmSpeed > 0){
        ArmMax.set(0.0);
      }
      else {
        ArmMax.set(-ArmSpeed * CraneConstants.ARM_MULTIPLIER);
      }

      // Arm speed control
      /* 
      if (Elbow.get() && ArmSpeed > 0){
        ArmMax.set(0);
      }  else if (ArmMax.getAbsoluteEncoder().getPosition() > 270 && ArmSpeed < 0){
        ArmMax.set(0);
      }  else {
        ArmMax.set(ArmSpeed * CraneConstants.ARM_MULTIPLIER);
      }
      */
    });
  }

    public Command setSpeed(double speed){
      return run(() -> {
        ArmMax.set(speed);
      });
    }

    public double getRotations(){
      return ArmEncoder.getPosition();
    }

   //methods/command
    public Angle getAngle() {
        return Degrees.of(Units.rotationsToDegrees(ArmEncoder.getPosition() / CraneConstants.ARM_REDUCTION));
    }

    public Double getAngleAsDouble() {
      return Units.rotationsToDegrees(ArmEncoder.getPosition() / CraneConstants.ARM_REDUCTION);
  }

    public AngularVelocity getAnglularVelocity() {
      return Degrees.of(Units.rotationsToDegrees(ArmEncoder.getVelocity() / CraneConstants.ARM_REDUCTION)).per(Minute);
  }

    public void reachGoal(double goalDegrees) {
        double goal = Degrees.of(goalDegrees).in(Rotations) * CraneConstants.ARM_REDUCTION;
        double clampedValue = MathUtil.clamp(
          ArmFeedforward.calculate(ArmController.getSetpoint().position, ArmController.getSetpoint().velocity)
            + ArmController.calculate(ArmEncoder.getPosition(), goal), -7, 7);
        ArmMax.setVoltage(clampedValue);
    }

    public Command setGoal(double goalDegrees) {
        return run(() -> reachGoal(goalDegrees));
    }

    public Command setArmAngle(double goalDegrees) {
        return setGoal(goalDegrees).until(() -> aroundAngle(goalDegrees));
    }

   public boolean aroundAngle(double degrees){
        return aroundAngle(degrees, CraneConstants.ARM_DEFAULT_TOLERANCE);
   }

   public boolean aroundAngle(double degrees, double tolerance){
        return MathUtil.isNear(degrees, getAngle().in(Degrees),tolerance);
   }

   public Command autoArmCommand(double setPoint) {
    return run (() -> {
      if (ArmEncoder.getPosition() >= setPoint) {
        ArmMax.set(0);
      } else {
        ArmMax.set(CraneConstants.ARM_MAX_SPEED);
      }
    });
   }

   public Command autoPID(double setPoint) {
    return run (() -> {
      ArmDegreeSetPoint = setPoint;
      reachGoal(ArmDegreeSetPoint);
    });
   }

  public void setZeroReferncePoint(){
    ArmEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(ArmDegreeSetPoint);
    SmartDashboard.putNumber("Arm Setpoint", ArmController.getSetpoint().position);
    SmartDashboard.putNumber("Arm Position", getRotations());
  }
}