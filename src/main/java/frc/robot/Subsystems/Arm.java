// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.Operator;
import frc.robot.RobotMath.ArmMath;
import static edu.wpi.first.units.Units.*;



public class Arm extends SubsystemBase {
  //Arm Members
  private final SparkMax ArmMax = new SparkMax(CraneConstants.ARM_MOTOR_ID, MotorType.kBrushless);

   // Encoders for testing
  private final RelativeEncoder ArmEncoder = ArmMax.getEncoder();
  
  //Limit Switches
  private final DigitalInput Elbow = new DigitalInput(CraneConstants.LIMIT_ELBOW);

 //PID Controllers
   private final ProfiledPIDController ArmController = new ProfiledPIDController(CraneConstants.ELEVATOR_KP,
            CraneConstants.ARM_KI,
            CraneConstants.ARM_KD,
            new Constraints(CraneConstants.ARM_MAX_VELOCITY,
                    CraneConstants.ARM_MAX_ACCELERATION));
    private final ArmFeedforward ArmFeedforward = new ArmFeedforward(CraneConstants.ELEVATOR_KS,
            CraneConstants.ARM_KG,
            CraneConstants.ARM_KV,
            CraneConstants.ARM_KA);



  /** Creates a new Arm. */
  public Arm() {

    // Sets PID in Arm Config
    SparkMaxConfig ArmConfig = new SparkMaxConfig();
    ArmConfig.closedLoop.p(0);
    ArmConfig.smartCurrentLimit(40);

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

   //methods/command
    public Angle getAngle() {
        return ArmMath.convertSensorUnitsToArmAngle(Rotations.of(ArmEncoder.getPosition()));
    }

    public AngularVelocity getVelocity() {
        return ArmMath.convertSensorUnitsToArmAngle(Rotations.of(ArmEncoder.getVelocity())).per(Minute);
    }

    public void reachGoal(double goalDegrees) {
        double goal = ArmMath.convertArmAngleToSensorUnits(Degrees.of(goalDegrees)).in(Rotations);

        ArmMax.setVoltage(ArmFeedforward.calculate(ArmController.getSetpoint().position, ArmController.getSetpoint().velocity)
                + ArmController.calculate(ArmEncoder.getPosition(), goal));
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


  public void setZeroReferncePoint(){
    ArmEncoder.setPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}