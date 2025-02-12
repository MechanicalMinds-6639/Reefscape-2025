// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;

public class Crane extends SubsystemBase {
  // Members
  private final SparkMax ArmMax = new SparkMax(CraneConstants.ARM_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax ElevatorRMax = new SparkMax(CraneConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);
  private final SparkMax ElevatorLMax = new SparkMax(CraneConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
  //private final SparkMax GrabberRMax = new SparkMax(CraneConstants.GRABBER_RIGHT_ID, MotorType.kBrushless);
  private final SparkMax GrabberMax = new SparkMax(CraneConstants.GRABBER_LEFT_ID,MotorType.kBrushless);
  private final SparkMax GrabberTwistMax = new SparkMax(CraneConstants.GRABBER_TWIST_ID, MotorType.kBrushless);

  private final DigitalInput ElevatorTop = new DigitalInput(CraneConstants.LIMIT_TOP);
  private final DigitalInput ElevatorBottom = new DigitalInput(CraneConstants.LIMIT_BOTTOM);
  private final DigitalInput Elbow = new DigitalInput(CraneConstants.LIMIT_ELBOW);

  /** Creates a new Crane. */
  public Crane() {
    // Sets configuration for right elevator sparkmax
    SparkMaxConfig RightElevatorConfig = new SparkMaxConfig();
    RightElevatorConfig.inverted(true);
    RightElevatorConfig.follow(ElevatorLMax);
    ElevatorRMax.configure(RightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*
   * Command to control the crane motion
   */
  public Command CraneConverterCommand(CommandXboxController CraneController){ //double ElevatorSpeed, double ArmSpeed, double TwistSpeed, double GrabberSpeed

    return run(() -> {


      double ElevatorSpeed = CraneController.getLeftY();
      double ArmSpeed = CraneController.getRightY();
      double TwistSpeed = CraneController.getRightX();


      // Elevator speed control
      if (ElevatorTop.get() && ElevatorSpeed > 0) {
        ElevatorLMax.set(0);
      } else if (ElevatorBottom.get() && ElevatorSpeed < 0){
        ElevatorLMax.set(0);
      } else {
        ElevatorLMax.set(ElevatorSpeed * CraneConstants.ELEVATOR_MULTIPLIER);
      }

      // Arm speed control
      if (Elbow.get() && ArmSpeed > 0){
        ArmMax.set(0);
      } else if (ArmMax.getAbsoluteEncoder().getPosition() > 270 && ArmSpeed < 0){
        ArmMax.set(0);
      } else {
        ArmMax.set(ArmSpeed * CraneConstants.ARM_MULTIPLIER);
      }
      
      // Twister speed control
      GrabberTwistMax.set(TwistSpeed * CraneConstants.TWIST_MULTIPLIER);
    });
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
