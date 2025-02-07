// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CraneConstants;

public class Crane extends SubsystemBase {
  // Members
  private final SparkMax ArmMax = new SparkMax(CraneConstants.ARM_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax ElevatorRMax = new SparkMax(CraneConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);
  private final SparkMax ElevatorLMax = new SparkMax(CraneConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);
  private final SparkMax GrabberRMax = new SparkMax(CraneConstants.GRABBER_RIGHT_ID, MotorType.kBrushless);
  private final SparkMax GrabberLMax = new SparkMax(CraneConstants.GRABBER_LEFT_ID,MotorType.kBrushless);
  private final SparkMax GrabberTwistMax = new SparkMax(CraneConstants.GRABBER_TWIST_ID, MotorType.kBrushless);

  /** Creates a new Crane. */
  public Crane() {
    SparkMaxConfig RightMax = new SparkMaxConfig();
      RightMax.inverted(true);
      ElevatorRMax.configure(RightMax, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      GrabberRMax.configure(RightMax, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
