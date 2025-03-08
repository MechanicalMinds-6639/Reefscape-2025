// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GrabberConstants;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private final SparkMax GrabberMax = new SparkMax(GrabberConstants.GRABBER_ID, MotorType.kBrushless);

  public Grabber() {}

  public Command Grab(int direction) {

    
    return run(() -> {
    if (direction == 1) {
      GrabberMax.set(GrabberConstants.GRABBER_SPEED);
    } else if (direction == -1) {
      GrabberMax.set(-GrabberConstants.GRABBER_SPEED);
    } else {
      System.out.println("YAY");
      GrabberMax.set(0);
    }
  
    });
  }

  public Command grabberDefaultCommand(CommandXboxController HeightController) {

    return run(() -> {
    if (HeightController.leftBumper().getAsBoolean()) {
      GrabberMax.set(GrabberConstants.GRABBER_SPEED);
    } else if (HeightController.rightBumper().getAsBoolean()) {
      GrabberMax.set(-GrabberConstants.GRABBER_SPEED);
    } else {
      GrabberMax.set(0.1);
    }
  
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
