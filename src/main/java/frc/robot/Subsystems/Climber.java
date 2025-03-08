// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {

  //Members
  private final SparkMax ClimbMax = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber() {}

  public Command climberDefaultCommand(CommandXboxController xboxController){
    return run (()-> {
      if (xboxController.leftTrigger().getAsBoolean()) {
        ClimbMax.set(ClimbConstants.CLIMBER_SPEED);
      } else if (xboxController.rightTrigger().getAsBoolean()) {
        ClimbMax.set(-ClimbConstants.CLIMBER_SPEED);
      } else {
        ClimbMax.set(0);
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
