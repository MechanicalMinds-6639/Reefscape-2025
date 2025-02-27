// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;

public class Grabber extends SubsystemBase {
  //Members
  private final SparkMax GrabberMax = new SparkMax(CraneConstants.GRABBER_LEFT_ID,MotorType.kBrushless);
  private final SparkMax GrabberTwistMax = new SparkMax(CraneConstants.GRABBER_TWIST_ID, MotorType.kBrushless);

   // Encoders for testing
  private final RelativeEncoder GrabberTwistEncoder = GrabberTwistMax.getEncoder();

  /** Creates a new Grabber. */
  public Grabber() {}



  //Grabber Command Factory
  public Command GrabberConverterCommand(CommandXboxController CraneController){ //double ElevatorSpeed, double ArmSpeed, double TwistSpeed, double GrabberSpeed

    return run(() -> {

      double TwistSpeed = CraneController.getRightX();
      
      // Twister speed control
      GrabberTwistMax.set(TwistSpeed * CraneConstants.TWIST_MULTIPLIER);
    });
    
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
