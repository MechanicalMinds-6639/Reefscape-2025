// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.CraneConstants;


public class RobotMath
{
 public static class ArmMath{
   public static Angle convertArmAngleToSensorUnits(Angle measurement){
     return Rotations.of(measurement.in(Rotations) * CraneConstants.ARM_REDUCTION);
   }
   public static Angle convertSensorUnitsToArmAngle(Angle measurement){
     return Rotations.of(measurement.in(Rotations) / CraneConstants.ARM_REDUCTION);
   }
 }
}