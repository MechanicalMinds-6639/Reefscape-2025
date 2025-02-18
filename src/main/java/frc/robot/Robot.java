// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

     // Constants such as camera and target height stored. Change per robot and goal!
     final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
     final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
     // Angle between horizontal and the camera.
     final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  
     // How far from the target we want to be
     final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  
     // Change this to match the name of your camera
     PhotonCamera camera = new PhotonCamera("VisionCamera");

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    var result = camera.getLatestResult();
    int ID = 0;
    try {
      ID = result.getBestTarget().fiducialId;
    } catch (Exception e) {
      ID = 99;
    }   

    SmartDashboard.putNumber("FiducialID", ID);
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
