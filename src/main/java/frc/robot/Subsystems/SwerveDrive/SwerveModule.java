package frc.robot.Subsystems.SwerveDrive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class SwerveModule {

    private final SparkMax m_driveMotor;
    private final SparkMax m_turnMotor;

    private final RelativeEncoder m_driveEncoder;
    private final AbsoluteEncoder m_turnEncoder;

    private final SparkClosedLoopController m_drivingController;
    private final SparkClosedLoopController m_turningController;
  
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    public SwerveModule(int driveID, int turnID, int encoderID, double chassisAngularoffset) {
    m_driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnID, MotorType.kBrushless);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getAbsoluteEncoder();

    m_drivingController = m_driveMotor.getClosedLoopController();
    m_turningController = m_turnMotor.getClosedLoopController(); 
    
    m_chassisAngularOffset = chassisAngularoffset;

    m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
    m_driveEncoder.setPosition(0);
    
    }

    /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
    public SwerveModuleState getState() {

        double wheelCircumference = Math.PI * 0.1016; //wheel diameter in meters
        double gearRatio = 6.75; //motor rotations per wheel rotation
        double velocityMetersPerSecond = (m_driveEncoder.getVelocity() / 60.0) * wheelCircumference / gearRatio;
        //driveEncoder.getVelocity() gives RPM (rotations per minute)
        //divided by 60.0 converts RPM to RPS (rotations per second)
        //wheelCircumference converts rotations to meters
        //divided by gearRatio accounts for drivetrain reduction

        double speed = velocityMetersPerSecond; //m/s if converted (Conversion Above)
        double angle = m_turnEncoder.getPosition() - m_chassisAngularOffset; //angle in radians
        return new SwerveModuleState(speed, new Rotation2d(angle));
    }

    /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
    public SwerveModulePosition getPosition() {
    
        // Apply chassis angular offset to the encoder position to get the position relative to the chassis.
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset)
        );
    }

    /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turnEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    
        if (Math.abs(correctedDesiredState.speedMetersPerSecond) > 0.05) {
        m_turningController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
        }

        m_desiredState = desiredState;
  }
}
