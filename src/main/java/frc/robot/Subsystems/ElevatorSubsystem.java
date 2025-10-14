// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu. wpi. first. math. trajectory. TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.Operator;

public class ElevatorSubsystem extends SubsystemBase {
  // Members
  //Sparkmax definitions
  
  private final SparkMax ElevatorRMax = new SparkMax(CraneConstants.ELEVATOR_RIGHT_ID, MotorType.kBrushless);
  private final SparkMax ElevatorLMax = new SparkMax(CraneConstants.ELEVATOR_LEFT_ID, MotorType.kBrushless);

  // Encoders for testing
  private final RelativeEncoder ElevatorEncoder = ElevatorLMax.getEncoder();

  //Limit switches
  private final DigitalInput ElevatorTop = new DigitalInput(CraneConstants.LIMIT_TOP);
  private final DigitalInput ElevatorBottom = new DigitalInput(CraneConstants.LIMIT_BOTTOM);

  private double SetPointHeight = 0.0;

  private boolean elevatorMode = true;

  //PID Controllers
   private final ProfiledPIDController ElevatorController = new ProfiledPIDController(CraneConstants.ELEVATOR_KP,
            CraneConstants.ELEVATOR_KI,
            CraneConstants.ELEVATOR_KD,
            new Constraints(CraneConstants.ELEVATOR_MAX_VELOCITY,
                    CraneConstants.ELEVATOR_MAX_ACCELERATION));
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(CraneConstants.ELEVATOR_KS,
            CraneConstants.ELEVATOR_KG,
            CraneConstants.ELEVATOR_KV,
            CraneConstants.ELEVATOR_KA);
    private double Multiplier = CraneConstants.ELEVATOR_SETPOINT_MULTIPLIER;


  /** Creates a new Crane. */
  public ElevatorSubsystem() {
    // Sets configuration for right elevator sparkmax
    SparkMaxConfig RightElevatorConfig = new SparkMaxConfig();
    RightElevatorConfig.follow(ElevatorLMax, true);
    ElevatorRMax.configure(RightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightElevatorConfig.smartCurrentLimit(40);

    SparkMaxConfig LeftElevatorConfig = new SparkMaxConfig();
    LeftElevatorConfig.smartCurrentLimit(40)
      .openLoopRampRate(CraneConstants.ELEVATOR_RAMP_RATE);
      ElevatorLMax.configure(LeftElevatorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);//
    
    }

  public Command RunElevator (CommandXboxController HeightController, CommandXboxController DriverController){
    //true is PID control

    return run (() -> {

      //System.out.println(HeightController.povDown().getAsBoolean()+" : "+DriverController.povDown().getAsBoolean());

      if (elevatorMode) {

        if (HeightController.povDown().getAsBoolean() && DriverController.povDown().getAsBoolean()) {
          elevatorMode = false;
          System.out.println("OVERRIDE: Elevator no longer using PID control");
        }
          
          //Sets elevator encoder to 0 when bottom limit switch is hit
        if (HeightController.b().getAsBoolean()){
          reset();
        }
        
        if (Math.abs(HeightController.getLeftY()) > Operator.DEADBAND){
          SetPointHeight = SetPointHeight - HeightController.getLeftY() * Multiplier;
        } else if (HeightController.a().getAsBoolean()){
            SetPointHeight = CraneConstants.ELEVATOR_CORAL_INTAKE_HEIGHT;
        } else if (HeightController.b().getAsBoolean()){
            SetPointHeight = CraneConstants.ELEVATOR_CORAL_TROUGH_HEIGHT;
        } else if (HeightController.x().getAsBoolean()){
            SetPointHeight = CraneConstants.ELEVATOR_L3_HEIGHT;
        } else if (HeightController.y().getAsBoolean()){
            SetPointHeight = CraneConstants.ELEVATOR_L4_HEIGHT;
        }
        
         if (HeightController.povUp().getAsBoolean()){
           Multiplier = CraneConstants.ELEVATOR_TURBO_SETPOINT_MULTIPLIER;
           System.out.println("Turbo Mode Activated");
         } else if (HeightController.povRight().getAsBoolean()){
           Multiplier = CraneConstants.ELEVATOR_SETPOINT_MULTIPLIER;
           System.out.println("Turbo Mode Disabled");
         }

        reachGoal(SetPointHeight);
      } else {
        if (HeightController.b().getAsBoolean()){
          ElevatorEncoder.setPosition(0);
        }
        //System.out.println(getPositionMeters());

        double ElevatorSpeed = HeightController.getLeftY();

        // Elevator speed control
        if ((Math.abs(ElevatorSpeed) < Operator.DEADBAND) ||
            (ElevatorTop.get() && ElevatorSpeed < 0)      ||
            ElevatorBottom.get() && ElevatorSpeed > 0){
          ElevatorLMax.set(0.0);
        } else {
          ElevatorLMax.set(-ElevatorSpeed * CraneConstants.ELEVATOR_MULTIPLIER);
        }
      }
        
    });
  
    

 }


  
    public double getPositionMeters() {
        return ElevatorEncoder.getPosition() * (2 * Math.PI * CraneConstants.ELEVATOR_DRUM_RADIUS)
                / CraneConstants.ELEVATOR_GEARING;
    }

    public double getVelocityMetersPerSecond() {
        return (ElevatorEncoder.getVelocity() / 60) * (2 * Math.PI * CraneConstants.ELEVATOR_DRUM_RADIUS)
                / CraneConstants.ELEVATOR_GEARING;
    }

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
                elevatorFeedforward.calculateWithVelocities(getVelocityMetersPerSecond(), ElevatorController.getSetpoint().velocity)
                + ElevatorController.calculate(getPositionMeters(), goal),
                -7,
                7);
        ElevatorLMax.setVoltage(voltsOutput);
    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    
    public Command stop(){
      return run(()-> 
        ElevatorLMax.set(0));
        
  }

    public boolean aroundHeight(double height){
        return aroundHeight(height, CraneConstants.ELEVATOR_DEFAULT_TOLERANCE);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

  public void reset() {
    SetPointHeight = 0;
    System.out.println("Elevator setpoint to zero");
  }

  public void resetElevatorCommand() {
    System.out.println("Elevator using PID control");
    elevatorMode = true;
  }

  public Command autoPID(double setPoint) {
    return run (() -> {
      SetPointHeight = setPoint;
      reachGoal(SetPointHeight);
    });
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Setpoint", ElevatorController.getSetpoint().position);
    SmartDashboard.putNumber("Elevator Position", getPositionMeters());
    if (SetPointHeight < 0) {
      SetPointHeight = 0;
    }
  }
}