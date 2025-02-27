package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.RobotMath.ArmMath;;



public class Constants {
    
    public static final class Drive {
        public static final double MAX_SPEED = 1.0; //Meters per second

        public static final int FL_ABS_ENCODER_ID = 0;
        public static final int FR_ABS_ENCODER_ID = 3;
        public static final int BL_ABS_ENCODER_ID = 1;
        public static final int BR_ABS_ENCODER_ID = 2;
    }

    public static final class Operator {
        public static final int DRIVER = 0;
        public static final int DRIVER2 = 1;
        public static final double DEADBAND = 0.1;
    }

    public static final class CraneConstants {
        // TODO Set IDs
        public static final int ARM_MOTOR_ID = 0;
        public static final int ELEVATOR_RIGHT_ID = 0;
        public static final int ELEVATOR_LEFT_ID = 0;
        public static final int GRABBER_RIGHT_ID = 0;
        public static final int GRABBER_LEFT_ID = 0;
        public static final int GRABBER_TWIST_ID = 0;

        public static final int LIMIT_TOP = 0;
        public static final int LIMIT_BOTTOM = 0;
        public static final int LIMIT_ELBOW = 0;

        // TODO set multipliers
        // Speed Multipliers
        public static final double TWIST_MULTIPLIER = 0.25;
        public static final double ARM_MULTIPLIER = 0.25;
        public static final double ELEVATOR_MULTIPLIER = 0.25;
        public static final double GRABBER_MULTIPLIER = 0.25;

        // TODO set speed
        public static final double ELEVATOR_MAX_SPEED = 0.2;
        public static final double ELEVATOR_MAX_VELOCITY = Meters.of(4).per(Second).in(MetersPerSecond);
        public static final double ELEVATOR_MAX_ACCELERATION = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final double ARM_MAX_SPEED = 0.2;
        public static final double ARM_MAX_ACCELERATION = ArmMath.convertArmAngleToSensorUnits(Degrees.of(180))
            .per(Second).per(Second).in(RPM.per(Second)) ;
        public static final double ARM_MAX_VELOCITY = ArmMath.convertArmAngleToSensorUnits(Degrees.of(90))//?
            .per(Second).in(RPM) ;

        // TODO PID Constants
        public static final double ELEVATOR_KP = 0;
        public static final double ELEVATOR_KI = 0;
        public static final double ELEVATOR_KD = 0;
        public static final double ELEVATOR_KG = 1.09;
        public static final double ELEVATOR_KV = 65.73;
        public static final double ELEVATOR_KA = 0.14;
        public static final double ELEVATOR_KS = 0;
        public static final double ARM_KI = 0;
        public static final double ARM_KD = 0;
        public static final double ARM_KG = 4.01;
        public static final double ARM_KV = 0.39;
        public static final double ARM_KA = 0.21;

        //Elevator Constants
        public static final double ELEVATOR_DRUM_RADIUS = 0.0222377; //pitch radius of that one gear on the elevator, yeah
        public static final double ELEVATOR_RAMP_RATE = 0.1; //Something else 
        public static final double ELEVATOR_GEARING = 9;
        public static final double ELEVATOR_DEFAULT_TOLERANCE =  Inches.of(1).in(Meters);

        //Arm Constants
        public static final double ARM_REDUCTION = 20;
        public static final double ARM_DEFAULT_TOLERANCE = 1;

    }
}
