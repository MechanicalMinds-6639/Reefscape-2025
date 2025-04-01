package frc.robot;

import static edu.wpi.first.units.Units.*;

public class Constants {
    
    public static final class Drive {
        public static final double MAX_SPEED = 3.0; //Meters per second

        public static final int FL_ABS_ENCODER_ID = 0;
        public static final int FR_ABS_ENCODER_ID = 3;
        public static final int BL_ABS_ENCODER_ID = 1;
        public static final int BR_ABS_ENCODER_ID = 2;
    }

    public static final class Operator {
        public static final int DRIVER = 0;
        public static final int COPILOT = 1;
        public static final double DEADBAND = 0.1;
        public static final double ARM_DEADBAND = 0.3;
    }

    public static final class CraneConstants {
        // TODO Set IDs
        public static final int ARM_MOTOR_ID = 13;
        public static final int ELEVATOR_RIGHT_ID = 12;
        public static final int ELEVATOR_LEFT_ID = 3;
       
        public static final int LIMIT_TOP = 1;
        public static final int LIMIT_BOTTOM = 2;
        public static final int LIMIT_ELBOW = 0;

        // TODO set multipliers
        // Speed Multipliers
        public static final double TWIST_MULTIPLIER = 0.25;
        public static final double ARM_MULTIPLIER = 0.5;
        public static final double ELEVATOR_MULTIPLIER = 0.25;
        public static final double GRABBER_MULTIPLIER = 0.25;
      
        //Arm Constants
        public static final double ARM_REDUCTION = 416.66667;
        public static final double ARM_DEFAULT_TOLERANCE = 1;
      
        // TODO set speed
        public static final double ELEVATOR_MAX_SPEED = 0.2;
        public static final double ELEVATOR_MAX_VELOCITY = Meters.of(0.5).per(Second).in(MetersPerSecond);
        public static final double ELEVATOR_MAX_ACCELERATION = Meters.of(0.5).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final double ARM_MAX_SPEED = 0.2;
        public static final double ARM_MAX_ACCELERATION = Degrees.of(30).per(Second).per(Second).in(RPM.per(Second)) * ARM_REDUCTION;
        public static final double ARM_MAX_VELOCITY = Degrees.of(60).per(Second).in(RPM) * ARM_REDUCTION;

        // TODO PID Constants
        public static final double ELEVATOR_KP = 10;
        public static final double ELEVATOR_KI = 0;
        public static final double ELEVATOR_KD = 0;
        public static final double ELEVATOR_KG = 0.5;
        public static final double ELEVATOR_KV = 7.4;//8;
        public static final double ELEVATOR_KA = 0.1;
        public static final double ELEVATOR_KS = 0;
        public static final double ELEVATOR_SETPOINT_MULTIPLIER = 0.005;
        public static final double ELEVATOR_MAX_HEIGHT = 0.56;
        public static final double ELEVATOR_L3_HEIGHT = 0.3;
        public static final double ELEVATOR_L4_HEIGHT = 0.555;
        public static final double ELEVATOR_CORAL_INTAKE_HEIGHT = 0.01;
        public static final double ARM_KI = 0;
        public static final double ARM_KD = 0;
        public static final double ARM_KG = 0; //4.01;
        public static final double ARM_KV = 0.4; //.25;
        public static final double ARM_KA = 0; //.21;
        public static final double ARM_KP = 0.6;
        public static final double ARM_KS = 0;
        public static final double ARM_L3_DEGREE = 32.0;
        public static final double ARM_CORAL_INTAKE_DEGREE = 20.0;
        public static final double ARM_SETPOINT_MULTIPLIER = -0.4;
        public static final double ARM_L4_SCORING_ANGLE = 15;

        //Elevator Constants
        public static final double ELEVATOR_DRUM_RADIUS = 0.0222377; //pitch radius of that one gear on the elevator, yeah
        public static final double ELEVATOR_RAMP_RATE = 0.1; //Something else 
        public static final double ELEVATOR_GEARING = 9;
        public static final double ELEVATOR_DEFAULT_TOLERANCE =  Inches.of(1).in(Meters);



    }
        //Wrist/Grabber Constants
    public static final class GrabberConstants {
        public static final int WRIST_CURRENT_LIMIT = 40;
        public static final double WRIST_CLOSED_LOOP_RAMP_RATE = 0.25;
        public static final int GRABBER_ID = 14;
        public static final int GRABBER_TWIST_ID = 11;
        public static final double TWIST_MULTIPLIER = 0.1;
        public static final double WRIST_AUTO_SPEED = 0.1;
        public static final double GRABBER_SPEED = 0.6;
        public static final double TWIST_RESTRAINT = 0.02;
        public static final double WRIST_INTAKE_ANGLE = 90;
        public static final double WRIST_SCORING_ANGLE = 0;
        public static final double WRIST_KP = 0.4;
        public static final double WRIST_KI = 0;
        public static final double WRIST_KD = 2;
        //BELOW NEEDS TO BE RAISED
        public static final double WRIST_SETPOINT_MULTIPLIER = 10;
        public static final double WRIST_MAX_OUTPUT = 0.4;
    }

    public static final class ClimbConstants {
        // TODO Set IDS
        public static final int CLIMB_MOTOR_ID = 30;
        public static final double CLIMBER_SPEED = 0.75;

    }
}
