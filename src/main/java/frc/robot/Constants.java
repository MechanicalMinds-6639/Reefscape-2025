package frc.robot;

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
        public static final double DEADBAND = 0.1;
    }
}
