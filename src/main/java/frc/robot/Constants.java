package frc.robot;

public final class Constants {

    public static final class HARDWARE {
        public static final int CTRE_CAN_TIMEOUT_MS = 10;
        public static final int CTRE_CAN_LONG_TIMEOUT_MS = 100;
        public static final int PCM_ID = 0;
        public static final int PDP_ID = 0;
        public static final int DRIVER_JOYSTICK_THROTTLE = 1;
        public static final int DRIVER_JOYSTICK_TURN = 0;
        public static final int PHOTOEYE_DIGITAL_CHANNEL = 0;
    }

    public static final class PRESSURE_SENSOR {
        public static final int ANALOG_CHANNEL = 0;
        public static final double VOLTS_AT_ZERO_PRESSURE = 1.19;               // Measure by reading analog input voltage @ 0-PSI 
        public static final double PSI_PER_VOLT = 115.0 / (3.62 - 1.19);        // Calculate with prior measurement and reading analog input voltage @ max operating PSI 
    }    

    public static final class DRIVETRAIN {
        public static final int LEFT_MASTER_ID = 15;
        public static final int LEFT_FOLLOWER_1_ID = 14;
        public static final int LEFT_FOLLOWER_2_ID = 13;
        public static final int RIGHT_MASTER_ID = 0;
        public static final int RIGHT_FOLLOWER_1_ID = 1;
        public static final int RIGHT_FOLLOWER_2_ID = 2;
        public static final int LOW_GEAR_SOLENOID_ID = 0;
        public static final int HIGH_GEAR_SOLENOID_ID = 1;
        public static final double VISION_TURN_PID_KP = 0.0;                    // Vision turn closed-loop proportional gain
        public static final double VISION_TURN_PID_KI = 0.0;                    // Vision turn closed-loop intgral gain
        public static final double VISION_TURN_PID_KD = 0.0;                    // Vision turn closed-loop derivative gain
        public static final double VISION_TURN_PID_KF = 0.0;                    // Vision turn closed-loop feed-forward
    }

}
