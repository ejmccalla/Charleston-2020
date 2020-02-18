package frc.robot;

public final class Constants {

    // Pressure sensor
    public static final int PRESSURE_SENSOR_ANALOG_CHANNEL = 0;
    public static final double PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE = 1.19;               // Measure by reading analog input voltage @ 0-PSI 
    public static final double PRESSURE_SENSOR_PRESSURE_PER_VOLT = 115.0 / (3.62 - 1.19);   // Calculate with prior measurement and reading analog input voltage @ max operating PSI 

    // Photoeye
    public static final int PHOTOEYE_DIGITAL_CHANNEL = 0;

    // Drivetrain device ID's and ports
    public static final int DRIVETRAIN_LEFT_MASTER_ID = 15;
    public static final int DRIVETRAIN_LEFT_FOLLOWER_1_ID = 14;
    public static final int DRIVETRAIN_LEFT_FOLLOWER_2_ID = 13;
    public static final int DRIVETRAIN_RIGHT_MASTER_ID = 0;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER_1_ID = 1;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER_2_ID = 2;
    public static final int DRIVETRAIN_LOW_GEAR_SOLENOID_ID = 0;
    public static final int DRIVETRAIN_HIGH_GEAR_SOLENOID_ID = 1;

    // Controllers and Joysticks
    public static final int DRIVER_JOYSTICK_THROTTLE = 1;
    public static final int DRIVER_JOYSTICK_TURN = 0;
    public static final int DRIVER_XBOX = 2;
    public static final int OPERATOR_XBOX = 3;

    // CAN bus
    public static final int CAN_TIMEOUT_MS = 10;
    public static final int CAN_LONG_TIMEOUT_MS = 100;

    // MISC. Hardware Device ID's
    public static final int PCM_ID = 0;
    public static final int PDP_ID = 0;

}
