package frc.robot;

import frc.robot.lib.controllers.LimelightVision.DistanceEstimator_t;

public final class Constants {

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the hardware which
    * isn't used by any individual subsystem.
    * @see {@link frc.robot.lib.drivers.Photoeye}
    */      
    public static final class HARDWARE {
        public static final int CTRE_CAN_TIMEOUT_MS = 10;                       // CTRE CAN normal CAN timeout for blocking calls
        public static final int CTRE_CAN_LONG_TIMEOUT_MS = 100;                 // CTRE CAN long CAN timeout for blocking calls
        public static final int PCM_ID = 0;                                     // PCM CAN ID
        public static final int PDP_ID = 0;                                     // PCM CAN ID
        public static final int PHOTOEYE_DIGITAL_CHANNEL = 0;                   // DIO channel the photoeye is connected to
    }

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the driver and
    * operator controls.
    * @see {@link frc.robot.RobotContainer#ConfigureButtonBindings}
    */      
    public static final class DRIVER {
        public static final int JOYSTICK_THROTTLE = 1;                          // Throttle joystick port number
        public static final int JOYSTICK_TURN = 0;                              // Turn joystick port number
        public static final double QUICKTURN_THRESHOLD = 0.05;                  // Threshold for curvature drive where the quickturn feature is enabled
    }

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the pressure
    * sensor.
    * @see {@link frc.robot.lib.drivers.PressureSensor}
    */   
    public static final class PRESSURE_SENSOR {
        public static final int ANALOG_CHANNEL = 0;                             // Analog channel the pressure sensor is connected to
        public static final double VOLTS_AT_ZERO_PRESSURE = 1.19;               // Measure by reading analog input voltage @ 0-PSI 
        public static final double PSI_PER_VOLT = 115.0 / (3.62 - 1.19);        // Calculate with prior measurement and reading analog input voltage @ max operating PSI 
    }    

    /**
    * These are the constants which are used to map the hardware and define the working bahavior of the the drivetrain
    * subsystem.  The Limelight 2 has a horizontal FOV of -29.8 to 29.8 degrees, which means the max error is within
    * this range as well.  To drive a mechanism at 25% at max Limelight error, set the P-gain to 0.25 / 29.8.
    * @see {@link frc.robot.subsystems.Drivetrain}
    * @see {@link https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#}
    * @see {@link https://docs.limelightvision.io/en/latest/networktables_api.html}
    */
    public static final class DRIVETRAIN {
        public static final int LEFT_MASTER_ID = 15;                            // Motor controller CAN ID AND PDP Port number
        public static final int LEFT_FOLLOWER_1_ID = 14;                        // Motor controller CAN ID AND PDP Port number
        public static final int LEFT_FOLLOWER_2_ID = 13;                        // Motor controller CAN ID AND PDP Port number
        public static final int RIGHT_MASTER_ID = 0;                            // Motor controller CAN ID AND PDP Port number
        public static final int RIGHT_FOLLOWER_1_ID = 1;                        // Motor controller CAN ID AND PDP Port number
        public static final int RIGHT_FOLLOWER_2_ID = 2;                        // Motor controller CAN ID AND PDP Port number
        public static final int LOW_GEAR_SOLENOID_ID = 0;                       // PCM port number for low gear shifting
        public static final int HIGH_GEAR_SOLENOID_ID = 1;                      // PCM port number for high gear shifting
        public static final boolean VISION_THREADED = true;                     // Vision threading flag 
        public static final double VISION_SEARCH_TIMEOUT_S = 3.0;               // Vision search (look for target) timeout threshold
        public static final double VISION_SEEK_TIMEOUT_S = 2.0;                 // Vision seek (get on target) timeout threshold
        public static final int VISION_SEEK_RETRY_LIMIT = 3;                    // Vision seek retry limit threshold
        public static final double VISION_TURN_PID_P = 2.98 / 29.8;             // Vision turn-to-target command proportional gain
        public static final double VISION_TURN_PID_I = 0.0;                     // Vision turn-to-target command intgral gain
        public static final double VISION_TURN_PID_D = 0.0;                     // Vision turn-to-target command derivative gain
        public static final double VISION_TURN_PID_F = 0.0;                     // Vision turn-to-target command feed-forward
        public static final double VISION_DISTANCE_PID_ALPHA = 0.2;             // Vision drive-to-target command *proportional gain
        public static final double VISION_ON_TARGET_TURN_THRESHOLD_DEG = 1.5;   // Vision turn-to-target command acceptable error threshold
        public static final double VISION_ON_TARGET_DISTANCE_THRESHOLD_FT = 0.5;// Vision drive-to-target command acceptable error threshold
        public static final DistanceEstimator_t VISION_DISTANCE_ESTIMATOR = DistanceEstimator_t.BoundingBox; // Vision distance estimator calculation
        public static final double VISION_TARGET_WIDTH_FT = 21.5 / 12.0;        // Vision known target width in feet (bounding box distance estimator)
        public static final double VISION_FOCAL_LENGTH_FT = (67.0*7.0)/VISION_TARGET_WIDTH_FT;// Vision focal length = (measured_target_pixels * known_distance_to_taget) / known_target_width (bounding box distance estimator)
        public static final double VISION_FLOOR_TO_TARGET_FT = 26.0 / 12.0;     // Vision known floor-to-target (bounding box distance estimator)
        public static final double VISION_FLOOR_TO_LIMELIGHT_FT = 1.0;          // Vision known floor-to-limelight (bounding box distance estimator)
        public static final double VISION_LIMELIGHT_MOUNT_ANGLE_DEG = 0.0;      // Vision known limelight mounting angle (bounding box distance estimator)
    }

}
