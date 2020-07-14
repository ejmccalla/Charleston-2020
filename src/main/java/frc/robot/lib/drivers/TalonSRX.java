package frc.robot.lib.drivers;

import frc.robot.Constants.HARDWARE;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* This class provides methods to configure Talon SRX motor controllers as both masters and followers.  Also, any
* sticky errors present will be logged and cleared.
*
* @see {@link https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html}
*/
public class TalonSRX {

    private static final Logger mLogger = LoggerFactory.getLogger( TalonSRX.class );

    /**
    * This method is intended to initialize the motor controller to default configuration common to all modes or
    * operation.
    * <p><ul>
    * <li>Sticky faults will be logged and cleared
    * <li>Reset to factory defaults
    * <li>Voltage compensation is enabled and set for 12V
    * </ul>
    *
    * @param talon WPI_TalonSRX The motor controller to initialize
    */
    public static void CommonConfig ( WPI_TalonSRX talon ) {
        StickyFaults faults = new StickyFaults();

        talon.getStickyFaults( faults );
        if ( faults.hasAnyFault() ) {
            mLogger.warn( "Clearing TalonSRX [{}] sticky faults: [{}]", talon.getDeviceID(), faults.toString() );
            final ErrorCode clearStickyFaults = talon.clearStickyFaults( HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
            if ( clearStickyFaults != ErrorCode.OK ) {
                mLogger.error( "Could not clear sticky faults due to EC: [{}]", clearStickyFaults );
            }  
        }
        final ErrorCode configFactoryDefault = talon.configFactoryDefault();
        if ( configFactoryDefault != ErrorCode.OK ) {
            mLogger.error( "Could not factory reset TalonSRX [{}] due to EC: [{}]", talon.getDeviceID(), configFactoryDefault );
        }  

        final ErrorCode configVoltageCompSaturation = talon.configVoltageCompSaturation( 12.0, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( configVoltageCompSaturation != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] voltage compensation due to EC: [{}]", talon.getDeviceID(), configVoltageCompSaturation );
                }  
        talon.enableVoltageCompensation( true );
    }

    /**
    * This method is intended to initialize the motor controller for using the CTRE Mag Encoder for feeback.
    * <p> <ul>
    * <li>Set the sensor to CTRE Mag Encoder (relative)
    * <li>Set velocity measurement period to 50ms
    * <li>Set velocity measuremeent window to 1
    * <li>Set closed-loop ramp-rate to 0   
    * </ul><p>
    *
    * @param talon WPI_TalonSRX The motor controller to configure
    */  
    public static void CTREMagEncoderConfig ( WPI_TalonSRX talon ) {
        final ErrorCode configSelectedFeedbackSensor = talon.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100 );
        if ( configSelectedFeedbackSensor != ErrorCode.OK ) {
            mLogger.error( "Could not detect encoder due EC: [{}]", configSelectedFeedbackSensor );
        }
        final ErrorCode configVelocityMeasurementPeriod = talon.configVelocityMeasurementPeriod( VelocityMeasPeriod.Period_50Ms, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( configVelocityMeasurementPeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] voltage compensation due to EC: [{}]", talon.getDeviceID(), configVelocityMeasurementPeriod );
        }
        final ErrorCode configVelocityMeasurementWindow = talon.configVelocityMeasurementWindow( 1, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( configVelocityMeasurementWindow != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] velocity measurement window due to EC: [{}]", talon.getDeviceID(), configVelocityMeasurementWindow );
        }
        final ErrorCode configClosedloopRamp = talon.configClosedloopRamp( 0.0, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( configClosedloopRamp != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] closed loop ramp due to EC: [{}]", talon.getDeviceID(), configClosedloopRamp );
        }
    }

    /**
    * Configures a Talon SRX motor controller to be a master.
    *
    * @param talon WPI_TalonSRX The motor controller to configure
    */  
    public static void ConfigureTalonSRX ( WPI_TalonSRX talon ) {
        CommonConfig( talon );
        final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod( StatusFrameEnhanced.Status_2_Feedback0, 20, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( setStatusFramePeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod );
        }  
        talon.set( ControlMode.PercentOutput, 0.0 );
        mLogger.info( "Configured master TalonSRX [{}]", talon.getDeviceID() );
    }

    /**
    * Configures a Talon SRX motor controller to be a follower.
    *
    * @param talon WPI_TalonSRX The motor controller to configure
    * @param master WPI_TalonSRX The motor controller to follow 
    */  
    public static void ConfigureTalonSRX ( WPI_TalonSRX talon, WPI_TalonSRX master ) {
        CommonConfig( talon );
        final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod( StatusFrameEnhanced.Status_2_Feedback0, 160, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( setStatusFramePeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod );
        }  
        talon.follow( master);
        mLogger.info( "Configured follower TalonSRX [{}], master [{}]", talon.getDeviceID(), master.getDeviceID() );
    }

    /**
    * Configures a Talon SRX motor controller to be a follower.
    *
    * @param talon WPI_TalonSRX The motor controller to configure
    * @param master WPI_VictorSPX The motor controller to follow
    */  
    public static void ConfigureTalonSRX ( WPI_TalonSRX talon, WPI_VictorSPX master ) {
        CommonConfig( talon );
        final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod( StatusFrameEnhanced.Status_2_Feedback0, 20, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( setStatusFramePeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod );
        }  
        talon.follow( master);
        mLogger.info( "Configured follower TalonSRX [{}], master [{}]", talon.getDeviceID(), master.getDeviceID() );
    }

}