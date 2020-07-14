package frc.robot.lib.drivers;

import frc.robot.Constants.HARDWARE;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* This class provides methods to configure Victor SPX motor controllers as both masters and followers.  Also, any
* sticky errors present will be logged and cleared.
*
* @see {@link https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html}
*/
public class VictorSPX {

    private static final Logger mLogger = LoggerFactory.getLogger( VictorSPX.class );

    /**
    * This method is intended to initialize the motor controller to default configuration common to all modes or
    * operation.
    * <p><ul>
    * <li>Sticky faults will be logged and cleared
    * <li>Reset to factory defaults
    * <li>Voltage compensation is enabled and set for 12V
    * </ul>
    *
    * @param victor WPI_VictorSPX The motor controller to initialize
    */
    private static void CommonConfig ( WPI_VictorSPX victor ) {
        StickyFaults faults = new StickyFaults();

        victor.getStickyFaults( faults );
        if ( faults.hasAnyFault() ) {
            mLogger.warn( "Clearing VictorSPX [{}] sticky faults: [{}]", victor.getDeviceID(), faults.toString() );
            final ErrorCode clearStickyFaults = victor.clearStickyFaults( HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
            if ( clearStickyFaults != ErrorCode.OK ) {
                mLogger.error( "Could not clear sticky faults due to EC: [{}]", clearStickyFaults );
            }  
        }
    
        final ErrorCode configFactoryDefault = victor.configFactoryDefault();
        if ( configFactoryDefault != ErrorCode.OK ) {
            mLogger.error( "Could not factory reset VictorSPX [{}] due to EC: [{}]", victor.getDeviceID(), configFactoryDefault );
        }  

        final ErrorCode configVoltageCompSaturation = victor.configVoltageCompSaturation( 12.0, HARDWARE.CTRE_CAN_LONG_TIMEOUT_MS );
        if ( configVoltageCompSaturation != ErrorCode.OK ) {
            mLogger.error( "Could not set VictorSPX [{}] voltage compensation due to EC: [{}]", victor.getDeviceID(), configVoltageCompSaturation );
        }  
        victor.enableVoltageCompensation( true );
    }

    /**
    * Configures a Victor SPX motor controller to be a master.
    *
    * @param victor WPI_VictorSPX The motor controller to configure
    */  
    public static void ConfigureVictorSPX ( WPI_VictorSPX victor ) {
        CommonConfig( victor );
        victor.set( ControlMode.PercentOutput, 0.0 );
        mLogger.info(" Configured leader VictorSPX [{}]", victor.getDeviceID() );
    }
  
    /**
    * Configures a Victor SPX motor controller to be a follower.
    *
    * @param victor WPI_VictorSPX The Victor SPX motor controller to configure 
    * @param master WPI_TalonSRX The Talon SRX motor controller to follow 
    */ 
    public static void ConfigureVictorSPX ( WPI_VictorSPX victor, WPI_TalonSRX master ) {
        CommonConfig( victor );
        victor.follow( master );
        mLogger.info( "Configured follower VictorSPX [{}], master [{}]", victor.getDeviceID(), master.getDeviceID() );
    }

    /**
    * Configures a Victor SPX motor controller to be a follower.
    *
    * @param victor WPI_VictorSPX The Victor SPX motor controller to configure 
    * @param master WPI_VictorSPX The Victor SPX motor controller to follow 
    */ 
    public static void ConfigureVictorSPX ( WPI_VictorSPX victor, WPI_VictorSPX master ) {
        CommonConfig( victor );
        victor.follow( master );
        mLogger.info( "Configured follower VictorSPX [{}], master [{}]", victor.getDeviceID(), master.getDeviceID() );
    }

}
