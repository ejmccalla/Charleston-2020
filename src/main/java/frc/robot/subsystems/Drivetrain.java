package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.lib.drivers.TalonSRX;
import frc.robot.lib.drivers.VictorSPX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {

    // Driver controller state
    public static enum driverMode_t {
        kXbox {
            @Override
            public String toString() {
                return "XBox";
            }
        },
        kJoysticks {
            @Override
            public String toString() {
                return "Joysticks";
            }
        };
    }

    // Hardware
    private final WPI_TalonSRX mLeftMaster;
    private final WPI_VictorSPX mLeftFollower_1;
    private final WPI_VictorSPX mLeftFollower_2;
    private final WPI_TalonSRX mRightMaster;
    private final WPI_VictorSPX mRightFollower_1;
    private final WPI_VictorSPX mRightFollower_2;
    private final DoubleSolenoid mShifter;
    public final DifferentialDrive mDifferentialDrive;

    // Hardware states
    private boolean mIsReversed;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private driverMode_t mDriverMode; 

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Drivetrain.class );


    public void SetReversed ( boolean wantsReversed ) {
        if ( wantsReversed != mIsReversed ) {
            mIsReversed = wantsReversed;
            mLeftMaster.setInverted( !wantsReversed );
            mLeftFollower_1.setInverted( !wantsReversed );
            mLeftFollower_2.setInverted( !wantsReversed );
            mRightMaster.setInverted( !wantsReversed) ;
            mRightFollower_1.setInverted( !wantsReversed );
            mRightFollower_2.setInverted( !wantsReversed );
            mLogger.info( "Reversed drive set to: [{}]", mIsReversed );
        }
    }
    public boolean IsReversed () {
        return mIsReversed;
    }

    public void SetHighGear ( boolean wantsHighGear ) {
        if ( wantsHighGear && !mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kForward );
            mLogger.info( "Gear set to: [High]" );
        } else if ( !wantsHighGear && mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kReverse );
            mLogger.info( "Gear set to: [Low]" );
        }
    }
    public boolean IsHighGear () {
        return mIsHighGear;
    }

    public void SetBrakeMode ( boolean wantsBrakeMode ) {
        if (wantsBrakeMode && !mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mLeftMaster.setNeutralMode( NeutralMode.Brake );
            mLeftFollower_1.setNeutralMode( NeutralMode.Brake );
            mLeftFollower_2.setNeutralMode( NeutralMode.Brake );
            mRightMaster.setNeutralMode( NeutralMode.Brake );
            mRightFollower_1.setNeutralMode( NeutralMode.Brake );
            mRightFollower_2.setNeutralMode( NeutralMode.Brake );
            mLogger.info( "Neutral mode set to: [Brake]" );

        } else if (!wantsBrakeMode && mIsBrakeMode) {
            mIsBrakeMode = wantsBrakeMode;
            mLeftMaster.setNeutralMode( NeutralMode.Coast );
            mLeftFollower_1.setNeutralMode( NeutralMode.Coast );
            mLeftFollower_2.setNeutralMode( NeutralMode.Coast );
            mRightMaster.setNeutralMode( NeutralMode.Coast );
            mRightFollower_1.setNeutralMode( NeutralMode.Coast );
            mRightFollower_2.setNeutralMode( NeutralMode.Coast );
            mLogger.info( "Neutral mode set to: [Coast]" );
        }
    }
    public boolean IsBrakeMode () {
        return mIsBrakeMode;
    }

    public void SetDriverControlMode ( driverMode_t driverMode ) {
        mDriverMode = driverMode;
        mLogger.info( "Drive control mode set to: [{}]", mDriverMode.toString() );
    }
    public boolean IsDriverControlModeXbox () {
        return mDriverMode == driverMode_t.kXbox;
    }


    private void SetSmartDashboard () {
        if ( IsHighGear() ) {
          SmartDashboard.putString( "Gear", "High-Speed" );
        } else {
          SmartDashboard.putString( "Gear", "Low-Speed" );
        }

        if ( IsBrakeMode() ) {
            SmartDashboard.putString( "Neutral Mode", "Brake" );
        } else {
            SmartDashboard.putString( "Neutral Mode", "Coast" );
        }

        if( IsReversed() ) {
            SmartDashboard.putString( "Reversed Mode", "True" );
        } else {
            SmartDashboard.putString( "Reversed Mode", "False" );
        }
    }

    public Drivetrain ( WPI_TalonSRX leftMaster, WPI_VictorSPX leftFollower_1, WPI_VictorSPX leftFollower_2,
                        WPI_TalonSRX rightMaster, WPI_VictorSPX rightFollower_1, WPI_VictorSPX rightFollower_2,
                        DoubleSolenoid shifter ) {
  
        // Set the hardware
        mLeftMaster = leftMaster;
        mLeftFollower_1 = leftFollower_1; 
        mLeftFollower_2 = leftFollower_2;
        mRightMaster = rightMaster; 
        mRightFollower_1 = rightFollower_1;
        mRightFollower_2 = rightFollower_2;
        mShifter = shifter;

        // Create differential drive object
        mDifferentialDrive = new DifferentialDrive( leftMaster, rightMaster );

        // Set the hardware states
        mIsHighGear = false;
        SetHighGear( true );
        mIsBrakeMode = true;
        SetBrakeMode( false );
        mIsReversed = true;
        SetReversed( false );
        //SetDriverControlMode( driverMode_t.kXbox );
        SetDriverControlMode( driverMode_t.kJoysticks );
    }

    public static Drivetrain create () {
        // Talon's and Victor's go through a custom wrapper for creation
        WPI_TalonSRX leftMaster = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( Constants.DRIVETRAIN_LEFT_MASTER_ID) );
        WPI_VictorSPX leftFollower_1 = VictorSPX.createVictorSPX( new WPI_VictorSPX( Constants.DRIVETRAIN_LEFT_FOLLOWER_1_ID), leftMaster );
        WPI_VictorSPX leftFollower_2 = VictorSPX.createVictorSPX( new WPI_VictorSPX( Constants.DRIVETRAIN_LEFT_FOLLOWER_2_ID), leftMaster );
        WPI_TalonSRX rightMaster = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( Constants.DRIVETRAIN_RIGHT_MASTER_ID) );
        WPI_VictorSPX rightFollower_1 = VictorSPX.createVictorSPX( new WPI_VictorSPX( Constants.DRIVETRAIN_RIGHT_FOLLOWER_1_ID), rightMaster );
        WPI_VictorSPX rightFollower_2 = VictorSPX.createVictorSPX( new WPI_VictorSPX( Constants.DRIVETRAIN_RIGHT_FOLLOWER_2_ID), rightMaster );
        DoubleSolenoid shifter = new DoubleSolenoid( Constants.PCM_ID, Constants.DRIVETRAIN_HIGH_GEAR_SOLENOID_ID, Constants.DRIVETRAIN_LOW_GEAR_SOLENOID_ID );
        return new Drivetrain( leftMaster, leftFollower_1, leftFollower_2, rightMaster, rightFollower_1, rightFollower_2, shifter );
    }

    @Override
    public void periodic () {
        SetSmartDashboard();
    }

}
