package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HARDWARE;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.lib.drivers.TalonSRX;
import frc.robot.lib.drivers.VictorSPX;
import frc.robot.lib.controllers.LimelightVision;
import frc.robot.lib.controllers.LimelightVision.SharedState;

/**
* The Drivetrain class is designed to use the command-based programming model and extends the SubsystemBase class.
* @see {@link edu.wpi.first.wpilibj2.command.SubsystemBase}
*/
public class Drivetrain extends SubsystemBase {

    // Hardware
    private final WPI_TalonSRX mLeftMaster;
    private final WPI_VictorSPX mLeftFollower_1;
    private final WPI_VictorSPX mLeftFollower_2;
    private final WPI_TalonSRX mRightMaster;
    private final WPI_VictorSPX mRightFollower_1;
    private final WPI_VictorSPX mRightFollower_2;
    private final DoubleSolenoid mShifter;

    // Drive conrol (both open and closed loop)
    private DifferentialDrive mDifferentialDrive;

    // Limelight Controller Closed-loop control
    private Notifier mLimelightVisionControllerThread;  // Threading interface only
    private LimelightVision mLimelightVisionController;
    private SharedState mLimelightVisionControllerSharedState;

    // State variables
    private boolean mIsReversedDirection;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;

    // Logging data
    public final String mLoggingHeader = "Current Command,Desired Command,Targeting State,Failing State, " +
                                         "Found Target,On Target Turn,On Target Distance,Output Turn, " +
                                         "Output Distance,dt (s),Error Turn (deg),Error Turn (deg/s),Error Turn Total " +
                                         "(deg),Target Distance (ft),Error Distance (ft)";
    public class LoggingData {
        public SharedState mLimelightVisionSharedState;
        public LoggingData ( SharedState limelightVisionSharedState ) {
            mLimelightVisionSharedState = limelightVisionSharedState;
        }
    }
    LoggingData mLoggingData;


    //-----------------------------------------------------------------------------------------------------------------
    /*                                              PUBLIC API METHODS                                               */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This method will set determine the direction of the drive motors.
    *
    * @param wantsReversed boolean True if the driver wants the direction reversed, false otherwise
    */ 
    public void SetReversedDirection ( boolean wantsReversedDirection ) {
        if ( wantsReversedDirection != mIsReversedDirection ) {
            mIsReversedDirection = wantsReversedDirection;
            mLeftMaster.setInverted( !mIsReversedDirection );
            mLeftFollower_1.setInverted( !mIsReversedDirection );
            mLeftFollower_2.setInverted( !mIsReversedDirection );
            mRightMaster.setInverted( !mIsReversedDirection) ;
            mRightFollower_1.setInverted( !mIsReversedDirection );
            mRightFollower_2.setInverted( !mIsReversedDirection );

        }
    }

    /**
    * This method will set the gearing of the transmission.
    *
    * @param wantsHighGear boolean True if the driver wants high gear, false for low gear
    */ 
    public void SetHighGear ( boolean wantsHighGear ) {
        if ( wantsHighGear && !mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kForward );

        } else if ( !wantsHighGear && mIsHighGear ) {
            mIsHighGear = wantsHighGear;
            mShifter.set( DoubleSolenoid.Value.kReverse );

        }
    }

    /**
    * This method will set the neutral mode of the motor controllers.
    *
    * @param wantsBrake boolean True if the driver wants brake, false for coast
    */ 
    public void SetBrakeMode ( boolean wantsBrake ) {
        if ( wantsBrake && !mIsBrakeMode ) {
            mIsBrakeMode = wantsBrake;
            mLeftMaster.setNeutralMode( NeutralMode.Brake );
            mLeftFollower_1.setNeutralMode( NeutralMode.Brake );
            mLeftFollower_2.setNeutralMode( NeutralMode.Brake );
            mRightMaster.setNeutralMode( NeutralMode.Brake );
            mRightFollower_1.setNeutralMode( NeutralMode.Brake );
            mRightFollower_2.setNeutralMode( NeutralMode.Brake );

        } else if ( !wantsBrake && mIsBrakeMode ) {
            mIsBrakeMode = wantsBrake;
            mLeftMaster.setNeutralMode( NeutralMode.Coast );
            mLeftFollower_1.setNeutralMode( NeutralMode.Coast );
            mLeftFollower_2.setNeutralMode( NeutralMode.Coast );
            mRightMaster.setNeutralMode( NeutralMode.Coast );
            mRightFollower_1.setNeutralMode( NeutralMode.Coast );
            mRightFollower_2.setNeutralMode( NeutralMode.Coast );

        }
    }

    /**
    * This method will return the state of the reversed mode 
    *
    * @return boolean True if the direction is reversed, false otherwise
    */
    public boolean IsReversedDirection () {
        return mIsReversedDirection;
    }

    /**
    * This method will return the state of the shifting transmission 
    *
    * @return boolean True if the transmission is in high-gear, false for low-gear
    */
    public boolean IsHighGear () {
        return mIsHighGear;
    }

    /**
    * This method will return the state of motor controllers neutral mode 
    *
    * @return boolean True for brake, false for coast
    */
    public boolean IsBrakeMode () {
        return mIsBrakeMode;
    }

    /**
    * This method will set the Limelight vision controller to perform the turn-to-target command.
    */
    public void StartTurnToTarget () {
        mLimelightVisionController.TurnToTarget();
    }

    /**
    * This method will set the Limelight vision controller to idle.
    */
    public void EndTurnToTarget () {
        mLimelightVisionController.Idle();
    }

    /**
    * This method will set the Limelight vision controller's output to the curvature drives turning input and set the
    * quickturn flag in order to get the robot to turn towards the target (taking into account the reversed direction
    * state).
    */
    public void SetLimelightVisionControllerOutput () {
        if ( mIsReversedDirection ) {
            mDifferentialDrive.curvatureDrive( 0.0, -mLimelightVisionControllerSharedState.outputTurn, true );
        } else {
            mDifferentialDrive.curvatureDrive( 0.0, mLimelightVisionControllerSharedState.outputTurn, true );
        }
    }

    /**
    * This method will set the Limelight vision controller's output to the curvature drives turning input and set and
    * use input as the throttle.
    */
    public void SetLimelightVisionControllerOutput ( double throttle, boolean quickTurn ) {
        if ( mIsReversedDirection ) {
            mDifferentialDrive.curvatureDrive( throttle, -mLimelightVisionControllerSharedState.outputTurn, quickTurn );
        } else {
            mDifferentialDrive.curvatureDrive( throttle, mLimelightVisionControllerSharedState.outputTurn, quickTurn );
        }
    }

    /**
    * This method will set the output based on the driver inputs and the reversed direction state.
    */
    public void SetOpenLoopOutput ( double throttle, double turn, boolean quickTurn ) {
        if ( mIsReversedDirection ) {
            mDifferentialDrive.curvatureDrive( throttle, -turn, quickTurn );
        } else {
            mDifferentialDrive.curvatureDrive( throttle, turn, quickTurn );
        }
    }

    /**
    * This method will output data to the smart dashboard.  This data is displayed at the driver's station and is meant
    * to be aid the drive team.
    */
    public void OutputSmartDashboard () {
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

        if( IsReversedDirection() ) {
            SmartDashboard.putString( "Reversed Mode", "True" );
        } else {
            SmartDashboard.putString( "Reversed Mode", "False" );
        }
        SmartDashboard.putBoolean( "Found Target", mLimelightVisionControllerSharedState.foundTarget );
        SmartDashboard.putBoolean( "On Target", mLimelightVisionControllerSharedState.onTargetTurn );
        SmartDashboard.putString( "Desired Command", mLimelightVisionControllerSharedState.desiredCommand.toString() );
        SmartDashboard.putString( "Running Command", mLimelightVisionControllerSharedState.currentCommand.toString() );
        SmartDashboard.putString( "Vision State", mLimelightVisionControllerSharedState.visionState.toString() );
        SmartDashboard.putString( "Failing State", mLimelightVisionControllerSharedState.failState.toString() );
    }

    /**
    * This method will return all of the logging data 
    *
    * @return LoggingData A class holding all of the logging data
    */
    public LoggingData GetLoggingData () {
        return mLoggingData;
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                                PRIVATE METHODS                                                */
    //-----------------------------------------------------------------------------------------------------------------
    

    /**
    * This method will initialize the Drivetrain subsystem.
    */
    private void Initialize () {
        ResetState();
        ResetSensors();
        mLimelightVisionControllerSharedState = mLimelightVisionController.GetSharedState();
        mLoggingData = new LoggingData( mLimelightVisionControllerSharedState );
    }

    /**
    * This method will reset senor and vision controller information.
    */ 
    private void ResetSensors () {
    }

    /**
    * This method will reset all of the internal states.
    */ 
    private void ResetState () {
        mIsHighGear = false;
        SetHighGear( true );
        mIsBrakeMode = false;
        SetBrakeMode( true );
        mIsReversedDirection = true;
        SetReversedDirection( false );
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                        CLASS CONSTRUCTOR AND OVERRIDES                                        */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * The constructor for the Drivetrain class.
    *
    * @param leftMaster WPI_TalonSRX A Talon SRX motor controller object
    * @param leftFollower_1 WPI_VictorSPX A Talon SRX motor controller object
    * @param leftFollower_2 WPI_VictorSPX A Talon SRX motor controller object
    * @param rightMaster WPI_TalonSRX A Talon SRX motor controller object
    * @param rightFollower_1 WPI_VictorSPX A Talon SRX motor controller object
    * @param rightFollower_2 WPI_VictorSPX A Talon SRX motor controller object
    * @param rightFollower_2 WPI_VictorSPX A Talon SRX motor controller object
    * @param shifter DoubleSolenoid A double solenoid object for shifting the transmission
    */  
    public Drivetrain ( WPI_TalonSRX leftMaster, WPI_VictorSPX leftFollower_1, WPI_VictorSPX leftFollower_2,
                        WPI_TalonSRX rightMaster, WPI_VictorSPX rightFollower_1, WPI_VictorSPX rightFollower_2,
                        DifferentialDrive differentialDrive, LimelightVision limelightVision, DoubleSolenoid shifter ) {
        mLeftMaster = leftMaster;
        mLeftFollower_1 = leftFollower_1; 
        mLeftFollower_2 = leftFollower_2;
        mRightMaster = rightMaster; 
        mRightFollower_1 = rightFollower_1;
        mRightFollower_2 = rightFollower_2;
        mShifter = shifter;
        mDifferentialDrive = differentialDrive;
        mLimelightVisionController = limelightVision;
        if ( DRIVETRAIN.VISION_THREADED ) {
            mLimelightVisionControllerThread = new Notifier (mLimelightVisionController.mThread); 
            mLimelightVisionControllerThread.startPeriodic( 0.01 );
        }
        Initialize();
    }

    /**
    * This mehtod will create a new Drivetrain object.  The purpose of doing the constructor this way is to allow for
    * unit testing.
    */  
    public static Drivetrain Create () {
        WPI_TalonSRX leftMaster = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( DRIVETRAIN.LEFT_MASTER_ID) );
        WPI_VictorSPX leftFollower_1 = VictorSPX.createVictorSPX( new WPI_VictorSPX( DRIVETRAIN.LEFT_FOLLOWER_1_ID),
                                                                                     leftMaster );
        WPI_VictorSPX leftFollower_2 = VictorSPX.createVictorSPX( new WPI_VictorSPX( DRIVETRAIN.LEFT_FOLLOWER_2_ID),
                                                                                     leftMaster );
        WPI_TalonSRX rightMaster = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( DRIVETRAIN.RIGHT_MASTER_ID) );
        WPI_VictorSPX rightFollower_1 = VictorSPX.createVictorSPX( new WPI_VictorSPX( DRIVETRAIN.RIGHT_FOLLOWER_1_ID),
                                                                                      rightMaster );
        WPI_VictorSPX rightFollower_2 = VictorSPX.createVictorSPX( new WPI_VictorSPX( DRIVETRAIN.RIGHT_FOLLOWER_2_ID),
                                                                                      rightMaster );
        DoubleSolenoid shifter = new DoubleSolenoid( HARDWARE.PCM_ID, DRIVETRAIN.HIGH_GEAR_SOLENOID_ID, 
                                                     DRIVETRAIN.LOW_GEAR_SOLENOID_ID );
        DifferentialDrive differentialDrive = new DifferentialDrive( leftMaster, rightMaster );
        LimelightVision limelightVision = LimelightVision.Create( DRIVETRAIN.VISION_SEARCH_TIMEOUT_S,
                                                                  DRIVETRAIN.VISION_SEEK_TIMEOUT_S,
                                                                  DRIVETRAIN.VISION_SEEK_RETRY_LIMIT,
                                                                  DRIVETRAIN.VISION_TURN_PID_KP,
                                                                  DRIVETRAIN.VISION_TURN_PID_KI,
                                                                  DRIVETRAIN.VISION_TURN_PID_KD,
                                                                  DRIVETRAIN.VISION_TURN_PID_KF,
                                                                  DRIVETRAIN.VISION_ON_TARGET_TURN_THRESHOLD_DEG,
                                                                  DRIVETRAIN.VISION_ON_TARGET_DISTANCE_THRESHOLD_FT,
                                                                  DRIVETRAIN.VISION_TARGET_WIDTH_FT,
                                                                  DRIVETRAIN.VISION_FOCAL_LENGTH_FT );

        return new Drivetrain( leftMaster, leftFollower_1, leftFollower_2, rightMaster, rightFollower_1,
                               rightFollower_2, differentialDrive, limelightVision, shifter );
    }

    /**
    * The subsystem periodic method gets called by the CommandScheduler at the very beginning of each robot loop.  All
    * sensor readings should be updated in this method.
    * @see {@link edu.wpi.first.wpilibj2.command.CommandScheduler#run}
    */ 
    @Override
    public void periodic () {
        if ( !DRIVETRAIN.VISION_THREADED ) {
            mLimelightVisionController.RunUpdate();
        }
        mLimelightVisionControllerSharedState = mLimelightVisionController.GetSharedState();
        mLoggingData.mLimelightVisionSharedState = mLimelightVisionControllerSharedState;
    }

}
