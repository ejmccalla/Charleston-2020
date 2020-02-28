package frc.robot.lib.controllers;

import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.drivers.Limelight;

/**
* The LimelightVision class implements control algorithms using vision as a direct feedback sensor or setpoint generator.
*/
public class LimelightVision implements Sendable, AutoCloseable {

    // Count class instances
    private static int mInstances;

    // State enumerations
    private enum VisionState_t {
        Searching { @Override public String toString() { return "Searching"; } },
        Seeking { @Override public String toString() { return "Seeking"; } },
        Tracking { @Override public String toString() { return "Tracking"; } },
        Failing { @Override public String toString() { return "Fail"; } };
    }

    private enum CommandState_t {
        Idle { @Override public String toString() { return "Idle"; } },
        TurnToTarget { @Override public String toString() { return "Turn-to-Target"; } }
    }

    private enum FailingState_t {
        TargetNotFound { @Override public String toString() { return "Target Search Timeout"; } },
        SeekTimeout { @Override public String toString() { return "Seek Timeout"; } },
        ExhaustedSeekRetries { @Override public String toString() { return "Exhausted Seek Retries"; } },
        LostTarget { @Override public String toString() { return "Lost Target"; } };
    }

    // Hardware
    private final Limelight mLimelight;
    private double mTargetSize;
    private double mFocalLength;

    // Closed-loop control
    private double mP, mI, mD, mF; //
    private double mSearchTimeout_S;
    private double mSeekTimeout_S;
    private int mSeekRetryLimit;
    private double mMinIntegral, mMaxIntegral; //
    private double mOnTargetErrorThreshold;
    private double mLastUpdateTime, mdeltaTime;
    private double mCurrentErrorXPosition;
    private double mCurrentErrorXVelocity;
    private double mPrevErrorXPosition;
    private double mTotalErrorXPosition;
    private double mOutput;
    private double mDistanceToTarget;

    // State variables
    private CommandState_t mCommandState;
    private VisionState_t mVisionState;
    private FailingState_t mFailState;
    private boolean mOnTarget;
    private boolean mFoundTarget;
    private double mSeekTimer_S; //
    private int mSeekRetries; //



    //-----------------------------------------------------------------------------------------------------------------
    /*                                              PUBLIC API METHODS                                               */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This method will setup the turn-to-target comamand by first resetting the PID calculations and internal state.
    * The command state is updated to run the command.
    */
    public void TurnToTarget () {
        ResetCalculations();
        ResetState();
        mCommandState = CommandState_t.TurnToTarget;
    }

    /**
    * This method will end any commands and move to the Idle command state.
    */
    public void SetIdle () {
        mCommandState = CommandState_t.Idle;
    }

    /**
    * This method will update the limelight vision output and the state of the internal state of the vision controller.
    * <p>
    * The Turn-to-Target command will attempt to drive the Limelight X-position to the given target position.  During
    * this process the controller will be in 1 of 3 states: Seeking, Tracking, Fail.
    * The Idle command literally does nothing and only exists for logging purposes.
    * <p>
    * The seeking state is the initial state when the command begins.  If we get lucky and happen to be be on-target,
    * the state is updated to tracking and the output is set to the PID calculation.  If we aren't on-target but do
    * have a target identified, then use the PID loop to calculate the output and continue seeking to the target.  If
    * there's no target identified then use the feed-forward term to "search" for a target and update the state to
    * "Searching".  If the seek timer expires or the seek retries are exhausted, the state is updated to failing and
    * the failure will be handled by the logic in the failing state.  Also, the output is set to 0.0.
    * <p>
    * The tracking state means that the error in the position is withing the given range and we are setting the output
    * to maintain, or further decrease, the error.  If the target is no longer within the specified range, then the
    * controller will simply fall back to the seeking state.  This is expected to be unlikely event when things are
    * static, but may be more common for dynamic systems (like a vision controlled turret with a moving robot).  If the
    * target has been lost, update to the failing state, and set the ouptut at 0.0.
    * <p>
    * The searching state means that the controller has yet to identity a target and is "searchin" for a target by
    * sending the feed-forward term as the output.  The conroller will continue to do this until either 1: a target is
    * identified and the controller moves to the seeking state, or 2: the controller times out and moves to the failing
    * state. 
    * <p>
    * The failing state means that one of the following errors was encountered: the seek timed out, there were too many
    * seek retries, the target was lost during tracking, or a target was never found.  If the target was lost while
    * tracking, we can assume there was either a sudden and significant change in the limelight pose, or, the tracking
    * threshold is wide open (nearing the horizontal FOV).  Either way, the controller will not try and recover from
    * this and will leave it to the caller to issue the command again.  A seek timout could happen because the timeout
    * threshold is too low, the controller took too long to get to the target, the tracking threshold is too low, or
    * one of several other cases.  In any event, it doesn't hurt to report the issue and retry the seek.  Mostly, this
    * mechanism is in place to give rise to a change in system behaviour (like seeks used to happen fast, now they're
    * timing out once or twice before reaching the target...time to look into performance).  Finally, if the Limelight
    * was never able to identify a target and timed out looking.  The user will have to issue the command again to
    * continue the search or consider increasing the search timeout threshold.
    *
    * @return double The ouptut of the limelight vision
    */     
    public double GetUpdate () {
        double currentTime = Timer.getFPGATimestamp();

        mdeltaTime = currentTime - mLastUpdateTime;
        mLastUpdateTime = currentTime;
        Calculate( mdeltaTime );

        switch ( mCommandState ) {
            case TurnToTarget:
                switch ( mVisionState ) {
                    case Seeking:
                        if ( mOnTarget ) {
                            mOutput = mP * mCurrentErrorXPosition + mI * mTotalErrorXPosition + mD * mCurrentErrorXVelocity;
                            mVisionState = VisionState_t.Tracking;

                        } else if ( currentTime - mSeekTimer_S > mSeekTimeout_S ) {
                            mOutput = 0.0;
                            mVisionState = VisionState_t.Failing;
                            mSeekRetries += 1;
                            if ( mSeekRetries > mSeekRetryLimit ) {
                                mFailState = FailingState_t.ExhaustedSeekRetries;
                            } else {
                                mFailState = FailingState_t.SeekTimeout;
                            }
                                
                        } else if ( mFoundTarget ) {
                            mOutput = mP * mCurrentErrorXPosition + mI * mTotalErrorXPosition + mD * mCurrentErrorXVelocity;
                        
                        } else {
                            mOutput = mF;
                            mVisionState = VisionState_t.Searching;

                        }
                        break;

                    case Tracking:
                        if ( mOnTarget && mFoundTarget ) {
                            mOutput = mP * mCurrentErrorXPosition + mI * mTotalErrorXPosition + mD * mCurrentErrorXVelocity;
                        
                        } else if ( !mOnTarget && mFoundTarget ) {
                            mOutput = mP * mCurrentErrorXPosition + mI * mTotalErrorXPosition + mD * mCurrentErrorXVelocity;
                            mVisionState = VisionState_t.Seeking;
                            mSeekRetries = 0;
                            SetSeekTimer();
    
                        } else {
                            mOutput = 0.0;
                            mVisionState = VisionState_t.Failing;
                            mFailState = FailingState_t.LostTarget;
                        }
                        break;
                
                    case Searching:
                        if ( currentTime - mSeekTimer_S > mSearchTimeout_S ) {
                            mOutput = 0.0;
                            mFailState = FailingState_t.TargetNotFound;

                        } else if ( mFoundTarget ) {
                            mOutput = mP * mCurrentErrorXPosition + mI * mTotalErrorXPosition + mD * mCurrentErrorXVelocity;
                            mVisionState = VisionState_t.Seeking;
                            mSeekRetries = 0;
                            SetSeekTimer();
                        
                        } else {
                            mOutput = mF;
                        }
                        break;

                    case Failing:
                        switch ( mFailState ) {
                            case LostTarget:
                                DriverStation.reportError( "Limelight lost target", false );
                                mOutput = 0.0;
                                mCommandState = CommandState_t.Idle;
                                break;

                            case SeekTimeout:
                                DriverStation.reportWarning( "Limelight seek timeout", false );
                                mVisionState = VisionState_t.Seeking;
                                mFailState = null;
                                SetSeekTimer();
                                break;

                            case ExhaustedSeekRetries:
                                DriverStation.reportError( "Limelight exhausted seek retries", false );
                                mOutput = 0.0;
                                mCommandState = CommandState_t.Idle;
                                break;

                            case TargetNotFound:
                                DriverStation.reportError( "Limelight couldn't find target", false );
                                mOutput = 0.0;
                                mCommandState = CommandState_t.Idle;
                                break;

                            default:
                                DriverStation.reportWarning( "Limelight failure is unhandled", false );

                        }
                    break;
                }
                break;

            case Idle:
                mOutput = 0.0;
                break;

        }
        return mOutput;
    }

    /**
    * This method will return whether or not the limelight sees a target or not
    *
    * @return boolean True if the limelight sees a target, false otherwise
    */
    public boolean GetFoundTarget () {
        return mFoundTarget;
    }

    /**
    * This method will return whether or not the controller is on-target or not
    *
    * @return boolean True if the controller is on-target, false otherwise
    */
    public boolean GetOnTarget () {
        return mOnTarget;
    }

    /**
    * This method will return the current X-position error
    *
    * @return double The current limelight X-position error
    */
    public double GetCurrentErrorXPosition () {
        return mCurrentErrorXPosition;
    }

    /**
    * This method will return the current X-velocity error
    *
    * @return double The current limelight X-velocity error
    */
    public double GetCurrentErrorXVelocity () {
        return mCurrentErrorXVelocity;
    }

    /**
    * This method will return the previous X-position error
    *
    * @return double The previous limelight X-position error
    */
    public double GetPrevErrorXPosition () {
        return mPrevErrorXPosition;
    }

    /**
    * This method will return the total X-position error
    *
    * @return double The previous limelight X-position error
    */
    public double GetTotalErrorXPosition () {
        return mTotalErrorXPosition;
    }

    /**
    * This method will return the command state
    *
    * @return String The current command state
    */
    public String GetCommandState () {
        return mCommandState.toString();
    }

    /**
    * This method will return the controller state
    *
    * @return String The current controller state
    */
    public String GetVisionState () {
        return mVisionState.toString();
    }

    /**
    * This method will return the failing command state
    *
    * @return String The current failing command state
    */
    public String GetFailState () {
        return mFailState.toString();
    }
  
    /**
    * This method will return the estimated distance to the target
    *
    * @return double The current failing command state
    */
    public double GetDistanceToTarget () {
        return mDistanceToTarget;
    }

    
    //-----------------------------------------------------------------------------------------------------------------
    /*                                                PRIVATE METHODS                                                */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This method will initialize the Limelight vision subsystem by setting the closed-loop parameters, resetting all
    * of the PID calculations, and resetting all of the internal states.
    *
    * @param searchTimeout_S double The timeout for the controller to search for target
    * @param seekTimeout_S double The timeout for the controller to get on-target
    * @param seekRetryLimit int The number of seek retries allowed
    * @param P double The proportional gain
    * @param I double The integral gain
    * @param D double The differential gain
    * @param F double The feed-forward gain
    * @param onTargetLimit double The on-target threshold used to differentiate seeking and tracking states
    */
    private void Initialize ( double searchTimeout_S, double seekTimeout_S, int seekRetryLimit, double P, double I,
                              double D, double F, double onTargetLimit, double targetSize, double focalLength ) {
        mSearchTimeout_S = searchTimeout_S;
        mSeekTimeout_S = seekTimeout_S;
        mSeekRetryLimit = seekRetryLimit;
        mP = P;
        mI = I;
        mD = D;
        mF = F;
        mOnTargetErrorThreshold = onTargetLimit;
        mTargetSize = targetSize;
        mFocalLength = focalLength;
        mMinIntegral = -1.0;
        mMaxIntegral = 1.0;
        mLastUpdateTime = 0.0;
        mdeltaTime = 0.0;
        ResetCalculations();
        ResetState();
        mOutput = 0.0;
        mDistanceToTarget = Double.NaN;
    }

    /**
    * This method will updated the Limelight targeting state information and perform the PID error calculations.  The
    * final output is not calculated by this method.  If the Limelight doesn't have a target in sight, then reset all
    * PID error calculations and update the state to reflect that a target hasn't been found.
    *
    * @param dt double The timestep
    */ 
    private void Calculate ( double dt ) {
        if ( mLimelight.GetFoundTarget() ) {
            mFoundTarget = true;
            mPrevErrorXPosition = mCurrentErrorXPosition;
            mCurrentErrorXPosition = mLimelight.GetHorizontalToTargetDeg();
            mCurrentErrorXVelocity = ( mCurrentErrorXPosition - mPrevErrorXPosition ) / dt;
            mTotalErrorXPosition = MathUtil.clamp(mTotalErrorXPosition + mCurrentErrorXPosition * dt, mMinIntegral / mI, mMaxIntegral / mI);
            if ( Math.abs( mCurrentErrorXPosition ) < mOnTargetErrorThreshold ) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }
            mDistanceToTarget = EstimateDistance( mLimelight.GetHorizontalPixels() );
        } else {
            mOnTarget = false;
            mFoundTarget = false;
            ResetCalculations();
            mDistanceToTarget = Double.NaN;
        }
    }

    /**
    * This method will calculate the 
    *
    * @param pixels double The size of the target in pixels
    */ 
    private double EstimateDistance ( double pixels ) {
        return ( mTargetSize * mFocalLength ) / pixels;
    }

    /**
    * This method will reset all of the PID error calculations.
    */ 
    private void ResetCalculations () {
        mCurrentErrorXPosition = 0.0;
        mPrevErrorXPosition = 0.0;
        mTotalErrorXPosition = 0.0;
        mCurrentErrorXVelocity = 0.0;
    }

    /**
    * This method will reset all of the internal states.
    */ 
    private void ResetState () {
        mOnTarget = false;
        mFoundTarget = false;
        mSeekRetries = 0;
        mCommandState = CommandState_t.Idle;
        mVisionState = VisionState_t.Seeking;
        mFailState = null;
        SetSeekTimer();
    }

    /**
    * This method will set the mSeekTimer_S variable to the current time in seconds.
    */
    private void SetSeekTimer () {
        mSeekTimer_S = Timer.getFPGATimestamp();
    }

    /**
    * This method will set the P-gain of the controller.
    *
    * @param p double P-gain
    */  
    private void SetP ( double p ) {
        mP = p;
    }
    
    /**
    * This method will get the P-gain of the controller.
    */  
    private double GetP () {
        return mP;
    }

    /**
    * This method will set the I-gain of the controller.
    *
    * @param i double I-gain
    */  
    private void SetI ( double i ) {
        mI = i;
    }

    /**
    * This method will get the I-gain of the controller.
    */  
    private double GetI () {
        return mI;
    }

    /**
    * This method will set the D-gain of the controller.
    *
    * @param d double D-gain
    */  
    private void SetD ( double d ) {
        mD = d;
    }

    /**
    * This method will get the D-gain of the controller.
    */  
    private double GetD () {
        return mD;
    }

    /**
    * This method will set the feedforward gain of the controller.
    *
    * @param f double Feedforward-gain
    */  
    private void SetF ( double f ) {
        mF = f;
    }

    /**
    * This method will get the feedforward gain of the controller.
    */  
    private double GetF () {
        return mF;
    }

    /**
    * This method will set the tracking threhsold of the controller.
    *
    * @param f double Tracking threshold
    */  
    private void SetTrackingThreshold ( double onTargetErrorThreshold ) {
        mOnTargetErrorThreshold = onTargetErrorThreshold;
    }

    /**
    * This method will get the tracking threshold of the controller.
    */  
    private double GetTrackingThreshold () {
        return mOnTargetErrorThreshold;
    }

    /**
    * This method will get the total output of the controller.
    */      
    private double GetOutput () {
        return mOutput;
    }

    /**
    * This method will get the P-term output of the controller.
    */      
    private double GetPTermOutput () {
        return mP * mCurrentErrorXPosition;
    }

    /**
    * This method will get the I-term output of the controller.
    */      
    private double GetITermOutput () {
        return mI * mTotalErrorXPosition;
    }

    /**
    * This method will get the D-term output of the controller.
    */      
    private double GetDTermOutput () {
        return mD * mCurrentErrorXVelocity;
    }    

    /**
    * This method will get the dt of the controller.
    */      
    private double GetDeltaTime () {
        return mdeltaTime;
    }


    //-----------------------------------------------------------------------------------------------------------------
    /*                                        CLASS CONSTRUCTOR AND OVERRIDES                                        */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * The constructor for the LimelightVision class.
    *
    * @param limelight Limelight The limelight camera object used for this controller
    * @param P double The proportional gain
    * @param I double The integral gain
    * @param D double The differential gain
    * @param F double The feed-forward gain
    * @param onTargetErrorThreshold double The on-target threshold used to differentiate seeking and tracking states
    * @param focalLength double The computed focal length
    */  
    public LimelightVision ( Limelight limelight, double searchTimeout_S, double seekTimeout_S, int seekRetryLimit,
                             double P, double I, double D, double F, double onTargetErrorThreshold, double targetSize,
                             double focalLength ) {
        mLimelight = limelight;
        Initialize (  searchTimeout_S, seekTimeout_S, seekRetryLimit, P, I, D, F, onTargetErrorThreshold, targetSize,
                      focalLength );
        mInstances++;
        SendableRegistry.addLW( this, "LimelightVision", mInstances );
    }
    
    /**
    * This mehtod will create a new LimelightVision object.  The purpose of doing the constructor this way is to allow
    * for unit testing.
    *
    * @param P double The proportional gain
    * @param I double The integral gain
    * @param D double The differential gain
    * @param F double The feed-forward gain
    * @param onTargetErrorThreshold double The on-target threshold used to differentiate seeking and tracking states
    * @param focalLength double The computed focal length
    */  
    public static LimelightVision Create ( double searchTimeout_S, double seekTimeout_S, int seekRetryLimit, double P,
                                           double I, double D, double F, double onTargetErrorThreshold,
                                           double targetSize, double focalLength ) {
        Limelight limelight = new Limelight();
        return new LimelightVision( limelight, searchTimeout_S, seekTimeout_S, seekRetryLimit, P, I, D, F,
                                    onTargetErrorThreshold, targetSize, focalLength );
    }

    /**
    * We are overriding the initSendable and using it to send information back-and-forth between the robot program and
    * the user PC for live PID tuning purposes.
    *
    * @param SendableBuilder This is inherited from SubsystemBase
    */ 
    @Override
    public void initSendable ( SendableBuilder builder ) {
        NetworkTableEntry entryTimeDelta = builder.getEntry("Time Delta");
        NetworkTableEntry entryCurrentErrorXPosition = builder.getEntry("Error Position");
        NetworkTableEntry entryCurrentErrorXVelocity = builder.getEntry("Error Velocity");
        NetworkTableEntry entryTotalErrorXPosition = builder.getEntry("Error Total");
        NetworkTableEntry entryOutput = builder.getEntry("Total Ouput");
        NetworkTableEntry entryPTerm = builder.getEntry("P-Term");
        NetworkTableEntry entryIterm = builder.getEntry("I-Term");
        NetworkTableEntry entryDTerm = builder.getEntry("D-Term");
        builder.setSmartDashboardType( "Vision Turn PID Tuning" );
        builder.addDoubleProperty( "P", this::GetP, this::SetP);
        builder.addDoubleProperty( "I", this::GetI, this::SetI);
        builder.addDoubleProperty( "D", this::GetD, this::SetD);
        builder.addDoubleProperty( "F", this::GetF, this::SetF);
        builder.addDoubleProperty( "Tracking Threshold", this::GetTrackingThreshold, this::SetTrackingThreshold);
        builder.setUpdateTable( () -> { entryTimeDelta.setDouble( GetDeltaTime() );
                                        entryCurrentErrorXPosition.setDouble( GetCurrentErrorXPosition() );
                                        entryCurrentErrorXVelocity.setDouble( GetCurrentErrorXVelocity() );
                                        entryTotalErrorXPosition.setDouble( GetTotalErrorXPosition() );
                                        entryOutput.setDouble( GetOutput() );
                                        entryPTerm.setDouble( GetPTermOutput() );
                                        entryIterm.setDouble( GetITermOutput() );
                                        entryDTerm.setDouble( GetDTermOutput() ); } );
    }

    /**
    * Override the close method from AutoCloseable.
    */ 
    @Override
    public void close() {
      SendableRegistry.remove( this );
    }    

} 
