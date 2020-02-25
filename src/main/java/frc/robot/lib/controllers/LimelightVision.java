package frc.robot.lib.controllers;

import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.drivers.Limelight;

/**
* The LimelightVision class implements control algorithms using vision as a direct feedback sensor or setpoint generator.
*/
public class LimelightVision implements Sendable {

    // State enumerations
    public static enum VisionState_t {
        Seeking { @Override public String toString() { return "Seeking"; } },
        Tracking { @Override public String toString() { return "Tracking"; } },
        Fail { @Override public String toString() { return "Fail"; } };
    }

    public static enum CommandState_t {
        Idle { @Override public String toString() { return "Idle"; } },
        TurnToTarget { @Override public String toString() { return "Turn-to-Target"; } }
    }

    public static enum FailingState_t {
        SeekTimeout { @Override public String toString() { return "Seek Timeout"; } },
        ExhaustedSeekRetries { @Override public String toString() { return "Exhausted Seek Retries"; } },
        LostTarget { @Override public String toString() { return "Lost Target"; } };
    }

    // Hardware
    private final Limelight mLimelight;

    // Closed-loop control
    private double mP, mI, mD, mF;
    private double mMinIntegral, mMaxIntegral;
    private double mTargetXPosition;
    private double mOnTargetLimit;
    private double mCurrentXPosition;
    private double mErrorXPosition;
    private double mPrevErrorXPosition;
    private double mTotalErrorXPosition;
    private double mErrorXVelocity;
    private double mOutput;

    // State variables
    private CommandState_t mCommandState;
    private VisionState_t mVisionState;
    private FailingState_t mFailState;
    private boolean mOnTarget;
    private boolean mFoundTarget;
    private double mSeekTimer_S;
    private int mSeekRetries;

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * This method will setup the turn-to-target comamand by first resetting the PID calculations and internal state.
    * The command state and target are updated also updated to run the command.
    * @param targetXPosition double  Target position for closed-loop position control
    */
    public void TurnToTarget ( double targetXPosition ) {
        ResetCalculations();
        ResetState();
        mCommandState = CommandState_t.TurnToTarget;
        mTargetXPosition = targetXPosition;
    }

    /**
    * This method will update the limelight vision output and the state of the internal state of the vision
    * controller.
    * @return double The ouptut of the limelight vision
    */     
    public double GetUpdate ( double dt ) {
        switch ( mCommandState ) {
            case TurnToTarget:
                Calculate( dt );
                switch ( mVisionState ) {
                    // When a new command comes in, the initial state is seeking.  If we get lucky and happen to be
                    // be on-target, set the state to tracking and the output to the PID calculation.  If we happen
                    // to have a target identified, use the PID loop to calculate the output.  If there's no target
                    // identified then use the feed-forward term to "search" for a target.
                    case Seeking:
                        if ( mOnTarget ) {
                            mOutput = mP * mErrorXPosition + mI * mTotalErrorXPosition + mD * mErrorXVelocity;
                            mVisionState = VisionState_t.Tracking;

                        } else if ( Timer.getFPGATimestamp() - mSeekTimer_S > 1.5 ) {
                            mOutput = 0.0;
                            mVisionState = VisionState_t.Fail;
                            mSeekRetries += 1;
                            if ( mSeekRetries > 100 ) {
                                mFailState = FailingState_t.ExhaustedSeekRetries;
                            } else {
                                mFailState = FailingState_t.SeekTimeout;
                            }
                                
                        } else if ( mFoundTarget ) {
                            mOutput = mP * mErrorXPosition + mI * mTotalErrorXPosition + mD * mErrorXVelocity;
                        
                        } else {
                            mOutput = mF;
                        }
                        break;

                    // If the target is no longer within the specified range, then fall back to seeking.  This is an
                    // unlikely event when things are static, but may be more common for dynamic systems (like a vision
                    // controlled turret with a moving robot).  If the target has been lost, then issue a warning,
                    // update the failing state, and set the ouptut at 0.0.
                    case Tracking:
                        if ( mOnTarget && mFoundTarget ) {
                            mOutput = mP * mErrorXPosition + mI * mTotalErrorXPosition + mD * mErrorXVelocity;
                        
                        } else if ( !mOnTarget && mFoundTarget ) {
                            mOutput = mP * mErrorXPosition + mI * mTotalErrorXPosition + mD * mErrorXVelocity;
                            mVisionState = VisionState_t.Seeking;
                            mSeekRetries = 0;
                            SetSeekTimer();
    
                        } else {
                            mOutput = 0.0;
                            mVisionState = VisionState_t.Fail;
                            mFailState = FailingState_t.LostTarget;
                        }
                        break;
                
                    // The controller has encountered a failure and needs to deal with it accordingly.
                    case Fail:
                        switch ( mFailState ) {
                            // If the target was lost while tracking, we can assume there was either a pretty
                            // significant change in the limelight pose, or, the tracking threshold is very wide open
                            // (nearing the horizontal FOV).  Either way, the controller will not try and recover from
                            // this and will leave it to the caller to issue the command again.
                            case LostTarget:
                                DriverStation.reportError( "Limelight lost target", false );
                                mOutput = 0.0;
                                mCommandState = CommandState_t.Idle;
                                break;

                            // A seek timout could happen because the timout threshold is too low, the controller took
                            // too long to get to the target, the tracking threshold is too low, there is no target
                            // around, or several other cases.  In any event, it doesn't hurt to report the issue and
                            // retry the seek.  Mostly, this mechanism is in place to give rise to a change in system
                            // behaviour (like seeks used to happen fast, now they're timing out once or twice before
                            // reaching the target...time to look into performance)
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

                            default:
                                DriverStation.reportWarning( "Limelight failure is unhandled", false );

                        }
                    break;
                }
                break;

            // The idle command literally does nothing and only exists for logging purposes.
            case Idle:
                mOutput = 0.0;
                break;

        }
        return mOutput;
    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    /**
    * This method will initialize the Vision subsystem.
    */
    private void Initialize ( double P, double I, double D, double F, double onTargetLimit ) {
        mP = P;
        mI = I;
        mD = D;
        mF = F;
        mOnTargetLimit = onTargetLimit;
        mMinIntegral = -1.0;
        mMaxIntegral = 1.0;
        ResetCalculations();
        ResetState();
    }

    /**
    * This method will updated the limelight targeting information.
    */ 
    private void Calculate ( double dt ) {
        if ( mLimelight.GetFoundTarget() ) {
            mFoundTarget = true;
            mCurrentXPosition = mLimelight.GetHorizontalToTargetDeg();
            mPrevErrorXPosition = mErrorXPosition;
            mErrorXPosition = mTargetXPosition - mCurrentXPosition;
            mErrorXVelocity = ( mErrorXPosition - mPrevErrorXPosition ) / dt;
            if ( mI != 0.0 ) {
                mTotalErrorXPosition = MathUtil.clamp(mTotalErrorXPosition + mErrorXPosition * dt, mMinIntegral / mI, mMaxIntegral / dt);
            }
            if ( Math.abs( mErrorXPosition ) < mOnTargetLimit ) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }   
        } else {
            mOnTarget = false;
            mFoundTarget = false;
            ResetCalculations();
        }
    }

    /**
    * This method will .
    */ 
    private void ResetCalculations () {
        mTargetXPosition = 0.0;
        mCurrentXPosition = 0.0;
        mErrorXPosition = 0.0;
        mPrevErrorXPosition = 0.0;
        mTotalErrorXPosition = 0.0;
        mErrorXVelocity = 0.0;
        mOutput = 0.0;
    }

    /**
    * This method will .
    */ 
    private void ResetState () {
        mOnTarget = false;
        mFoundTarget = false;
        mSeekRetries = 0;
        SetSeekTimer();
        mCommandState = CommandState_t.Idle;
        mVisionState = VisionState_t.Seeking;
        mFailState = null;
    }

    /**
    * This method will set the mSeekTimer_S variable to the current time in seconds.
    */
    private void SetSeekTimer () {
        mSeekTimer_S = Timer.getFPGATimestamp();
    }

    /**
    * This method will set the P-gain of the velocity controller.
    */  
    private void SetP ( double p ) {
        mP = p;
    }
    
    /**
    * This method will get the P-gain of the velocity controller.
    */  
    private double GetP () {
        return mP;
    }

    /**
    * This method will set the I-gain of the velocity controller.
    */  
    private void SetI ( double i ) {
        mI = i;
    }

    /**
    * This method will get the I-gain of the velocity controller.
    */  
    private double GetI () {
        return mI;
    }


    /**
    * This method will set the D-gain of the velocity controller.
    */  
    private void SetD ( double d ) {
        mD = d;
    }

    /**
    * This method will get the D-gain of the velocity controller.
    */  
    private double GetD () {
        return mD;
    }

    /**
    * This method will set the feedforward gain of the velocity controller.
    */  
    private void SetF ( double f ) {
        mF = f;
    }

    /**
    * This method will get the feedforward gain of the velocity controller.
    */  
    private double GetF () {
        return mF;
    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    public LimelightVision ( Limelight limelight, double P, double I, double D, double F, double onTargetLimit ) {
        mLimelight = limelight;
        Initialize ( P, I, D, F, onTargetLimit );
    }
    
    public static LimelightVision Create ( double P, double I, double D, double F, double onTargetLimit ) {
        Limelight limelight = new Limelight();
        return new LimelightVision( limelight, P, I, D, F, onTargetLimit );
    }

    /**
    * We are overriding the initSendable and using it to send information back-and-forth between the robot program and
    * the user PC for live PID tuning purposes.
    * @param SendableBuilder This is inherited from SubsystemBase
    */ 
    @Override
    public void initSendable ( SendableBuilder builder ) {
        builder.setSmartDashboardType( "Vision Turn PID Tuning" );
        builder.addDoubleProperty( "P", this::GetP, this::SetP);
        builder.addDoubleProperty( "I", this::GetI, this::SetI);
        builder.addDoubleProperty( "D", this::GetD, this::SetD);
        builder.addDoubleProperty( "F", this::GetF, this::SetF);
    }

}