package frc.robot.lib.controllers;

import java.lang.Math;
import edu.wpi.first.wpiutil.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.drivers.Limelight;
import frc.robot.lib.drivers.Limelight.LEDMode_t;

/**
* The LimelightVision class implements control algorithms using vision as a direct feedback sensor.
* 
* <p>
* <b>Features</b><ul>
* <li>Interactive PID Tuning - All control parameters are sent to the <i>Live Window</i> output for on-the-fly control
* loop tuning
* <li>Threading - Class can be configured to run in the main robot thread or in it's own thread using the startPeriodic
* method of the Notifier class
* <li>Automated LED Control - The LED's will only be turned on while actively running a targeting command
* <li>Distance Estimator - Support for measuring distance-to-target using fixed angles and bounding box target area
* </ul><p>
* <b>Prerequisites</b><ul>
* <li>Calibrated Limelight Pipeline(s)
* {@link https://docs.limelightvision.io/en/latest/vision_pipeline_tuning.html}
* <li>Calibrated Limelight Crosshair(s)
* {@link https://docs.limelightvision.io/en/latest/crosshair_calibration.html}
* <li>Calibrated Focal Length
* {@link https://github.com/ejmccalla/Charleston-2020/blob/master/src/main/java/frc/robot/Robot.java}
* <li>Calibrated Control Loop Parameters (PIDF and On-Target Thresholds)
* </ul><p>
* <b>Interactive PID Tuning</b><p>
* The PIDF gains and on-target thresholds are sent to the <i>Live Window</i> and can be accessed for on-the-fly tuning
* updates by running the robot in <i>test</i> mode.  Also, the command scheduler is re-enabled 
* <i>CommandScheduler.getInstance().enable();</i> in order to use the user input to run commands.  It is recommend to
* use Shuffleboard to acccess the tuning parameters under the <i>Live Window</i> tab.
* <p>
* <b>Threading</b><p>
* Thread safety for LimelightVision class is acheived by synchronizing all methods which modify the shared state of the
* controller.  With this in place, this class can be setup to run as a standalone thread or within the main robot
* thread.
* <p>
* <b>Automated LED Control</b><p>
* The Limelight Vision controller will handle the LED state automatically by turning off the LEDs whenever the current
* command is Idle.  By doing so, the Limelight will be in accordance with R8-M of the 2020 FRC game manual.
* <p>
* <b>Distance Estimator</b><p>
* <b>Controller</b><p>
* All of the shared controller state can be read using the <i>GetSharedState</i> method and accessing the member
* variables.  The shared state <i>outputTurn</i> and <i>outputDistance</i> are the raw output from the PIDF
* calculations.  It is up to the caller to ensure these outputs are applied correctly to the mechanism is being
* controlled.  The feed-forward term is applied differently by this controller than normal implementations.  The term
* is only used to set the <i>outputTurn</i> when the Limelight doesn't have a target identified.  When the controller
* receives an input targeting comnmand, and a target isn't within sight, the controller will enter into the
* <i>searching</i> state.  In this state, the <i>outputTurn</i> will be set to the feed-forward, the
* <i>outputDistance</i> will be set to 0.0, and the controller will keep doing so until either a target is identified
* or the search times out.  For the case when the controller has identified a target, as indicated by the shared state
* <i>foundTarget</i>, the <i>outputTurn</i> and <i>outputDistance</i> will be the normal PID calculations.  For further
* details on the controller, see {@link #Update}
* <p>

*/  
public class LimelightVision implements Sendable, AutoCloseable {

    // Count class instances
    private static int mInstances;

    // State enumerations
    public enum VisionState_t {
        Searching { @Override public String toString() { return "Searching"; } },
        Seeking { @Override public String toString() { return "Seeking"; } },
        Tracking { @Override public String toString() { return "Tracking"; } },
        Failing { @Override public String toString() { return "Fail"; } };
    }

    public enum CommandState_t {
        Idle { @Override public String toString() { return "Idle"; } },
        TurnToTarget { @Override public String toString() { return "Turn-to-Target"; } },
        DriveToTarget { @Override public String toString() { return "Drive-to-Target"; } }
    }

    public enum FailingState_t {
        Ok { @Override public String toString() { return "Ok"; } },        
        TargetNotFound { @Override public String toString() { return "Target Search Timeout"; } },
        SeekTimeout { @Override public String toString() { return "Seek Timeout"; } },
        ExhaustedSeekRetries { @Override public String toString() { return "Exhausted Seek Retries"; } },
        LostTarget { @Override public String toString() { return "Lost Target"; } };
    }

    // Hardware
    private final Limelight mLimelight;

    // Constants set during object construction
    private double mTargetSize_FT;
    private double mFocalLength_FT;
    private double mSearchTimeout_S;
    private double mSeekTimeout_S;
    private int mSeekRetryLimit;

    // Tuning parameters set during object construction and updated during test mode via live window
    private volatile double mP, mI, mD, mF;
    private volatile double mOnTargetTurnErrorThreshold_DEG;
    private volatile double mOnTargetDistanceErrorThreshold_FT;

    // Non-shared State variables
    private int mSeekRetries;
    private double mSeekTimer_S;
    private double mLastUpdateTime_S;
    private double mPrevErrorXPosition_DEG;
    private CircularBuffer mIntegralBuffer;
    private final int mIntegralBufferSize = 25;
    
    // Shared state variables
    public class SharedState {
        public CommandState_t desiredCommand;
        public CommandState_t currentCommand;
        public VisionState_t visionState;
        public FailingState_t failState;
        public boolean onTargetTurn;
        public boolean onTargetDistance;
        public boolean foundTarget;
        public double deltaTime_S;
        public double currentErrorXPosition_DEG;
        public double currentErrorXVelocity_DEGPS;
        public double totalErrorXPosition_DEG;
        public double targetDistance_FT;
        public double currentErrorDistance_FT;
        public double outputTurn;
        public double outputDistance;
        public SharedState () {
            desiredCommand = CommandState_t.Idle;
            currentCommand = CommandState_t.Idle;
            visionState = VisionState_t.Seeking;
            failState = FailingState_t.Ok;
            onTargetTurn = false;
            onTargetDistance = false;
            foundTarget = false;
            deltaTime_S = 0.0;
            currentErrorXPosition_DEG = 0.0;
            currentErrorXVelocity_DEGPS = 0.0;
            totalErrorXPosition_DEG = 0.0;
            targetDistance_FT = 0.0;
            currentErrorDistance_FT = 0.0;
            outputTurn = 0.0;
            outputDistance = 0.0;
        }
    }
    SharedState mSharedState;

    // Threading interface
    public Runnable mThread = new Runnable() { @Override public void run() { Update(); } };


    //-----------------------------------------------------------------------------------------------------------------
    /*                                              PUBLIC API METHODS                                               */
    //-----------------------------------------------------------------------------------------------------------------


    /**
    * This method will requrest the controller to turn to the target.
    */
    public synchronized void TurnToTarget () {
        mSharedState.desiredCommand = CommandState_t.TurnToTarget;
    }

    /**
    * This method will requrest the controller to drive to the target.
    *
    * @param targetDistance_FT double The target distance-to-target
    */
    public synchronized void DriveToTarget ( double targetDistance_FT) {
        mSharedState.desiredCommand = CommandState_t.DriveToTarget;
        mSharedState.targetDistance_FT = targetDistance_FT;
    }

    /**
     * This method will requrest the controller stop the current command.
     */
    public synchronized void Idle() {
        mSharedState.desiredCommand = CommandState_t.Idle;
    }

    /**
     * This method will requrest the controller to run the updated method. It's intended to be used when the controller
     * is running in the main robot thread.
     */
    public void RunUpdate() {
        Update();
    }

    /**
     * This method will return the current values of the shared state variables.
     * 
     * @return SharedState The current value of the shared state variables
     */
    public synchronized SharedState GetSharedState() {
        return mSharedState;
    }

    /**
     * This method will update the values of the shared state variables.
     * 
     * @param updatedSharedState SharedState The new shared state
     */
    public synchronized void SetSharedState ( SharedState updatedState ) {
        mSharedState = updatedState;
    }


    // -----------------------------------------------------------------------------------------------------------------
    /* PRIVATE METHODS */
    // -----------------------------------------------------------------------------------------------------------------


    /**
     * This method will update the state of the controller.
     * <p>
     * The seeking state is the initial controller state when the command begins.  If we get lucky and happen to be be
     * on-target, the state is updated to tracking and the output is set to the PID calculation.  If we aren't
     * on-target, but do have a target identified, then use the PID loop to calculate the output and continue seeking
     * to the target.  If there's no target identified, then use the feed-forward term to "search" for a target and
     * update the state to "Searching".  If the seek timer expires or the seek retries are exhausted, the state is
     * updated to failing and the output is set to 0.0.  The failure will be handled by the logic in the failing state.
     * <p>
     * The tracking state means that the error in the position is within the given range and we are setting the output
     * to maintain, or further decrease, the error.  If the target is no longer within the specified range, then the
     * controller will simply fall back to the seeking state.  This is expected to be an unlikely event when things are
     * static, but may be more common for dynamic systems (like a vision controlled turret with a moving robot).  If
     * the target has been lost, update to the failing state, and set the ouptut at 0.0.
     * <p>
     * The searching state means that the controller has yet to identity a target and is "searching" for a target by
     * sending the feed-forward term as the output.  The conroller will continue to do this until either a target is
     * identified and the controller moves to the seeking state, or, the controller times out and moves to the failing
     * state.
     * <p>
     * The failing state means that one of the following errors was encountered: the seek timed out, there were too
     * many seek retries, the target was lost during tracking, or a target was never found.  If the target was lost
     * while tracking, we can assume there was either a sudden and significant change in the limelight pose, or, the
     * tracking threshold is wide open (nearing the horizontal FOV).  Either way, the controller will not try and
     * recover from this and will leave it to the caller to issue the command again.  A seek timout could happen
     * because the timeout threshold is too low, the controller took too long to get to the target, the tracking
     * threshold is too low, or one of several other reasons.  In any event, it doesn't hurt to report the issue and
     * retry the seek.  Mostly, this mechanism is in place to give rise to a change in system behaviour (like seeks
     * used to happen fast, now they're timing out once or twice before reaching the target...time to look into
     * performance).  Finally, if the Limelight was never able to identify a target and timed out while looking for
     * one.  The user will have to issue the command again to continue the search or consider increasing the search
     * timeout threshold.
     */
    private void Update () {
        double currentTime = Timer.getFPGATimestamp();
        SharedState updatedState = GetSharedState();

        // Update the targeting state and error calculations
        updatedState.deltaTime_S = currentTime - mLastUpdateTime_S;
        mLastUpdateTime_S = currentTime;
        if ( mLimelight.GetFoundTarget() ) {
            updatedState.foundTarget = true;
            mPrevErrorXPosition_DEG = updatedState.currentErrorXPosition_DEG;
            updatedState.currentErrorXPosition_DEG = mLimelight.GetHorizontalToTargetDeg();
            updatedState.currentErrorXVelocity_DEGPS =
                (updatedState.currentErrorXPosition_DEG - mPrevErrorXPosition_DEG) / updatedState.deltaTime_S;

            // The total error for computing the integral term uses the sum of a fixed-length buffer and clamps the
            // magnitude.
            double currentIntegral = updatedState.currentErrorXPosition_DEG * updatedState.deltaTime_S;
            updatedState.totalErrorXPosition_DEG -= mIntegralBuffer.get( mIntegralBufferSize - 1 ); // Subtract the old
            updatedState.totalErrorXPosition_DEG += currentIntegral; // Add the new
            mIntegralBuffer.addFirst( currentIntegral ); // Limit integration length
            updatedState.totalErrorXPosition_DEG = MathUtil.clamp( updatedState.totalErrorXPosition_DEG,
                                                                   -mOnTargetTurnErrorThreshold_DEG, 
                                                                   mOnTargetTurnErrorThreshold_DEG );
            
            updatedState.currentErrorDistance_FT = updatedState.targetDistance_FT - EstimateDistance( mLimelight.GetHorizontalPixels() );
            if ( Math.abs( updatedState.currentErrorXPosition_DEG ) < mOnTargetTurnErrorThreshold_DEG ) {
                updatedState.onTargetTurn = true;
            } else {
                updatedState.onTargetTurn = false;
            }
            if ( Math.abs( updatedState.currentErrorDistance_FT ) < mOnTargetDistanceErrorThreshold_FT ) {
                updatedState.onTargetDistance = true;
            } else {
                updatedState.onTargetDistance = false;
            }
        } else {
            updatedState.foundTarget = false;
            updatedState.onTargetTurn = false;
            updatedState.onTargetDistance = false;
            updatedState.currentErrorXPosition_DEG = 0.0;
            updatedState.currentErrorXVelocity_DEGPS = 0.0;
            updatedState.totalErrorXPosition_DEG = 0.0;
            updatedState.currentErrorDistance_FT = 0.0;
            mPrevErrorXPosition_DEG = 0.0;
        }

        // Is there a new command request to process?
        if ( updatedState.currentCommand != updatedState.desiredCommand ) {
            updatedState.currentCommand = updatedState.desiredCommand;
            updatedState.visionState = VisionState_t.Seeking;
            mSeekRetries = 0;
            SetSeekTimer();
            updatedState.failState = FailingState_t.Ok;
        }
        if ( updatedState.currentCommand == CommandState_t.Idle ) {
            mLimelight.SetLedMode( LEDMode_t.Off );
        
        } else {
            mLimelight.SetLedMode( LEDMode_t.On );
        }


        // Update the vision and failing states as well as the output
        switch ( updatedState.currentCommand ) {
        case DriveToTarget:
            switch ( updatedState.visionState ) {
            case Seeking: // DriveToTarget
                if ( updatedState.onTargetTurn && updatedState.onTargetDistance ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.outputDistance = CalculateDriveToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                                updatedState.currentErrorDistance_FT );
                    updatedState.visionState = VisionState_t.Tracking;

                } else if ( currentTime - mSeekTimer_S > mSeekTimeout_S ) {
                    updatedState.outputTurn = 0.0;
                    updatedState.outputDistance = 0.0;
                    updatedState.visionState = VisionState_t.Failing;
                    mSeekRetries += 1;

                    if ( mSeekRetries > mSeekRetryLimit ) {
                        updatedState.failState = FailingState_t.ExhaustedSeekRetries;
                    } else {
                        updatedState.failState = FailingState_t.SeekTimeout;
                    }

                } else if ( updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.outputDistance = CalculateDriveToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                                updatedState.currentErrorDistance_FT );

                } else {
                    updatedState.outputTurn = mF;
                    updatedState.outputDistance = 0.0;
                    updatedState.visionState = VisionState_t.Searching;

                }
                break;

            case Tracking: // DriveToTarget
                if ( updatedState.onTargetTurn && updatedState.onTargetDistance && updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.outputDistance = CalculateDriveToTargetOutput( updatedState.currentErrorXPosition_DEG, 
                                                                                updatedState.currentErrorDistance_FT );
                                                                                
                } else if ( ( !updatedState.onTargetTurn || !updatedState.onTargetDistance ) && updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.outputDistance = CalculateDriveToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                                updatedState.currentErrorDistance_FT );
                    updatedState.visionState = VisionState_t.Seeking;
                    mSeekRetries = 0;
                    SetSeekTimer();

                } else {
                    updatedState.outputTurn = 0.0;
                    updatedState.outputDistance = 0.0;
                    updatedState.visionState = VisionState_t.Failing;
                    updatedState.failState = FailingState_t.LostTarget;
                }
                break;

            case Searching: // DriveToTarget
                if ( currentTime - mSeekTimer_S > mSearchTimeout_S ) {
                    updatedState.outputTurn = 0.0;
                    updatedState.outputDistance = 0.0;
                    updatedState.failState = FailingState_t.TargetNotFound;

                } else if ( updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.outputDistance = CalculateDriveToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                                updatedState.currentErrorDistance_FT );
                    updatedState.visionState = VisionState_t.Seeking;
                    mSeekRetries = 0;
                    SetSeekTimer();

                } else {
                    updatedState.outputTurn = mF;
                }
                break;

            case Failing: // DriveToTarget
                switch ( updatedState.failState ) {
                case LostTarget:
                    DriverStation.reportError( "Limelight lost target", false );
                    updatedState.outputTurn = 0.0;
                    updatedState.outputDistance = 0.0;
                    updatedState.desiredCommand = CommandState_t.Idle;
                    break;

                case SeekTimeout:
                    DriverStation.reportWarning( "Limelight seek timeout", false );
                    updatedState.visionState = VisionState_t.Seeking;
                    updatedState.failState = FailingState_t.Ok;
                    SetSeekTimer();
                    break;

                case ExhaustedSeekRetries:
                    DriverStation.reportError( "Limelight exhausted seek retries", false );
                    updatedState.outputTurn = 0.0;
                    updatedState.outputDistance = 0.0;
                    updatedState.desiredCommand = CommandState_t.Idle;
                    break;

                case TargetNotFound:
                    DriverStation.reportError( "Limelight couldn't find target", false );
                    updatedState.outputTurn = 0.0;
                    updatedState.outputDistance = 0.0;
                    updatedState.desiredCommand = CommandState_t.Idle;
                    break;

                default:
                    DriverStation.reportWarning( "Limelight failure is unhandled", false );
                    break;
                } // switch ( updatedState.failState )
                break;
            } // switch ( updatedState.visionState )
            break;

        case TurnToTarget:
            updatedState.outputDistance = 0.0;
            switch ( updatedState.visionState ) {
            case Seeking: // TurnToTarget
                if ( updatedState.onTargetTurn ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.visionState = VisionState_t.Tracking;

                } else if ( currentTime - mSeekTimer_S > mSeekTimeout_S ) {
                    updatedState.outputTurn = 0.0;
                    updatedState.visionState = VisionState_t.Failing;
                    mSeekRetries += 1;

                    if ( mSeekRetries > mSeekRetryLimit ) {
                        updatedState.failState = FailingState_t.ExhaustedSeekRetries;
                    } else {
                        updatedState.failState = FailingState_t.SeekTimeout;
                    }

                } else if ( updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );

                } else {
                    updatedState.outputTurn = mF;
                    updatedState.visionState = VisionState_t.Searching;

                }
                break;

            case Tracking: // TurnToTarget
                if ( updatedState.onTargetTurn && updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );

                } else if ( !updatedState.onTargetTurn && updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.visionState = VisionState_t.Seeking;
                    mSeekRetries = 0;
                    SetSeekTimer();

                } else {
                    updatedState.outputTurn = 0.0;
                    updatedState.visionState = VisionState_t.Failing;
                    updatedState.failState = FailingState_t.LostTarget;
                }
                break;

            case Searching: // TurnToTarget
                if ( currentTime - mSeekTimer_S > mSearchTimeout_S ) {
                    updatedState.outputTurn = 0.0;
                    updatedState.failState = FailingState_t.TargetNotFound;

                } else if ( updatedState.foundTarget ) {
                    updatedState.outputTurn = CalculateTurnToTargetOutput( updatedState.currentErrorXPosition_DEG,
                                                                           updatedState.totalErrorXPosition_DEG,
                                                                           updatedState.currentErrorXVelocity_DEGPS );
                    updatedState.visionState = VisionState_t.Seeking;
                    mSeekRetries = 0;
                    SetSeekTimer();

                } else {
                    updatedState.outputTurn = mF;
                }
                break;

            case Failing: // TurnToTarget
                switch ( updatedState.failState ) {
                case LostTarget:
                    DriverStation.reportError( "Limelight lost target", false );
                    updatedState.outputTurn = 0.0;
                    updatedState.desiredCommand = CommandState_t.Idle;
                    break;

                case SeekTimeout:
                    DriverStation.reportWarning( "Limelight seek timeout", false );
                    updatedState.visionState = VisionState_t.Seeking;
                    updatedState.failState = FailingState_t.Ok;
                    SetSeekTimer();
                    break;

                case ExhaustedSeekRetries:
                    DriverStation.reportError( "Limelight exhausted seek retries", false );
                    updatedState.outputTurn = 0.0;
                    updatedState.desiredCommand = CommandState_t.Idle;
                    break;

                case TargetNotFound:
                    DriverStation.reportError( "Limelight couldn't find target", false );
                    updatedState.outputTurn = 0.0;
                    updatedState.desiredCommand = CommandState_t.Idle;
                    break;

                default:
                    DriverStation.reportWarning( "Limelight failure is unhandled", false );
                    break;
                } // switch ( updatedState.failState )
                break;
            } // switch ( updatedState.visionState )
            break;

        case Idle:
            updatedState.outputTurn = 0.0;
            updatedState.outputDistance = 0.0;
            break;

        } // switch ( updatedState.currentCommand )
        SetSharedState( updatedState );
    }

    /**
     * This method will initialize the Limelight vision subsystem by setting the closed-loop parameters, resetting all
     * of the PID calculations, and resettingall of the internal states.
     * 
     * @param searchTimeout_S double The timeout for the controller to search for target
     * @param seekTimeout_S double The timeout for the controller to get on-target
     * @param seekRetryLimit int The number of seek retries allowed
     * @param P double The proportional gain
     * @param I double The integral gain
     * @param D double The differential gain
     * @param F double The feed-forward gain
     * @param onTargetTurnErrorThreshold_DEG double The on-target turning threshold used to differentiate seeking and
     *                                              tracking states
     * @param onTargetDistanceErrorThreshold_FT double The on-target distance threshold used to differentiate seeking
     *                                                 and tracking states
     * @param targetSize_FT double The measured target size
     * @param focalLength_FT double The measured focal length
     */
    private void Initialize ( double searchTimeout_S, double seekTimeout_S, int seekRetryLimit,
                              double P, double I, double D, double F,
                              double onTargetTurnErrorThreshold_DEG, double onTargetDistanceErrorThreshold_FT,
                              double targetSize_FT, double focalLength_FT ) {

        // Constants set during object construction
        mSearchTimeout_S = searchTimeout_S;
        mSeekTimeout_S = seekTimeout_S;
        mSeekRetryLimit = seekRetryLimit;
        mTargetSize_FT = targetSize_FT;
        mFocalLength_FT = focalLength_FT;

        // Tuning parameters set during object construction and updated durint test mode
        // via live window
        mP = P;
        mI = I;
        mD = D;
        mF = F;
        mOnTargetTurnErrorThreshold_DEG = onTargetTurnErrorThreshold_DEG;
        mOnTargetDistanceErrorThreshold_FT = onTargetDistanceErrorThreshold_FT;

        // Non-shared State variables
        mSeekRetries = 0;
        SetSeekTimer(); // mSeekTimer_S;
        mLastUpdateTime_S = 0.0;
        mPrevErrorXPosition_DEG = 0.0;
        mIntegralBuffer = new CircularBuffer( mIntegralBufferSize );

        // Shated state variables
        mSharedState = new SharedState();
    }

    /**
     * This method will calculate the output for drive-to-target command
     * 
     * @param currentErrorXPosition_DEG double The current turning error
     * @param currentErrorDistance_FT double The current driving error
     */
    private double CalculateDriveToTargetOutput ( double currentErrorXPosition_DEG,
                                                  double currentErrorDistance_FT ) {
        return (1 - (Math.abs(currentErrorXPosition_DEG) / 29.8)) * 0.2 * currentErrorDistance_FT;
    }

    /**
     * This method will calculate the output for turn-to-target command
     * 
     * @param currentErrorXPosition_DEG double The current turning error
     * @param totalErrorXPosition_DEG double The total turning error
     * @param currentErrorXVelocity_DEGPS double The turning velocity error
     */
    private double CalculateTurnToTargetOutput ( double currentErrorXPosition_DEG,
                                                 double totalErrorXPosition_DEG,
                                                 double currentErrorXVelocity_DEGPS ) {
        return mP * currentErrorXPosition_DEG + mI * totalErrorXPosition_DEG + mD * currentErrorXVelocity_DEGPS;
        //return 0.0;
    }

    /**
     * This method will estimate the distance-to-target.
     * 
     * @param pixels double The size of the target in pixels
     */
    private double EstimateDistance ( double pixels ) {
        return ( mTargetSize_FT * mFocalLength_FT ) / pixels;
    }

    /**
     * This method will set the mSeekTimer_S variable to the current time in seconds.
     */
    private void SetSeekTimer () {
        mSeekTimer_S = Timer.getFPGATimestamp();
    }

    /**
     * This method will set the P-gain of the controller and is used by the live window sendable.
     *
     * @param p double P-gain
     */
    private void SetP ( double p ) {
        mP = p;
    }

    /**
     * This method will get the P-gain of the controller and is used by the live window sendable.
     * 
     * @return double The value of the P-gain
     */
    private double GetP () {
        return mP;
    }

    /**
     * This method will set the I-gain of the controller and is used by the live window sendable.
     *
     * @param i double I-gain
     */
    private void SetI ( double i ) {
        mI = i;
    }

    /**
     * This method will get the I-gain of the controller and is used by the live window sendable.
     * 
     * @return double The value of the I-gain
     */
    private double GetI () {
        return mI;
    }

    /**
     * This method will set the D-gain of the controller and is used by the live window sendable.
     *
     * @param d double D-gain
     */
    private void SetD ( double d ) {
        mD = d;
    }

    /**
     * This method will get the D-gain of the controller and is used by the live window sendable.
     * 
     * @return double The value of the D-gain
     */
    private double GetD() {
        return mD;
    }

    /**
     * This method will set the feedforward gain of the controller and is used by the live window sendable.
     *
     * @param f double Feedforward-gain
     */
    private void SetF ( double f ) {
        mF = f;
    }

    /**
     * This method will get the feedforward gain of the controller and is used by the live window sendable.
     * 
     * @return double The value of the FF-gain
     */
    private double GetF () {
        return mF;
    }

    /**
     * This method will set the on-target turning error threshold of the controller and is used by the live window
     * sendable.
     *
     * @param onTargetTurnErrorThreshold_DEG double Turning error threshold
     */
    private void SetOnTargetTurnErrorThreshold ( double onTargetTurnErrorThreshold_DEG ) {
        mOnTargetTurnErrorThreshold_DEG = onTargetTurnErrorThreshold_DEG;
    }

    /**
     * This method will get the on-target turning error threshold of the controller and is used by the live window
     * sendable.
     * 
     * @return double The value of the on-target threshold
     */
    private double GetOnTargetTurnErrorThreshold () {
        return mOnTargetTurnErrorThreshold_DEG;
    }

    /**
     * This method will set the on-target distance error threshold of the controller and is used by the live window
     * sendable.
     * 
     * @param onTargetDistanceErrorThreshold_FT double Turning error threshold
     */
    private void SetOnTargetDistanceErrorThreshold_FT ( double onTargetDistanceErrorThreshold_FT ) {
        mOnTargetDistanceErrorThreshold_FT = onTargetDistanceErrorThreshold_FT;
    }

    /**
     * This method will get the on-target distance error threshold of the controller and is used by the live window
     * sendable.
     * 
     * @return double The value of the on-target threshold
     */
    private double GetOnTargetDistanceErrorThreshold_FT () {
        return mOnTargetDistanceErrorThreshold_FT;
    }


    // -----------------------------------------------------------------------------------------------------------------
    /* CLASS CONSTRUCTOR AND OVERRIDES */
    // -----------------------------------------------------------------------------------------------------------------


    /**
     * The constructor for the LimelightVision class.
     * 
     * @param limelight Limelight The limelight camera object used for this controller
     * @param searchTimeout_S double The timeout used during <i>searching</i> state
     * @param seekTimeout_S double The timeout used during <i>seeking</i> state
     * @param seekRetryLimit int The number of allowed seek retries before failing
     * @param P double The proportional gain
     * @param I double The integral gain
     * @param D double The differential gain
     * @param F double The feed-forward gain
     * @param onTargetTurnErrorThreshold_DEG double The on-target turning threshold used to differentiate seeking and
     *                                              tracking states
     * @param onTargetDistanceErrorThreshold_FT double The on-target distance threshold used to differentiate seeking
     *                                                 and tracking states
     * @param targetSize_FT double The measured target size
     * @param focalLength_FT double The measured focal length
     */
    public LimelightVision ( Limelight limelight, double searchTimeout_S, double seekTimeout_S,
                             int seekRetryLimit, double P, double I, double D, double F,
                             double onTargetTurnErrorThreshold_DEG, double onTargetDistanceErrorThreshold_FT,
                             double targetSize_FT, double focalLength_FT ) {
        mLimelight = limelight;
        Initialize( searchTimeout_S, seekTimeout_S, seekRetryLimit, P, I, D, F, onTargetTurnErrorThreshold_DEG,
                    onTargetDistanceErrorThreshold_FT, targetSize_FT, focalLength_FT );

        mInstances++;
        SendableRegistry.addLW( this, "LimelightVision", mInstances );
    }

    /**
     * This mehtod will create a new LimelightVision object. The purpose of doing the constructor this way is to allow
     *  for unit testing.
     * 
     * @param searchTimeout_S double The timeout used during <i>searching</i> state
     * @param seekTimeout_S double The timeout used during <i>seeking</i> state
     * @param seekRetryLimit int The number of allowed seek retries before failing
     * @param P double The proportional gain
     * @param I double The integral gain
     * @param D double The differential gain
     * @param F double The feed-forward gain
     * @param onTargetTurnErrorThreshold_DEG double The on-target turning threshold used to differentiate seeking and
     *                                              tracking states
     * @param onTargetDistanceErrorThreshold_FT double The on-target distance threshold used to differentiate seeking
     *                                                 and tracking states
     * @param targetSize_FT double The measured target size
     * @param focalLength_FT double The measured focal length
     */
    public static LimelightVision Create ( double searchTimeout_S, double seekTimeout_S,
                                           int seekRetryLimit, double P, double I, double D,
                                           double F, double onTargetTurnErrorThreshold_DEG,
                                           double onTargetDistanceErrorThreshold_FT, double targetSize_FT,
                                           double focalLength_FT ) {
        Limelight limelight = new Limelight();
        return new LimelightVision ( limelight, searchTimeout_S, seekTimeout_S, seekRetryLimit, P, I, D, F,
                                     onTargetTurnErrorThreshold_DEG, onTargetDistanceErrorThreshold_FT,
                                     targetSize_FT, focalLength_FT );
    }

    /**
     * We are overriding the initSendable and using it to send information back-and-forth between the robot program and
     * the user PC for live PID tuning purposes.
     * 
     * @param builder SendableBuilder This is inherited from SubsystemBase
     */
    @Override
    public void initSendable ( SendableBuilder builder ) {
        NetworkTableEntry entryTimeDelta = builder.getEntry( "Time Delta" );
        NetworkTableEntry entryCurrentErrorXPosition = builder.getEntry( "Turn Error Position" );
        NetworkTableEntry entryCurrentErrorXVelocity = builder.getEntry( "Turn Error Velocity" );
        NetworkTableEntry entryTotalErrorXPosition = builder.getEntry( "Turn Error Total" );
        NetworkTableEntry entryOutputTurn = builder.getEntry( "Turn Ouput" );
        NetworkTableEntry entryPTerm = builder.getEntry( "P-Term" );
        NetworkTableEntry entryIterm = builder.getEntry( "I-Term" );
        NetworkTableEntry entryDTerm = builder.getEntry( "D-Term" );
        NetworkTableEntry entryTargetDistance = builder.getEntry( "Target Distance" );
        NetworkTableEntry entryEstimatedDistance = builder.getEntry( "Estimated Distance" );
        NetworkTableEntry entryDistanceError = builder.getEntry( "Distance Error" );
        NetworkTableEntry entryOutputDistance = builder.getEntry( "Distance Output" );
        builder.setSmartDashboardType( "Limelight Vision Controller PIDF Tuning" );
        builder.addDoubleProperty( "P", this::GetP, this::SetP );
        builder.addDoubleProperty( "I", this::GetI, this::SetI );
        builder.addDoubleProperty( "D", this::GetD, this::SetD );
        builder.addDoubleProperty( "F", this::GetF, this::SetF );
        builder.addDoubleProperty( "Turn On-Target Threshold", this::GetOnTargetTurnErrorThreshold,
                                   this::SetOnTargetTurnErrorThreshold );
        builder.addDoubleProperty( "Distance On-Target Threshold", this::GetOnTargetDistanceErrorThreshold_FT,
                                   this::SetOnTargetDistanceErrorThreshold_FT );
        builder.setUpdateTable( () -> { SharedState currentState = GetSharedState();
                                        entryTimeDelta.setDouble( currentState.deltaTime_S );
                                        entryCurrentErrorXPosition.setDouble( currentState.currentErrorXPosition_DEG );
                                        entryTotalErrorXPosition.setDouble( currentState.totalErrorXPosition_DEG );
                                        entryCurrentErrorXVelocity.setDouble( currentState.currentErrorXVelocity_DEGPS );
                                        entryOutputTurn.setDouble( currentState.outputTurn );
                                        entryPTerm.setDouble( currentState.currentErrorXPosition_DEG * mP );
                                        entryIterm.setDouble( currentState.totalErrorXPosition_DEG * mI );
                                        entryDTerm.setDouble( currentState.currentErrorXVelocity_DEGPS * mD ); 
                                        entryTargetDistance.setDouble( currentState.targetDistance_FT );
                                        entryEstimatedDistance.setDouble( currentState.targetDistance_FT - currentState.currentErrorDistance_FT );
                                        entryDistanceError.setDouble( currentState.currentErrorDistance_FT );
                                        entryOutputDistance.setDouble( currentState.outputDistance );                                    
                                      } );
    }

    /**
    * Override the close method from AutoCloseable.
    */ 
    @Override
    public void close () {
      SendableRegistry.remove( this );
    }    

} 
