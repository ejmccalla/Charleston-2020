package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SolenoidBase;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.Constants;
import frc.robot.lib.drivers.PressureSensor;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.lib.drivers.PDP;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import org.slf4j.Logger;

public class RobotContainer {
    
    // Hardware
    // private final XboxController mDriverXbox = new XboxController( Constants.DRIVER_XBOX );
    // private final XboxController mOperatorXbox = new XboxController( Constants.OPERATOR_XBOX );
    // private final Joystick mDriverJoystickThrottle = new Joystick( Constants.DRIVER_JOYSTICK_THROTTLE );
    // private final JoystickButton mDriverJoystickThrottleButton = new JoystickButton( mDriverJoystickThrottle, 1 );
    // private final Joystick mDriverJoystickTurn = new Joystick( Constants.DRIVER_JOYSTICK_TURN );
    // private final JoystickButton mDriverJoystickTurnButton = new JoystickButton( mDriverJoystickTurn, 1 );
    private final PressureSensor mPressureSensor = new PressureSensor( Constants.PRESSURE_SENSOR_ANALOG_CHANNEL, Constants.PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE, Constants.PRESSURE_SENSOR_PRESSURE_PER_VOLT );
    private final Photoeye mPhotoeye = new Photoeye( Constants.PHOTOEYE_DIGITAL_CHANNEL );
    private final PowerDistributionPanel mPDP = PDP.createPDP( new PowerDistributionPanel( Constants.PDP_ID ), Constants.PDP_ID );
    // Subsystems
    private final Drivetrain mDrivetrain = Drivetrain.create();
    
    // Autonomous chooser
    //private final TeleopDrive mTeleopDrive = new TeleopDrive( mDrivetrain, mDriverXbox, mDriverJoystickThrottle, mDriverJoystickTurn );
    private final SendableChooser<Command> mAutoChooser = new SendableChooser<>();

    // Match states for debug data output
    public static enum MatchState_t {
        robotInit {
            @Override public String toString() { return "Robot Init"; }
        },
        robotPeriodic {
            @Override public String toString() { return "Robot Periodic"; }
        },
        disabledInit {
            @Override public String toString() { return "Disabled Init"; }
        },
        autonomousInit {
            @Override public String toString() { return "Autonomous Init"; }
        },
        autonomousPeriodic {
            @Override public String toString() { return "Autonomous Periodic"; }
        },
        teleopInit {
            @Override public String toString() { return "Teleop Init"; }
        },
        teleopPeriodic {
            @Override public String toString() { return "Teleop Periodic"; }
        };
    }
    
    private MatchState_t mMatchState;

    public MatchState_t GetMatchState () {
        return mMatchState;
    }

    public void SetMatchState (MatchState_t matchState) {
        mMatchState = matchState;
    }

    public Command GetAutonomousCommand () {
        return mAutoChooser.getSelected();
    }

    // Button mappings
    private void ConfigureButtonBindings () {
        // new JoystickButton( mDriverXbox, 1).whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        // new JoystickButton( mDriverXbox, 4).whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
        // mDriverJoystickThrottleButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        // mDriverJoystickTurnButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
    }

    // Debug logging
    public void LogRobotDataHeader (Logger fileLogger) {
        fileLogger.debug( "Time,"+
                          "Match State,"+
                          "Pressure (PSI),"+
                          "Photoeye Closed,"+
                          "PDP Voltage,"+
                          "PDP Slot 0 Current"
                          );
    }   
    public void LogRobotDataToRoboRio (Logger fileLogger) {
        fileLogger.debug( "{},{},{},{},{},{}", 
                          Timer.getFPGATimestamp(),
                          mMatchState.toString(),
                          mPressureSensor.GetPressureInPSI(),
                          mPhotoeye.IsPhotoeyeClosed(),
                          mPDP.getVoltage(),
                          mPDP.getCurrent( Constants.DRIVETRAIN_LEFT_MASTER_ID )
                          );
    }

    // Smartdashboard output
    public void UpdateSmartDashboard() {
        SmartDashboard.putNumber( "Pressure Sensor (PSI)", mPressureSensor.GetPressureInPSI() );
        SmartDashboard.putBoolean( "Photoeye", mPhotoeye.IsPhotoeyeClosed() );
        mDrivetrain.OutputSmartDashboard();
    }

    public RobotContainer () {
        ConfigureButtonBindings();
        //mDrivetrain.setDefaultCommand( new TeleopDrive( mDrivetrain, mDriverXbox, mDriverJoystickThrottle, mDriverJoystickTurn ) );
        mDrivetrain.setDefaultCommand( new RunCommand( () -> mDrivetrain.mDifferentialDrive.arcadeDrive( 0.0, 0.0 ), mDrivetrain ) );
        mAutoChooser.setDefaultOption( "Auto 1", new Auto1( mDrivetrain ) );
        mAutoChooser.addOption( "Auto 2", new Auto2( mDrivetrain ) );
        SmartDashboard.putData( "Auto Chooser", mAutoChooser );
        mMatchState = MatchState_t.robotInit;
        SolenoidBase.clearAllPCMStickyFaults( Constants.PCM_ID );
    }

}
