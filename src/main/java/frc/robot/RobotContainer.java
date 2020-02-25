package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.HARDWARE;
import frc.robot.Constants.PRESSURE_SENSOR;
import frc.robot.lib.drivers.PressureSensor;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import org.slf4j.Logger;

public class RobotContainer {

    // State variables
    public static enum MatchState_t {
        robotInit { @Override public String toString() { return "Robot Init"; } },
        robotPeriodic { @Override public String toString() { return "Robot Periodic"; } },
        disabledInit { @Override public String toString() { return "Disabled Init"; } },
        disabledPeriodic { @Override public String toString() { return "Disabled Periodic"; } },
        autonomousInit { @Override public String toString() { return "Autonomous Init"; } },
        autonomousPeriodic { @Override public String toString() { return "Autonomous Periodic"; } },
        teleopInit { @Override public String toString() { return "Teleop Init"; } },
        teleopPeriodic { @Override public String toString() { return "Teleop Periodic"; } };
    }    
    private MatchState_t mMatchState;

    // Hardware
    private final Joystick mDriverJoystickThrottle;
    private final JoystickButton mDriverJoystickThrottleButton;
    private final Joystick mDriverJoystickTurn;
    private final JoystickButton mDriverJoystickTurnButton;
    private final PressureSensor mPressureSensor;
    private final Photoeye mPhotoeye;
    private final PowerDistributionPanel mPDP;
    
    // Subsystems
    private Drivetrain mDrivetrain = Drivetrain.create();
    
    // Autonomous chooser
    private final SendableChooser<Command> mAutoChooser = new SendableChooser<>();

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    public MatchState_t GetMatchState () {
        return mMatchState;
    }

    public void SetMatchState (MatchState_t matchState) {
        mMatchState = matchState;
    }

    public Command GetAutonomousCommand () {
        return mAutoChooser.getSelected();
    }

    // Smartdashboard output
    public void UpdateSmartDashboard() {
        SmartDashboard.putNumber( "Pressure Sensor (PSI)", mPressureSensor.GetPressureInPSI() );
        SmartDashboard.putBoolean( "Photoeye", mPhotoeye.IsPhotoeyeClosed() );
        mDrivetrain.OutputSmartDashboard();
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
                          mPDP.getCurrent( DRIVETRAIN.LEFT_MASTER_ID )
                          );
    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    private void ConfigureButtonBindings () {
        mDriverJoystickThrottleButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        mDriverJoystickTurnButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
    }

    //-----------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------
    
    /**
    * This is the robot container class consructor.  It is setup for injecting the dependencies in order to allow for
    * mocking those dependencies during unit-testing.
    * @param driverJoystickThrottle Joystick driver joystick throttle
    * @param driverJoystickThrottleButton JoystickButton driver joystick throttle button
    * @param driverJoystickTurn Joystick driver joystick turn
    * @param driverJoystickTurnButton JoystickButton driver joystick turn button
    * @param pressureSensor PressureSensor analog pressure sensor
    * @param photoeye Photoeye digital photo-eye
    * @param powerDistributionPanel PowerDistributionPanel power distribution panel
    */
    public RobotContainer ( Joystick driverJoystickThrottle, JoystickButton driverJoystickThrottleButton,
                            Joystick driverJoystickTurn, JoystickButton driverJoystickTurnButton,
                            PressureSensor pressureSensor, Photoeye photoeye,
                            PowerDistributionPanel powerDistributionPanel ) {
        mDriverJoystickThrottle = driverJoystickThrottle;
        mDriverJoystickThrottleButton = driverJoystickThrottleButton;
        mDriverJoystickTurn = driverJoystickTurn;
        mDriverJoystickTurnButton = driverJoystickTurnButton;
        mPressureSensor = pressureSensor;
        mPhotoeye = photoeye;
        mPDP = powerDistributionPanel;
        mPDP.clearStickyFaults();
        ConfigureButtonBindings();
        mDrivetrain.setDefaultCommand( new TeleopDrive( mDrivetrain, mDriverJoystickThrottle, mDriverJoystickTurn ) );
        mAutoChooser.setDefaultOption( "Auto 1", new Auto1( mDrivetrain ) );
        mAutoChooser.addOption( "Auto 2", new Auto2( mDrivetrain ) );
        SmartDashboard.putData( "Auto Chooser", mAutoChooser );
        mMatchState = MatchState_t.robotInit;
        SolenoidBase.clearAllPCMStickyFaults( HARDWARE.PCM_ID );
    }

    /**
    * This is methods calls the robot container consructor and creates the robot container object.
    * @see {@link frc.robot.lib.drivers.Photoeye}
    * @see {@link frc.robot.lib.drivers.PressureSensor}
    */   
    public static RobotContainer Create () {
        Joystick driverJoystickThrottle = new Joystick( HARDWARE.DRIVER_JOYSTICK_THROTTLE );
        JoystickButton driverJoystickThrottleButton = new JoystickButton( driverJoystickThrottle, 1 );
        Joystick driverJoystickTurn = new Joystick( HARDWARE.DRIVER_JOYSTICK_TURN );
        JoystickButton driverJoystickTurnButton = new JoystickButton( driverJoystickTurn, 1 );
        PressureSensor pressureSensor = new PressureSensor( PRESSURE_SENSOR.ANALOG_CHANNEL,
                                                            PRESSURE_SENSOR.VOLTS_AT_ZERO_PRESSURE,
                                                            PRESSURE_SENSOR.PSI_PER_VOLT );
        Photoeye photoeye = new Photoeye( HARDWARE.PHOTOEYE_DIGITAL_CHANNEL );
        PowerDistributionPanel powerDistributionPanel = new PowerDistributionPanel( HARDWARE.PDP_ID );

        return new RobotContainer( driverJoystickThrottle, driverJoystickThrottleButton, driverJoystickTurn,
                                   driverJoystickTurnButton, pressureSensor, photoeye, powerDistributionPanel );
    }

}
