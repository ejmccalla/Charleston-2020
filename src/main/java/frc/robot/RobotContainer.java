package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.drivers.PressureSensor;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.TeleopDrive;
import org.slf4j.Logger;

public class RobotContainer {
    
    // Hardware
    private final XboxController mDriverXbox = new XboxController( Constants.DRIVER_XBOX );
    //private final XboxController mOperatorXbox = new XboxController( Constants.OPERATOR_XBOX );
    private final Joystick mDriverJoystickThrottle = new Joystick( Constants.DRIVER_JOYSTICK_THROTTLE );
    private final JoystickButton mDriverJoystickThrottleButton = new JoystickButton( mDriverJoystickThrottle, 1 );
    private final Joystick mDriverJoystickTurn = new Joystick( Constants.DRIVER_JOYSTICK_TURN );
    private final JoystickButton mDriverJoystickTurnButton = new JoystickButton( mDriverJoystickTurn, 1 );
    private final PressureSensor mPressureSensor = new PressureSensor( Constants.PRESSURE_SENSOR_ANALOG_CHANNEL,
                                                                       Constants.PRESSURE_SENSOR_VOLTS_AT_ZERO_PRESSURE,
                                                                       Constants.PRESSURE_SENSOR_PRESSURE_PER_VOLT );
    private final Photoeye mPhotoeye = new Photoeye( Constants.PHOTOEYE_ANALOG_CHANNEL );

    // Subsystems
    private final Drivetrain mDrivetrain = Drivetrain.create();
    
    // Commands
    private final TeleopDrive mTeleopDrive = new TeleopDrive( mDrivetrain, mDriverXbox, mDriverJoystickThrottle, mDriverJoystickTurn );

    // Logging
    public void LogRobotDataHeader (Logger robotLogger) {
        robotLogger.info( "Time,Pressure (PSI),Photoeye Closed" );
    }   
   
    public void LogRobotDataToRoboRio (Logger robotLogger) {
        SmartDashboard.putNumber( "Pressure Sensor (PSI)", mPressureSensor.GetPressureInPSI() );
        SmartDashboard.putBoolean( "Photoeye", mPhotoeye.IsPhotoeyeClosed() );
        robotLogger.info( "{},{},{}", Timer.getFPGATimestamp(), mPressureSensor.GetPressureInPSI(), mPhotoeye.IsPhotoeyeClosed() );
    }

    // Button mappings
    private void ConfigureButtonBindings () {
        mDriverJoystickThrottleButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetHighGear( !mDrivetrain.IsHighGear() ), mDrivetrain ) );
        mDriverJoystickTurnButton.whenPressed( new InstantCommand( () -> mDrivetrain.SetReversed( !mDrivetrain.IsReversed() ), mDrivetrain ) );
    }
 
    public RobotContainer () {
        ConfigureButtonBindings();
        mDrivetrain.setDefaultCommand(mTeleopDrive);
    }
  


}
