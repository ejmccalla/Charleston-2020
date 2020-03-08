package frc.robot.commands;

import java.lang.Math;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVER;
import frc.robot.subsystems.Drivetrain;

public class TurnToTarget extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final Joystick mDriverThrottle;
    
    @Override
    public void initialize () {
        mDrivetrain.StartTurnToTarget();
    }

    @Override
    public void execute() {
        if ( Math.abs( mDriverThrottle.getX() ) < DRIVER.QUICKTURN_THRESHOLD ) {
            mDrivetrain.SetLimelightVisionControllerOutput( 0.0, true );
        } else {
            mDrivetrain.SetLimelightVisionControllerOutput( mDriverThrottle.getX(), false );
        }
    }

    @Override
    public void end ( boolean interrupted ) {
        mDrivetrain.EndLimelightCommand();
    }

    @Override
    public boolean isFinished () {
      return false;
    }

    public TurnToTarget ( Drivetrain drivetrain, Joystick driverThrottle ) {
        mDrivetrain = drivetrain;
        mDriverThrottle = driverThrottle;
        addRequirements(mDrivetrain);
    }

}
