package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final Joystick mDriverThrottle;
    private final Joystick mDriverTurn;

    @Override
    public void initialize () {}

    @Override
    public void execute() {
        if ( mDrivetrain.IsReversed() ) {
            mDrivetrain.mDifferentialDrive.curvatureDrive( mDriverThrottle.getX(), mDriverTurn.getY(), false );
        } else {
            mDrivetrain.mDifferentialDrive.curvatureDrive( mDriverThrottle.getX(), mDriverTurn.getY(), false );
        }
    }

    @Override
    public void end ( boolean interrupted ) {}

    @Override
    public boolean isFinished () {
      return false;
    }

    public TeleopDrive ( Drivetrain drivetrain, Joystick driverThrottle, Joystick driverTurn ) {
        mDrivetrain = drivetrain;
        mDriverThrottle = driverThrottle;
        mDriverTurn = driverTurn;
        addRequirements(mDrivetrain);
    }

}
