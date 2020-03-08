package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToTarget extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final double mTargetDistance;
    
    @Override
    public void initialize () {
        mDrivetrain.StartDriveToTarget( mTargetDistance );
    }

    @Override
    public void execute() {
        mDrivetrain.SetLimelightVisionControllerOutput( false );
    }

    @Override
    public void end ( boolean interrupted ) {
        mDrivetrain.EndLimelightCommand();
    }

    @Override
    public boolean isFinished () {
      return false;
    }

    public DriveToTarget ( Drivetrain drivetrain, double targetDistance ) {
        mDrivetrain = drivetrain;
        mTargetDistance = targetDistance;
        addRequirements(mDrivetrain);
    }

}
