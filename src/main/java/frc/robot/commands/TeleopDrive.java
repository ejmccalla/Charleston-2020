package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain mDrivetrain;
    private final XboxController mDriverXbox;
    private final Joystick mDriverThrottle;
    private final Joystick mDriverTurn;

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (mDrivetrain.IsDriverControlModeXbox()) {
            // Arcade drive driven off of the left Xbox joystick only 
            SmartDashboard.putNumber("Xbox-Y", mDriverXbox.getY(Hand.kLeft));
            SmartDashboard.putNumber("Xbox-X", mDriverXbox.getX(Hand.kLeft));
            //mDrivetrain.mDifferentialDrive.arcadeDrive(mDriverXbox.getY(Hand.kLeft), mDriverXbox.getX(Hand.kLeft));
        } else {
            // Curvature drive driven off of the left joystick for throttle and the right joystick for turning
            SmartDashboard.putNumber("Joystick-Throttle", mDriverThrottle.getX());
            SmartDashboard.putNumber("Joystick-Turn", mDriverTurn.getY());
            mDrivetrain.mDifferentialDrive.curvatureDrive(mDriverThrottle.getX(), mDriverTurn.getY(), false);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }

    public TeleopDrive(Drivetrain drivetrain, XboxController driverXbox, Joystick driverThrottle, Joystick driverTurn) {
        mDrivetrain = drivetrain;
        mDriverXbox = driverXbox;
        mDriverThrottle = driverThrottle;
        mDriverTurn = driverTurn;
        addRequirements(mDrivetrain);
    }

}
