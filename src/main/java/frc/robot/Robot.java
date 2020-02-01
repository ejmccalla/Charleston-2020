package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Robot extends TimedRobot {
    private final Logger mLogger = LoggerFactory.getLogger( Robot.class );
    private RobotContainer mRobotContainer;

    @Override
    public void robotInit () {
        mLogger.info("<=========== ROBOT INIT ===========>");
        mRobotContainer = new RobotContainer();
        mRobotContainer.LogRobotDataHeader( mLogger );
    }

    @Override
    public void robotPeriodic () {
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit () {
        mLogger.info( "<=========== DISABLED INIT ===========>" );
    }

    @Override
    public void disabledPeriodic () {
    }

    @Override
    public void autonomousInit () {
        mLogger.info( "<=========== DISABLED INIT ===========>" );
        // mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        // if (mAutonomousCommand != null) {
        //     mAutonomousCommand.schedule();
        // }
    }

    @Override
    public void autonomousPeriodic () {}

    @Override
    public void teleopInit () {
        mLogger.info( "<=========== TELEOP INIT ===========>" );
        // if (mAutonomousCommand != null) {
        //     mAutonomousCommand.cancel();
        // }
    }

    @Override
    public void teleopPeriodic () {}

    @Override
    public void testInit () {
        mLogger.info( "<=========== TEST INIT ===========>" );
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic () {}
}
