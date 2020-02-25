package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.MatchState_t;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Robot extends TimedRobot {
    private final Logger mLogger = LoggerFactory.getLogger( Robot.class );
    private RobotContainer mRobotContainer;
    private Command mAutonomousCommand;

    @Override
    public void robotInit () {
        mLogger.info("<=========== ROBOT INIT ===========>");
        mRobotContainer = RobotContainer.Create();
        mRobotContainer.SetMatchState( MatchState_t.robotInit );
        mRobotContainer.LogRobotDataHeader( mLogger );
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
    }

    @Override
    public void robotPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.robotPeriodic ) {
            mLogger.info("<=========== ROBOT PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.robotPeriodic );
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.UpdateSmartDashboard();
    }

    @Override
    public void disabledInit () {
        mLogger.info( "<=========== DISABLED INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.disabledInit );
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();        
    }

    @Override
    public void disabledPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.robotPeriodic ) {
            mLogger.info("<=========== DISABLED PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.disabledPeriodic );
            mRobotContainer.LogRobotDataToRoboRio( mLogger );
            mRobotContainer.UpdateSmartDashboard();
        }
    }

    @Override
    public void autonomousInit () {
        mLogger.info( "<=========== AUTONOMOUS INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.autonomousInit );
        mAutonomousCommand = mRobotContainer.GetAutonomousCommand();
        if (mAutonomousCommand != null) {
            mAutonomousCommand.schedule();
            mLogger.info( "Starting autonomous command {}", mAutonomousCommand.getName() );
        }
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();         
    }

    @Override
    public void autonomousPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.autonomousPeriodic ) {
            mLogger.info("<=========== AUTONOMOUS PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.autonomousPeriodic );
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
    }

    @Override
    public void teleopInit () {
        mLogger.info( "<=========== TELEOP INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.teleopInit );
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();          
    }

    @Override
    public void teleopPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.teleopPeriodic ) {
            mLogger.info("<=========== TELEOP PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.teleopPeriodic );
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
    }

    @Override
    public void testInit () {
        mLogger.info( "<=========== TEST INIT ===========>" );
        CommandScheduler.getInstance().cancelAll();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
    }

    @Override
    public void testPeriodic () {
       
    }
}
