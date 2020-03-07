package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.MatchState_t;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* The Robot class contains the RobotContainer and data logger objects and simply calls the command-based scheduler to
* run the robot commands from the given robot states.  Also, the data logger is used to log data during while the
* robot is in the various states.
*/
public class Robot extends TimedRobot {
    
    private final Logger mLogger = LoggerFactory.getLogger( Robot.class );
    private RobotContainer mRobotContainer;
    private Command mAutonomousCommand;

    /**
    * This method is called when the robot is first powered up.  This method get called only once.
    */ 
    @Override
    public void robotInit () {
        mLogger.info("<=========== ROBOT INIT ===========>");
        mRobotContainer = RobotContainer.Create();
        mRobotContainer.SetMatchState( MatchState_t.robotInit );
        mRobotContainer.LogRobotDataHeader( mLogger );
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
    }

    /**
    * This method is called when the robot is disabled.  This method get called only once.
    */     
    @Override
    public void disabledInit () {
        mLogger.info( "<=========== DISABLED INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.disabledInit );
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();        
    }

    /**
    * This method is called when the robot is entering autonomous.  This method get called only once.
    */     
    @Override
    public void autonomousInit () {
        mLogger.info( "<=========== AUTONOMOUS INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.autonomousInit );
        mAutonomousCommand = mRobotContainer.GetAutonomousCommand();
        if (mAutonomousCommand != null) {
            mAutonomousCommand.schedule();
            mLogger.info( "Starting autonomous command {}", mAutonomousCommand.getName() );
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();         
    }

    /**
    * This method is called when the robot is entering teleop.  This method get called only once.
    */     
    @Override
    public void teleopInit () {
        mLogger.info( "<=========== TELEOP INIT ===========>" );
        mRobotContainer.SetMatchState( MatchState_t.teleopInit );
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();          
    }

    /**
    * This method is called whenever the robot is in a periodic mode.  This means that both autonomous periodic and
    * teleop periodic will run prior to this code.  The disabled periodic will also run prior to this method.
    */     
    @Override
    public void robotPeriodic () {
    }

    /**
    * This method is called periodically whenever the robot is disabled.
    */     
    @Override
    public void disabledPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.disabledPeriodic ) {
            mLogger.info("<=========== DISABLED PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.disabledPeriodic );
            mRobotContainer.LogRobotDataToRoboRio( mLogger );
            mRobotContainer.UpdateSmartDashboard();
        }
    }

    /**
    * This method is called periodically whenever the robot is in autonomous mode.
    */     
    @Override
    public void autonomousPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.autonomousPeriodic ) {
            mLogger.info("<=========== AUTONOMOUS PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.autonomousPeriodic );
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
    }

    /**
    * This method is called periodically whenever the robot is in teleop mode.
    */   
    @Override
    public void teleopPeriodic () {
        if ( mRobotContainer.GetMatchState() != MatchState_t.teleopPeriodic ) {
            mLogger.info("<=========== TELEOP PERIODIC ===========>");
            mRobotContainer.SetMatchState( MatchState_t.teleopPeriodic );
        }
        CommandScheduler.getInstance().run();
        mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
    }


    /**
    * This method is called when the robot is entering test mode.  This method get called only once.
    */       
    @Override
    public void testInit () {
        mLogger.info( "<=========== TEST INIT ===========>" );
        CommandScheduler.getInstance().enable();
        //mRobotContainer.LogRobotDataToRoboRio( mLogger );
        mRobotContainer.UpdateSmartDashboard();
    }    

    /**
    * This method is called periodically whenever the robot is in test mode.
    */
    @Override
    public void testPeriodic () {
        mRobotContainer.UpdateSmartDashboard();
        CommandScheduler.getInstance().run();
    }

}
