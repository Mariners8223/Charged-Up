package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.Drivebase;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.dashboardUtil.TimerWidget;
import frc.util.dashboardUtil.TimerWidget.Mode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private TimerWidget widget;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    Pneumatics.getInstance();
    PathPlannerServer.startServer(5811);
    Logger logger = Logger.getInstance();
    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    if (Constants.currentMode == Constants.Mode.SIM || Constants.currentMode == Constants.Mode.REAL) {
      logger.addDataReceiver(new WPILOGWriter(""));
      logger.addDataReceiver(new NT4Publisher());
    } else if (Constants.currentMode == Constants.Mode.REPLAY) {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      logger.setReplaySource(new WPILOGReader(logPath));
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    // Start AdvantageKit logger
    logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    SmartDashboard.putNumber("angle", 0);
    SmartDashboard.putNumber("length", 0);
    robotContainer = new RobotContainer();
    Pneumatics.getInstance().enableCompressor();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (widget == null)
    {
      widget = new TimerWidget("Auto", 15, Mode.AUTO);
      SmartDashboard.putData("ExampleTimer", widget);
    }
    else
    {
      widget.setMode(Mode.AUTO);
      widget.setName("Auto");
      widget.setDuration(15); // 15s
      widget.reset();
    }

    autonomousCommand = robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (widget == null)
    {
      widget = new TimerWidget("Tele-op", 15, Mode.TELEOP);
      SmartDashboard.putData("ExampleTimer", widget);
    }
    else
    {
      widget.setMode(Mode.TELEOP);
      widget.setName("Tele-op");
      widget.setDuration(135); // 2m 15s
      widget.reset();
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    Drivebase.getInstance().setDefaultCommand(new RunCommand(() -> {
      Drivebase.getInstance().drive(RobotContainer.calculateDeadband(RobotContainer.getCotroller().getRawAxis(0)) * SwerveModuleConstants.freeSpeedMetersPerSecond,
          RobotContainer.calculateDeadband(-RobotContainer.getCotroller().getRawAxis(1)) * SwerveModuleConstants.freeSpeedMetersPerSecond, RobotContainer.calculateDeadband(RobotContainer.getCotroller().getRawAxis(2)) * 10);
    }, Drivebase.getInstance()));



    Pneumatics.getInstance().enableCompressor();
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}