// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.ControllConstants;
import frc.robot.Util.IO;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String defaultAuto = "Default";
  private static final String customAuto = "My Auto";
  private static final String Balance = "balance";
  private String autoSelected;
  private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Auto Choices");
  private RobotContainer m_RobotContainer;
  private Command m_autonomousCommand;
  private Command m_DriveAuto;
  PneumaticHub m_ph = new PneumaticHub(1);
  private String autoName;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    PathPlannerServer.startServer(5811);

    Logger logger = Logger.getInstance();
    
    m_RobotContainer = new RobotContainer();

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
    // Set up data receivers & replay source
    if (isReal()) {
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      logger.addDataReceiver(new WPILOGWriter(""));
      logger.addDataReceiver(new NT4Publisher());
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit logger
    logger.start();

    // Initialize auto chooser
    chooser.addDefaultOption("Default Auto", defaultAuto);
    chooser.addOption("Bump", customAuto);
    chooser.addOption("bALANCE", Balance);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    //Compressor
    m_ph.enableCompressorAnalog(90,115);
    //swerve stuff
    CommandScheduler.getInstance().run();
    RobotContainer.Swerve.PeriodicSoftwareSwerve();
    RobotContainer.driveBase.periodicHardwareSwerve();

    //armStuff
    RobotContainer.arm.armPeriodic();
  
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {

    autoSelected = chooser.get();


    switch (autoSelected) {
      case customAuto:
        autoName = new String("Bump");
        break;
      case Balance:
      autoName = new String("Balance");
        break;
      case defaultAuto:
      default:
          autoName = new String("SamplePath");
        break;
    }

    m_autonomousCommand = m_RobotContainer.getAutonomousCommand(autoName);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    } 


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
      
        /*
       * Position References
       * idle U=-90 L=19
       * Mid U=-47 L=52
       * High U=-7 L=20
       * Intake U=-74 L=-8
       */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_DriveAuto = m_RobotContainer.MatchAuto();
    //drive Stuff
    double Leftx = IO.getLeftX()*MathUtil.applyDeadband(IO.getRightTrigger(), ControllConstants.DeadBandTriger);
    double Lefty = IO.getLeftY()*MathUtil.applyDeadband(IO.getRightTrigger(), ControllConstants.DeadBandTriger);
   
    if (IO.getRightBumperPressed()) {
      RobotContainer.inmtchAuto.runAuto(RobotContainer.inmtchato.autoSelect());;
    } else {
      RobotContainer.Swerve.absoluteDrive(Leftx, Lefty, IO.getRightX(), IO.getRightY(), false, IO.getBackPressed(), IO.getStartPressed(), IO.getPanic(), IO.getYbutton());
    }

    //ARM   
    RobotContainer.arm.SetPointMode();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
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
