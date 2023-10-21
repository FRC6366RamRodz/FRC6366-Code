// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.CTREPCMJNI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
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
  private String autoSelected;
  private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>("Auto Choices");
  private boolean tank, strafe, stinger, slow;
  public Compressor compressoor = new Compressor(PneumaticsModuleType.CTREPCM);
  public Timer timee = new Timer();
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
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
    if (isReal()) {
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit logger
    logger.start();

    // Initialize auto chooser
    chooser.addDefaultOption("Default Auto", defaultAuto);
    chooser.addOption("My Auto", customAuto);

    tank = false;
    stinger = false;
    strafe = false;
    slow = false;
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    RobotContainer.TSS.periodicDrive();
    compressoor.enableDigital();
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.get();
    System.out.println("Auto selected: " + autoSelected);
    timee.stop();
    timee.reset();
    tank = false;
    stinger = false;
    strafe = false;
    slow = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    timee.start();
    switch (autoSelected) {
      case customAuto:
        if (timee.get()<4) {
          RobotContainer.TSS.auto();
        } else {
          RobotContainer.TSS.Tank(0, 0);
        }

        break;
      case defaultAuto:
      default:
      if (timee.get()<4) {
        RobotContainer.TSS.auto();
      } else {
        RobotContainer.TSS.Tank(0, 0);
      }
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    tank = false;
    stinger = false;
    strafe = false;
    slow = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (IO.getRightBumperPressed()) {
      tank = false;
      stinger = !stinger;
      strafe = false;
    } else if (IO.getLeftBumperPressed()) {
      tank = false;
      stinger = false;
      strafe = !strafe;
    } else if (IO.getAbuttonPressed()) {
      tank = !tank;
      stinger = false;
      strafe = false;
    }

    if (IO.getXButtonPressed()) {
      slow = !slow;
    }



    RobotContainer.TSS.TSSLogic(stinger, strafe, tank, slow);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    tank = false;
    stinger = false;
    strafe = false;
    slow = false;
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
