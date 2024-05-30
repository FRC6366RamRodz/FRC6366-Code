// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private Timer isPressed = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;

    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

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
    RobotContainer.shooter.ShooterPeriodic();//timed style needs a periodic run here

    Logger.recordOutput("BackLeftCamAlive", RobotContainer.BackLeftcam.isCameraConnected());//camera logging.
    Logger.recordOutput("BackRightCamAlive", RobotContainer.BackRightcam.isCameraConnected());
    Logger.recordOutput("FrontRightCamAlive", RobotContainer.FrontRightcam.isCameraConnected());
    Logger.recordOutput("FrontLeftCamAlive", RobotContainer.FrontLeftcam.isCameraConnected());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();//identify auton command to run

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();//run auton command
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

      RobotContainer.drive.checkFrontVision();//Check vision pose.


  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel(); //stop the auton command
    }

      autonomousCommand = AutoBuilder.buildAuto("DriveToAmp"); //overide autonomous command for tele.

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //button binding for timed style controls
    RobotContainer.shooter.advancedShoot(RobotContainer.io.getOpX(), RobotContainer.io.getOpRB(), RobotContainer.io.getOpRTrigger(), RobotContainer.io.getOPLB(), RobotContainer.io.getOpLTrigger(), RobotContainer.io.getOPB(), RobotContainer.io.getOpA(), RobotContainer.io.getOpY(), RobotContainer.io.getOpRightY(), RobotContainer.io.getOPLYUp(), RobotContainer.io.getDrLeftBumper(), Math.abs(DriveCommands.pid.getPositionError())  + Math.abs(DriveCommands.pid.getVelocityError()*0.5));
    RobotContainer.io.op.setRumble(RumbleType.kRightRumble, RobotContainer.shooter.LaunchPermision());
    RobotContainer.io.drRumble(RobotContainer.shooter.IntakeRumble());
    RobotContainer.io.drLightRumble(RobotContainer.shooter.lightRumble());

    RobotContainer.drive.checkFrontVision();


    
    //drive to amp stuff, needs a debounce to not loop.
    if (RobotContainer.io.DriveLTPressed() && isPressed.get() < 0.1) {
      isPressed.start();
      autonomousCommand.schedule();
    } else if (!RobotContainer.io.DriveLTPressed()) {
      autonomousCommand.cancel();
      isPressed.stop();
      isPressed.reset();
    }


  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
