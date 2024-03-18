// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveIO;
import frc.robot.Subsystems.Drive.DriveNEO;
import frc.robot.Subsystems.Drive.DriveSim;

/** Add your docs here. */
public class RobotContainer {
  public static Drive drive;
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(new DriveNEO()); // real hardware file
        break;

      case SIM:
        drive = new Drive(new DriveSim()); // sim mode
        break;

      default:
        drive = new Drive(new DriveIO() {}); // replay mode
        break;
    }

  // Set up auto routines
  autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
}

/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
public Command getAutonomousCommand() {
  return autoChooser.get();
}
}
