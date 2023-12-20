// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveIO;
import frc.robot.Subsystems.Drive.DriveNEO;
import frc.robot.Subsystems.Drive.DriveSim;

/** Add your docs here. */
public class RobotContainer {
  public static Drive drive;

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
  }
}
