// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import java.io.File;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;

/** Add your docs here. */
public class SubSwerveTrain implements SwerveDriveIO {
      /**
   * Swerve drive object.
   */
  private final SwerveDrive       swerveDrive;
  /**
   * The auto builder for PathPlanner, there can only ever be one created so we save it just incase we generate multiple
   * paths with events.
   */
  

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */


    public SubSwerveTrain(File directory)
    {
      // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
      try
      {
        swerveDrive = new SwerveParser(directory).createSwerveDrive();
      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }
    }

      /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */

    public SubSwerveTrain(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg);
  }


  /**
   * 
   * @param translation
   * @param rotation
   * @param fieldRelative
   * @param isOpenLoop
   */
  @Override
  public void runSwerve(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

}
