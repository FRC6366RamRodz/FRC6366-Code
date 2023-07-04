// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Subsystems.Drive.SwerveSprkMx;

/** Add your docs here. */
public class RobotContainer {
    
    private static final SwerveSprkMx driveBase = new 
    SwerveSprkMx(new File(Filesystem.getDeployDirectory(), "swerve"));

    public static final Swerve Swerve = new
    Swerve(driveBase, driveBase);
}
