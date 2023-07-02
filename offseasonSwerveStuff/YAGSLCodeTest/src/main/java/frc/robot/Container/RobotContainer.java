// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Container;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Subsystems.SubSwerveTrain;
import frc.robot.Subsystems.SwerveDrive;

/** Add your docs here. */
public class RobotContainer {
    public static final SwerveDrive SwerveDrive = new 
    SwerveDrive(new SubSwerveTrain(new File(Filesystem.getDeployDirectory(), "swerve")));
}
