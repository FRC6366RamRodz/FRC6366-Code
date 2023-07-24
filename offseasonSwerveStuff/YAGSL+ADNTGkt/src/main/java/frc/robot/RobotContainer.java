// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Drive.DummyDriveSubsystem;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Subsystems.Drive.SwerveSprkMx;
import frc.robot.Subsystems.Drive.Auto.Autos;
import swervelib.SwerveDrive;


/** Add your docs here. */
public class RobotContainer {

    public RobotContainer() {
        configureBindings();
    }

    XboxController driverXbox = new XboxController(0);

    
    public static final SwerveSprkMx driveBase = new 
    SwerveSprkMx(new File(Filesystem.getDeployDirectory(), "swerve"));

    public static final DummyDriveSubsystem sub = new DummyDriveSubsystem();

    public static final Swerve Swerve = new
    Swerve(driveBase, driveBase);

    public Command getAutonomousCommand() {
        return Autos.exampleAuto(driveBase, sub);
    }

    private void configureBindings() {
    }
}
