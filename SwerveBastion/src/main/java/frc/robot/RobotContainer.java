// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.ArmSim;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveSparkMax;
import frc.robot.Subsystems.Swerve.Auto.Autos;
import frc.robot.Subsystems.Swerve.Auto.DummySubsystem;

/** Add your docs here. */
public class RobotContainer {

    //Swerve stuff
    public static final SwerveSparkMax driveBase = new 
    SwerveSparkMax(new File(Filesystem.getDeployDirectory(), "swerve"));

    public static final DummySubsystem sub = new DummySubsystem();

    public static final Swerve Swerve = new
    Swerve(driveBase, driveBase);

    public Command getAutonomousCommand(String path) {
        return Autos.startAuto(driveBase, path);
    }

    public Command MatchAuto() {
        return Autos.inMatchAuto(driveBase, sub);
    }

    public static final Arm arm = new Arm(new ArmSim());
}
