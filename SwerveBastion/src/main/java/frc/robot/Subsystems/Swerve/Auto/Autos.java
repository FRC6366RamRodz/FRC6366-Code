// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Swerve.SwerveSparkMax;
import frc.robot.Util.ControllConstants.Auton;

/** Add your docs here. */
public final class Autos {
    public static CommandBase startAuto(SwerveSparkMax swerve, String path) {
        boolean onTheFly = false;
        List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup(path, new PathConstraints(4, 3));
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getPose2d, swerve::resetOdometry, new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), swerve::setChasisSpeeds, null, true);

            return Commands.sequence(autoBuilder.fullAuto(example1));
    }

    public static CommandBase inMatchAuto(SwerveSparkMax swerve, DummySubsystem sub) {
        PathPlannerTrajectory example;

        example = PathPlanner.generatePath(

                new PathConstraints(4, 2), new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), 
                new PathPoint(new Translation2d(3, 5), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)));

                return Commands.sequence(new FollowTrajectory(swerve, example, false, sub));
    }


}
