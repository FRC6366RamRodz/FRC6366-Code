// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.SwerveSprkMx;
import frc.robot.Util.Constants.Auton;

/** Add your docs here. */
public final class Autos {
    
    private static AprilTagFieldLayout aprilTagField = null;

    /**
     * @param swerve
     */
    public static CommandBase exampleAuto(SwerveSprkMx swerve) { 
        boolean onTheFly = false;
        PathPlannerTrajectory example;
        if (onTheFly) {
            example = PathPlanner.generatePath(

                new PathConstraints(4, 3), new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), 
                new PathPoint(new Translation2d(3, 5), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)), 
                new PathPoint(new Translation2d(5, 5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

            
        } else {
            List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup("SamplePath", new PathConstraints(4, 3));
            
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getPose, swerve::resetOdometry, new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), swerve::setChassisSpeeds, null, false);  
            
            autoBuilder.fullAuto(example1);

            return Commands.sequence(autoBuilder.fullAuto(example1));
        }

        return Commands.sequence(new FollowTrajectory(swerve, example, true));
        
    }

}

