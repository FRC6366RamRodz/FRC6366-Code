// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive.Auto;

import java.io.IOException;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.DummyDriveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSprkMx;
import frc.robot.Util.Constants.Auton;

/** Add your docs here. */
public final class Autos {
    private static AprilTagFieldLayout aprilTagField = null;

    /**
     * @param swerve
     */
    public static CommandBase exampleAuto(SwerveSprkMx swerve, DummyDriveSubsystem sub) { 
        boolean onTheFly = false;
        PathPlannerTrajectory example;
        if (onTheFly) {
            example = PathPlanner.generatePath(

                new PathConstraints(4, 3), new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), 
                new PathPoint(new Translation2d(3, 5), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)), 
                new PathPoint(new Translation2d(5, 5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

            
        } else {
            List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup("SamplePath", new PathConstraints(4, 3));
            
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getPose, swerve::resetOdometry, new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), swerve::setChassisSpeeds, null, true);  

            return Commands.sequence(autoBuilder.fullAuto(example1));
        }

        return Commands.sequence(new FollowTrajectory(sub, swerve, example, true));
        
    }

    /**
     * {@link FollowTrajectory}
     * @param swerve
     * @param id
     * @param rotation
     * @param holonomicRotation
     * @param offset
     * @return {@link FollowTrajectory}
     */
    public static CommandBase DriveToAprilTag(DummyDriveSubsystem sub, SwerveSprkMx swerve, int id, Rotation2d rotation, Rotation2d holonomicRotation, Translation2d offset) {
        if (aprilTagField == null) {
              try {
                aprilTagField = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
            PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4, 3), false, 
                                                                PathPoint.fromCurrentHolonomicState(swerve.getPose(), swerve.getRobotVelocity()), 
                                                                new PathPoint(aprilTagField.getTagPose(1).get().getTranslation().toTranslation2d().plus(new Translation2d(0,0)), new Rotation2d(0), new Rotation2d(0)));

        return Commands.sequence(new FollowTrajectory(sub, swerve, path, true));            

    }

}

