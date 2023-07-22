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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.Drive.SwerveSprkMx;
import frc.robot.Util.Constants.Auton;

/** Add your docs here. */
public final class Autos {
    private SwerveSprkMx swerve;
    
    private static AprilTagFieldLayout aprilTagField = null;

    public Autos(SwerveSprkMx swerve){
        this.swerve = swerve;
    }

    /**
     * @param swerve
     */
    public void exampleAuto() { 
        boolean onTheFly = false;
        PathPlannerTrajectory example;

        if (onTheFly) {


            example = PathPlanner.generatePath(

                new PathConstraints(4, 3), new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), 
                new PathPoint(new Translation2d(3, 5), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)), 
                new PathPoint(new Translation2d(5, 5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

                new FollowTrajectory(swerve, example, true);

            
        } else {
            List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup("SamplePath", new PathConstraints(4, 3));
            
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getPose, swerve::resetOdometry, new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), swerve::setChassisSpeeds, null, true);  
            
            autoBuilder.fullAuto(example1);

        }

        
        
    }


  /**
   * Create a {@link FollowTrajectory} command to go to the April Tag from the current position.
   *
   * @param swerve            Swerve drive subsystem.
   * @param id                April Tag ID to go to.
   * @param rotation          Rotation to go to.
   * @param holonomicRotation Holonomic rotation to be at.
   * @param offset            Offset from the April Tag.
   * @return {@link FollowTrajectory} command. May return null if cannot load field.
   */
    public void driveToAprilTag(int id, Rotation2d rotation, Rotation2d holomonicRotation, Translation2d offset) {
        if (aprilTagField == null) {
            try {
                aprilTagField = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            } catch (Exception ignored) {
                aprilTagField = null;
            }
        }
        PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4, 3), false,
        PathPoint.fromCurrentHolonomicState(swerve.getPose(), swerve.getRobotVelocity()), new PathPoint(aprilTagField.getTagPose(id).get() .getTranslation() .toTranslation2d().plus(offset), rotation, holomonicRotation));

        new FollowTrajectory(swerve, path, false);
    }

}

