// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.SwerveSparkMax;
import frc.robot.Util.ControllConstants.Auton;

/** Add your docs here. */
public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(SwerveSparkMax driveBase, PathPlannerTrajectory trajectory, boolean resetOdometry) {

        if (resetOdometry) {
            driveBase.resetOdometry(trajectory.getInitialHolonomicPose());
        }

        addCommands(
            new PPSwerveControllerCommand(trajectory, driveBase::getPose2d, Auton.xAutoPID.createPIDController(), Auton.yAutoPID.createPIDController(), Auton.angleAutoPID.createPIDController(), driveBase::setChasisSpeeds, null)
        );
    }
}
