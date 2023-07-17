// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Subsystems.Drive.SwerveSprkMx;
import frc.robot.Util.Constants.Auton;

/** Add your docs here. */
public class FollowTrajectory {
    public FollowTrajectory(SwerveSprkMx driveBase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        if (resetOdometry)
        {
            driveBase.resetOdometry(trajectory.getInitialHolonomicPose());
        }

        addCommands(
            new PPSwerveControllerCommand(trajectory, driveBase::getPose, Auton.xAutoPID.createPIDController(), Auton.yAutoPID.createPIDController(), Auton.angleAutoPID.createPIDController(), driveBase::setChassisSpeeds, null)
        );
    }

    private void addCommands(PPSwerveControllerCommand ppSwerveControllerCommand) {
    }
}
