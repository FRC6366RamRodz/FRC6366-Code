// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drive.DummyDriveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSprkMx;
import frc.robot.Util.Constants.Auton;

/** Add your docs here. */
public class FollowTrajectory extends SequentialCommandGroup{
    public FollowTrajectory(DummyDriveSubsystem subsystem, SwerveSprkMx drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {

        addRequirements(subsystem);

        if (resetOdometry)
        {
          drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        }
    
        addCommands(
            new PPSwerveControllerCommand( trajectory, drivebase::getPose, Auton.xAutoPID.createPIDController(), Auton.yAutoPID.createPIDController(), Auton.angleAutoPID.createPIDController(), drivebase::setChassisSpeeds, subsystem)
                   );
       
    }
  
}
