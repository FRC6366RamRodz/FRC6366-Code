// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.DT_SET;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** Add your docs here. */
public class SwerveDrive {
    private SwerveDriveIO io;
    private SubSwerveTrain swerve;


    public SwerveDrive(SwerveDriveIO io) {
        this.io = io;
    }

    public SwerveDrive(SubSwerveTrain swerve) {
        this.swerve = swerve;
    }

    public void SwervePeriodic() {
        swerve.periodic();
    }


    public void teleopDrive(double vX, double vY, double omega, boolean driveMode, boolean isOpenLoop) {
        double xVelocity  = MathUtil.applyDeadband(Math.pow(vX, 3), DT_SET.DedBnd);
        double yVelocity  = MathUtil.applyDeadband(Math.pow(vY, 3), DT_SET.DedBnd);
        double angVelocity = MathUtil.applyDeadband(Math.pow(omega, 3), DT_SET.DedBnd);

        io.runSwerve(new Translation2d(xVelocity * DT_SET.MX_SPD, yVelocity * DT_SET.MX_SPD), angVelocity*DT_SET.MX_SPD, driveMode, isOpenLoop);
    }

    public void absoluteDrive(double vX, double vY, double headingHorizontal, double headingVertical, boolean isOpenLoop) {
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(MathUtil.applyDeadband(vX, DT_SET.DedBnd), MathUtil.applyDeadband(vY, DT_SET.DedBnd), MathUtil.applyDeadband(headingHorizontal, DT_SET.DedBnd), headingVertical);

        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(), Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS), swerve.getSwerveDriveConfig());

        io.runSwerve(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    }
}
