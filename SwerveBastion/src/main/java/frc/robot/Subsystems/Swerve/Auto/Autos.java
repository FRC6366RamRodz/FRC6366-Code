// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Auto;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Swerve.SwerveSparkMax;
import frc.robot.Util.GeomUtil;
import frc.robot.Util.ControllConstants.Auton;

/** Add your docs here. */
public final class Autos {
    private SwerveSparkMax swerve;

    public Autos(SwerveSparkMax swerve) {
        this.swerve = swerve;
    }

    public static CommandBase startAuto(SwerveSparkMax swerve, String path) {
        boolean onTheFly = false;
        List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup(path, new PathConstraints(4, 3));
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getPose2d, swerve::resetOdometry, new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), swerve::setChasisSpeeds, null, true);

            return Commands.sequence(autoBuilder.fullAuto(example1));
    }

    public static CommandBase inMatchAuto(SwerveSparkMax swerve, DummySubsystem sub, Pose2d inmtchPose) {
        PathPlannerTrajectory example;

        example = PathPlanner.generatePath(

                new PathConstraints(4, 2), new PathPoint(swerve.getPose2d().getTranslation(), swerve.getPose2d().getRotation(), swerve.getHeading() ), 
                new PathPoint( inmtchPose.getTranslation(), swerve.getHeading(), inmtchPose.getRotation()));

                return Commands.sequence(new FollowTrajectory(swerve, example, false, sub));
    }


    private final ProfiledPIDController driveController = new ProfiledPIDController(5, 3, 0.5, new TrapezoidProfile.Constraints(4, 3), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(1.8, 0, 0.01, new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)), 0.02);
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private Translation2d lastSetpointTranslation;
    
    public void runAuto(Pose2d poseSupplier) {

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var currentPose = swerve.getPose2d();
        var targetpose = poseSupplier;
        driveController.reset(currentPose.getTranslation().getDistance(poseSupplier.getTranslation()), Math.min(0.0, -new Translation2d(swerve.getFieldVelocity().vxMetersPerSecond, swerve.getFieldVelocity().vyMetersPerSecond).rotateBy(poseSupplier.getTranslation().minus(swerve.getPose2d().getTranslation()).getAngle().unaryMinus()).getX()));
        lastSetpointTranslation = swerve.getPose2d().getTranslation();
        driveController.setTolerance(0.01);
        thetaController.setTolerance(0.01);

        double currentDistance = currentPose.getTranslation().getDistance(poseSupplier.getTranslation());
        double ffScalar = MathUtil.clamp((currentDistance - 0.2) / (0.8 - 0.2), 0, 1.0);
        driveErrorAbs = currentDistance;

        double driveVelocityScalar = driveController.getSetpoint().velocity * 1 + driveController.calculate(driveErrorAbs, 0.0);
        if (driveController.atGoal()) driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(targetpose.getTranslation(), currentPose.getTranslation().minus(targetpose.getTranslation()).getAngle()).transformBy(GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0)).getTranslation();

        double thetaVelocity = thetaController.getSetpoint().velocity * 1 + thetaController.calculate(currentPose.getRotation().getRadians(), targetpose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetpose.getRotation()).getRadians());
        if (thetaController.atGoal()) thetaVelocity = 0.0;

        var driveVelocity = new Pose2d(new Translation2d(), currentPose.getTranslation().minus(targetpose.getTranslation()).getAngle()).transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0)).getTranslation();

        var y = Math.cos(targetpose.getRotation().getRadians());
        var x = Math.sin(targetpose.getRotation().getRadians());

        
        swerve.run(new Translation2d(MathUtil.clamp(driveVelocity.getX(), -4, 4), MathUtil.clamp(driveVelocity.getY(), -4, 4)), thetaVelocity, true, false);
        //RobotContainer.Swerve.absoluteDrive(driveVelocity.getX(), driveVelocity.getY(), x, y, false, false, false, false, false);


    }


}
