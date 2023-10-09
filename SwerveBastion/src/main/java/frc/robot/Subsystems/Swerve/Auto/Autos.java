// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Subsystems.Swerve.SwerveSparkMax;
import frc.robot.Subsystems.Swerve.Auto.AutoPassthrough.IntakeDown;
import frc.robot.Util.GeomUtil;
import frc.robot.Util.RobotContants;
import frc.robot.Util.ControllConstants.Auton;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** Add your docs here. */
public final class Autos {
    private SwerveSparkMax swerve;

    public Autos(SwerveSparkMax swerve) {
        this.swerve = swerve;
    }

    public static CommandBase startAuto(SwerveSparkMax swerve, String path) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("IntakeDown", new AutoPassthrough());
        List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup(path, new PathConstraints(4, 3));
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getPose2d, swerve::resetOdometry, new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), swerve::setChasisSpeeds, eventMap, true);

            
            return Commands.sequence(autoBuilder.fullAuto(example1));
           
        
    }

    public String autoMarkers(String path) {
        PathPlannerTrajectory example1 = PathPlanner.loadPath(path, new PathConstraints(4, 3));;
        return example1.getMarkers().toString();
    }

    public static CommandBase inMatchAuto(SwerveSparkMax swerve, DummySubsystem sub, Pose2d inmtchPose) {
        PathPlannerTrajectory example;

        example = PathPlanner.generatePath(

                new PathConstraints(4, 2), new PathPoint(swerve.getPose2d().getTranslation(), swerve.getPose2d().getRotation(), swerve.getHeading() ), 
                new PathPoint( inmtchPose.getTranslation(), swerve.getHeading(), inmtchPose.getRotation()));


                return Commands.sequence(new FollowTrajectory(swerve, example, false, sub));
    }

    //private final ProfiledPIDController driveController = new ProfiledPIDController(4, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.feetToMeters(12), Units.feetToMeters(12)), 0.02);
    private final PIDController driveController = new PIDController(5, 0.0, 0.02);
    //private final ProfiledPIDController thetaController = new ProfiledPIDController(1.8, 0, 0.01, new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)), 0.02);
    private final PIDController thetaController = new PIDController(1.8, 0, 0.01);
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private Translation2d lastSetpointTranslation;
    
    public void runAuto(Pose2d poseSupplier) {

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var currentPose = swerve.getPose2d();
        var targetpose = poseSupplier;
        lastSetpointTranslation = swerve.getPose2d().getTranslation();
        driveController.setTolerance(0.0);
        thetaController.setTolerance(0.0);

        double currentDistance = currentPose.getTranslation().getDistance(poseSupplier.getTranslation());
        double ffScalar = MathUtil.clamp((currentDistance - 0.2) / (0.8-0.2), 0, 1.0);
        driveErrorAbs = currentDistance;

        double driveVelocityScalar = driveController.getSetpoint() * ffScalar + driveController.calculate(driveErrorAbs, 0.0);
        if (driveController.atSetpoint()) driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(targetpose.getTranslation(), currentPose.getTranslation().minus(targetpose.getTranslation()).getAngle()).transformBy(GeomUtil.translationToTransform(driveController.getSetpoint(), 0.0)).getTranslation();

        double thetaVelocity = thetaController.getSetpoint() * 1 + thetaController.calculate(swerve.getHeading().getRadians(), targetpose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(swerve.getHeading().minus(targetpose.getRotation()).getRadians());
        if (thetaController.atSetpoint()) thetaVelocity = 0.0;

        var driveVelocity = new Pose2d(new Translation2d(), currentPose.getTranslation().minus(targetpose.getTranslation()).getAngle()).transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0)).getTranslation();

        var y = Math.cos(targetpose.getRotation().getRadians());
        var x = Math.sin(targetpose.getRotation().getRadians());

        
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(MathUtil.clamp(driveVelocity.getX(), -0.9, 0.9), MathUtil.clamp(driveVelocity.getY(), -0.9, 0.9) , x, y);

        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose2d(), 0.13, RobotContants.RobotMass, List.of(RobotContants.Chassis), swerve.getSwerveDriveConfig());
        
        swerve.run(translation, thetaVelocity, true, false);


    }


}
