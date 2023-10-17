// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.io.File;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class SwerveSparkMax implements SwerveIO {
    private final swervelib.SwerveDrive SwerveDrive;
    private SwerveAutoBuilder autoBuilder = null;

//instantiate swerve
    public SwerveSparkMax(File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        try {
            SwerveDrive = new SwerveParser(directory).createSwerveDrive();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

//swerve creater    
    public SwerveSparkMax(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerConfiguration){
        SwerveDrive = new SwerveDrive(driveCfg, controllerConfiguration);
    }


// getters

    public Rotation2d getHeading() {
        return SwerveDrive.getYaw();
    }

    public Pose2d LimeOffsetPose2d(Transform2d other) {
        return SwerveDrive.getPose().transformBy(other);
    }

    public Pose2d getPose2d() {
        return SwerveDrive.getPose();
    }

    public SwerveDriveKinematics getKinematics() {
        return SwerveDrive.kinematics;
    }

    public ChassisSpeeds getFieldVelocity() {
        return SwerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return SwerveDrive.getRobotVelocity();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double haedingX, double headingY) {
        double xinput = Math.pow(xInput, 3);
        double yinput = Math.pow(yInput, 3);
        return SwerveDrive.swerveController.getTargetSpeeds(xinput, yinput, haedingX, headingY, getHeading().getRadians());
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return SwerveDrive.getModulePositions();
    }

    public SwerveDriveConfiguration getSwerveDriveConfig() {
        return SwerveDrive.swerveDriveConfiguration;
    }


//setter and senders
    public void setSwerveModulePositions() {
        SwerveDrive.setModuleStates(null, false);
    }

    public void resetOdometry(Pose2d initialHolomonicPose){
        SwerveDrive.resetOdometry(initialHolomonicPose);
    }

    public void lockPose() {
        SwerveDrive.lockPose();
    }

    public void setChasisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        SwerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro() {
        SwerveDrive.zeroGyro();
    }


//controlls loops
    public void periodicHardwareSwerve() {
        SwerveDrive.updateOdometry();
        SwerveDrive.invertOdometry = false;
    }

    @Override
    public void updateInputs(SwerveIoInputs inputs) {
    }

    @Override
    public void run(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }



    //autonomous

}
