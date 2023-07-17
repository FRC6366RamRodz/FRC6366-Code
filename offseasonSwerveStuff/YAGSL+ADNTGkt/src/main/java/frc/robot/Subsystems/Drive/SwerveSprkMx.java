// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class SwerveSprkMx implements SwerveIO {
    private final SwerveDrive swerveDrive;

    
    public SwerveSprkMx(File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public SwerveSprkMx(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg){
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    //getters

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        double xinput = Math.pow(xInput, 3);
        double yinput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xinput, yinput, headingX, headingY, getHeading().getRadians());
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }



    public SwerveDriveConfiguration getSwerveDriveConfig() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolomonicPose) {
        swerveDrive.resetOdometry(initialHolomonicPose);
    }

    public SwerveModulePosition[] swerveModulePosition() {
        return swerveDrive.getModulePositions();
    }

    public void periodicTask() {
        swerveDrive.updateOdometry();
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
    }

}
