// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import java.io.File;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class SwerveSprkMx implements SwerveIO {
    private final SwerveDrive swerveDrive;
    private SwerveAutoBuilder autoBuilder = null;

    
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



    //getters

    public void postTrajectory(Trajectory trajectory)
    {
      swerveDrive.postTrajectory(trajectory);
    }
    
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
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

    public void lock(){
        swerveDrive.lockPose();
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

    public void setSwerveModulePositions() {
        swerveDrive.setModuleStates(null, false);
    }

    public SwerveModulePosition[] swerveModulePosition() {
        return swerveDrive.getModulePositions();
    }

    public void periodicTask() {
        swerveDrive.updateOdometry();
        swerveDrive.invertOdometry = false;
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    /**
     * @param path
     * @param constraints {@link PathConstraints} for {@link com.pathplanner.lib.PathPlanner#loadPathGroup}
     * @param eventMap {@link java.util.HashMap}
     * @param translation {@link PIDConstants}
     * @param rotation {@link PIDConstants}
     * @param useAllianceColor 
     * @return
     */
    public Command createPathPlannerCommand(String path, PathConstraints constraints, Map<String, Command> eventMap, PIDConstants translation, PIDConstants rotation, boolean useAllianceColor) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);

        if (autoBuilder == null)
        {
          autoBuilder = new SwerveAutoBuilder(swerveDrive::getPose, swerveDrive::resetOdometry, translation, rotation, swerveDrive::setChassisSpeeds, eventMap, useAllianceColor, null);
        }
    
        return autoBuilder.fullAuto(pathGroup);
    }
  }

