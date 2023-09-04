// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.ControllConstants;
import frc.robot.Util.RobotContants;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** Add your docs here. */
public class Swerve {
    private SwerveIO io;
    private SwerveSparkMax swerve;
    private SwerveIoInputsAutoLogged inputs = new SwerveIoInputsAutoLogged();
    private Pose2d botPose2d;
    private double[] botPoseData;
    double offset = 0;


//instantiate
    public Swerve(SwerveIO io, SwerveSparkMax swerve){
        this.io = io;
        this.swerve = swerve;
    }

    public void PeriodicSoftwareSwerve() {

        //limelight Pose
        if(DriverStation.getAlliance().equals(Alliance.Blue)) {
            botPoseData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } else {
            botPoseData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
        }
        //botPose2d = new Pose2d(botPoseData[0], botPoseData[1], Rotation2d.fromDegrees(botPoseData[5]));
        botPose2d = new Pose2d(botPoseData[0], botPoseData[1], Rotation2d.fromDegrees(botPoseData[5]));

        swerve.periodicHardwareSwerve();
        Logger.getInstance().recordOutput("Odometry", swerve.getPose2d());

        io.updateInputs(inputs);
        Logger.getInstance().processInputs("drive", inputs);

        Logger.getInstance().recordOutput("LimeLight Pose", botPose2d);


        Transform2d offset;
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false) == true) {
            Translation2d OfsetPose = botPose2d.getTranslation().minus(swerve.getPose2d().getTranslation());
            offset = new Transform2d(OfsetPose, swerve.getHeading());
            swerve.LimeOffsetPose2d(offset);
        } else {
        }
       
    }


    public void absoluteDrive(double Vx, double Vy, double headingHorizontal, double headingVertical, boolean isOpenLoop, boolean trimLeft, boolean trimRight, boolean Panic, boolean Lock) {

        

        double vX = MathUtil.applyDeadband(Vy, ControllConstants.DeadBand);
        double vY = MathUtil.applyDeadband(Vx+offset, ControllConstants.DeadBand);
        double hdngHorz = MathUtil.applyDeadband(headingHorizontal, ControllConstants.DeadBand);
        double hdngVert = MathUtil.applyDeadband(headingVertical,  ControllConstants.DeadBand);


        if(trimLeft){
            offset = ControllConstants.trimLeftOffset+offset;
        }else if(trimRight){
            offset = ControllConstants.trimRightOffset+offset;
        } else if (Panic) {
            offset = 0;
        }

        if(Panic){
            swerve.resetOdometry(ControllConstants.panicPose);
        }

        if(Lock) {
            swerve.lockPose();
        }

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX, vY, hdngHorz, hdngVert);

        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose2d(), 0.13, RobotContants.RobotMass, List.of(RobotContants.Chassis), swerve.getSwerveDriveConfig());

        io.run(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    }
}
