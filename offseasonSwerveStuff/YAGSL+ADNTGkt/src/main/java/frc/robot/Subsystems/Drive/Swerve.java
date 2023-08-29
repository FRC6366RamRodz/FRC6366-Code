// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Util.Constants.Swrv_STG;
import swervelib.SwerveController;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

/** Add your docs here. */
public class Swerve {
    private SwerveIO io;
    private SwerveSprkMx swerve;
    private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    private static String tableName = "limelight";
    private DoubleArrayEntry botPosEntry;
    private final IntegerEntry idEntry;
    private Pose2d botPose2d;

    public Swerve(SwerveIO io, SwerveSprkMx swerve){
        this.io = io;
        this.swerve = swerve;


        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);
        botPosEntry = limelight.getDoubleArrayTopic("botpose").getEntry(new double[] {});
        idEntry = limelight.getIntegerTopic("tid").getEntry(0);

    }

    public void PeriodicSwerve() {

        double[] botposeData = botPosEntry.get();
        if(botposeData.length !=6) {
            botPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));;
        } else {
            botPose2d = new Pose2d(botposeData[0], botposeData[1], Rotation2d.fromDegrees(botposeData[5]));
        }
        
        swerve.periodicTask();
        Logger.getInstance().recordOutput("Odometry", swerve.getPose());

        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive", inputs);

        
        Logger.getInstance().recordOutput("LimePose", botPose2d);



    }

    public void absoluteDrive(double Vx, double Vy, double headingHorizontal, double headingVertical, boolean isOpenLoop){
        double RobotMass = 53 * 0.453592; //lbs * kg per lbs
        Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(14)), RobotMass);

      double vX = MathUtil.applyDeadband(Vy, Swrv_STG.DED_BND);
      double vY = MathUtil.applyDeadband(Vx, Swrv_STG.DED_BND);
      double hdngHorz = MathUtil.applyDeadband(headingHorizontal, Swrv_STG.DED_BND);
      double hdngVert = MathUtil.applyDeadband(headingVertical, Swrv_STG.DED_BND);

      ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX, vY, hdngHorz, hdngVert);

      Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
      translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(), 0.13, RobotMass, List.of(CHASSIS), swerve.getSwerveDriveConfig());

      io.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    }
}
