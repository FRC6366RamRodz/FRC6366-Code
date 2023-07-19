// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.Constants.Swrv_STG;
import swervelib.SwerveController;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

/** Add your docs here. */
public class Swerve {
    private SwerveIO io;
    private SwerveSprkMx swerve;
    private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    public Swerve(SwerveIO io, SwerveSprkMx swerve){
        this.io = io;
        this.swerve = swerve;
    }

    public void PeriodicSwerve() {
        swerve.periodicTask();
        Logger.getInstance().recordOutput("Odometry", swerve.getPose());

        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive", inputs);

    }

    public void absoluteDrive(double Vx, double Vy, double headingHorizontal, double headingVertical, boolean isOpenLoop){
        double RobotMass = 40 * 0.453592; //lbs * kg per lbs
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
