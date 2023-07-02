// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Util.Constants.DT_SET;

/** Add your docs here. */
public class SwerveDrive {
    private SwerveDriveIO io;


    public SwerveDrive(SwerveDriveIO io) {
        this.io = io;
    }


    public void teleopDrive(double vX, double vY, double omega, boolean driveMode, boolean isOpenLoop) {
        double xVelocity  = Math.pow(vX, 3);
        double yVelocity  = Math.pow(vY, 3);
        double angVelocity = Math.pow(omega, 3);

        io.runSwerve(new Translation2d(xVelocity * DT_SET.MX_SPD, yVelocity * DT_SET.MX_SPD), angVelocity*DT_SET.MX_SPD, driveMode, isOpenLoop);
    }
}
