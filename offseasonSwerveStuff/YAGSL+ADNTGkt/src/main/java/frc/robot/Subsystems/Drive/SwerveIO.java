// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        public double drivePositionRad = 0.0;
    }

    public default void updateInputs(SwerveIOInputs inputs) {
    }

    public default void runSwerve(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    }
}
